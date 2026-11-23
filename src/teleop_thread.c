#include "teleop.h"

#include "cmd_queue.h"
#include "robot_config.h"
#include "safety.h"
#include "scheduler_utils.h"
#include "shared_state.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

enum { UDP_HEARTBEAT = 1, UDP_MOVE, UDP_HEIGHT, UDP_ESTOP, UDP_RESET };
enum { DATAGRAM_SIZE = 20, TELEOP_POLL_MS = 20, MAX_PACKETS_PER_POLL = 32 };

typedef struct {
    unsigned opcode;
    uint32_t sequence;
    XgoCmdMsg command;
} Datagram;

static uint32_t read_u32(const unsigned char *bytes)
{
    uint32_t value;
    memcpy(&value, bytes, sizeof(value));
    return ntohl(value);
}

static int64_t read_i32(const unsigned char *bytes)
{
    uint32_t value = read_u32(bytes);
    return value <= INT32_MAX ? value : (int64_t)value - INT64_C(4294967296);
}

static bool parse_datagram(const unsigned char *buffer, size_t length, Datagram *packet)
{
    if (length != DATAGRAM_SIZE || memcmp(buffer, "XGO2", 4) != 0 ||
        buffer[4] != 1 || buffer[6] != 0 || buffer[7] != 0)
        return false;

    unsigned opcode = buffer[5];
    int64_t first = read_i32(buffer + 12);
    int64_t second = read_i32(buffer + 16);
    *packet = (Datagram){ .opcode = opcode, .sequence = read_u32(buffer + 8) };
    switch (opcode) {
    case UDP_MOVE:
        xgo_cmd_init(&packet->command, XGO_CMD_MOVE, 0);
        packet->command.vx = (float)first / 1000.0f;
        packet->command.wz = (float)second / 1000.0f;
        return packet->command.vx >= -MAX_FORWARD_SPEED &&
               packet->command.vx <= MAX_FORWARD_SPEED &&
               packet->command.wz >= -MAX_TURN_SPEED &&
               packet->command.wz <= MAX_TURN_SPEED;
    case UDP_HEIGHT:
        xgo_cmd_init(&packet->command, XGO_CMD_HEIGHT, 0);
        packet->command.height_z = (float)first;
        return first >= MIN_HEIGHT_MM && first <= MAX_HEIGHT_MM && second == 0;
    case UDP_HEARTBEAT:
    case UDP_ESTOP:
    case UDP_RESET:
        return first == 0 && second == 0;
    default:
        return false;
    }
}

static void handle_datagram(Datagram *packet, uint64_t now)
{
    switch (packet->opcode) {
    case UDP_HEARTBEAT:
        shared_state_operator_seen(now);
        break;
    case UDP_ESTOP:
        shared_state_estop(FAULT_OPERATOR_ESTOP, now);
        break;
    case UDP_RESET:
        shared_state_request_reset();
        break;
    default: {
        RobotStateSnapshot snapshot = shared_state_snapshot();
        if (safety_motion_allowed(&snapshot, monotonic_ns())) {
            packet->command.timestamp_ns = now;
            QueuedCommand queued = { packet->command, snapshot.generation };
            cmd_queue_push(&queued);
        }
        break;
    }
    }
}

int teleop_open(Teleop *teleop, const char *bind_ip, unsigned bind_port,
                const char *operator_ip, unsigned operator_port)
{
    struct sockaddr_in bind_address = { .sin_family = AF_INET, .sin_port = htons(bind_port) };
    teleop->operator_address = (struct sockaddr_in){
        .sin_family = AF_INET, .sin_port = htons(operator_port)
    };
    if (inet_pton(AF_INET, bind_ip, &bind_address.sin_addr) != 1 ||
        inet_pton(AF_INET, operator_ip, &teleop->operator_address.sin_addr) != 1) {
        errno = EINVAL;
        return -1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1)
        return -1;
    if (fcntl(fd, F_SETFD, FD_CLOEXEC) == -1 ||
        fcntl(fd, F_SETFL, O_NONBLOCK) == -1 ||
        bind(fd, (struct sockaddr *)&bind_address, sizeof(bind_address)) == -1) {
        int error = errno;
        close(fd);
        errno = error;
        return -1;
    }
    teleop->socket_fd = fd;
    return 0;
}

static bool receive_packets(const Teleop *teleop, bool *seen_sequence, uint32_t *last_sequence)
{
    for (unsigned i = 0; i < MAX_PACKETS_PER_POLL; ++i) {
        /* One extra byte makes oversize packets fail the exact length check. */
        unsigned char buffer[DATAGRAM_SIZE + 1];
        struct sockaddr_in sender;
        socklen_t length = sizeof(sender);
        ssize_t size = recvfrom(teleop->socket_fd, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&sender, &length);
        if (size < 0)
            return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
        if (length != sizeof(sender) || sender.sin_family != AF_INET ||
            sender.sin_addr.s_addr != teleop->operator_address.sin_addr.s_addr ||
            sender.sin_port != teleop->operator_address.sin_port)
            continue;
        Datagram packet;
        if (!parse_datagram(buffer, (size_t)size, &packet) ||
            (*seen_sequence && !sequence_newer(packet.sequence, *last_sequence)))
            continue;
        *seen_sequence = true;
        *last_sequence = packet.sequence;
        handle_datagram(&packet, monotonic_ns());
    }
    return true;
}

void *teleop_worker(void *argument)
{
    Teleop *teleop = argument;
    struct pollfd input = { .fd = teleop->socket_fd, .events = POLLIN };
    bool seen_sequence = false;
    uint32_t last_sequence = 0;
    while (shared_state_snapshot().safety != SAFETY_SHUTDOWN) {
        int result = poll(&input, 1, TELEOP_POLL_MS);
        if (result < 0 && errno == EINTR)
            continue;
        if (result < 0 || (input.revents & (POLLERR | POLLHUP | POLLNVAL)) ||
            ((input.revents & POLLIN) && !receive_packets(teleop, &seen_sequence, &last_sequence))) {
            fputs("Teleop socket failed\n", stderr);
            shared_state_fail();
            break;
        }
    }
    return NULL;
}

void teleop_close(Teleop *teleop)
{
    close(teleop->socket_fd);
}
