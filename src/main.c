#include "cmd_queue.h"
#include "ipc_bridge.h"
#include "metrics.h"
#include "process_manager.h"
#include "robot_config.h"
#include "safety.h"
#include "scheduler_utils.h"
#include "shared_state.h"
#include "teleop.h"

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>

typedef struct {
    const char *bind_ip;
    const char *operator_ip;
    const char *python;
    const char *metrics;
    unsigned port;
    unsigned operator_port;
    bool mock;
    bool hardware;
    bool telemetry_only;
    bool fifo;
} Options;

static void usage(FILE *output, const char *name)
{
    fprintf(output, "Usage: %s [--mock | --hardware] [--scheduler other|fifo]\n"
                    "       [--bind-ip IPv4] [--port PORT] [--operator-ip IPv4]\n"
                    "       [--operator-port PORT] [--python EXE] [--metrics CSV]\n"
                    "       [--telemetry-only]\n"
                    "       The default mode is mock. Run from the project root.\n", name);
}

static bool parse_port(const char *value, unsigned *port)
{
    char *end;
    errno = 0;
    unsigned long number = strtoul(value, &end, 10);
    if (value == end || *end != '\0' || errno != 0 || number == 0 || number > 65535)
        return false;
    *port = (unsigned)number;
    return true;
}

static bool parse_options(int argc, char **argv, Options *options)
{
    enum { BIND_IP = 1, PORT, OPERATOR_IP, OPERATOR_PORT, PYTHON, METRICS,
           SCHEDULER, MOCK, HARDWARE, TELEMETRY_ONLY };
    static const struct option arguments[] = {
        { "bind-ip", required_argument, NULL, BIND_IP },
        { "port", required_argument, NULL, PORT },
        { "operator-ip", required_argument, NULL, OPERATOR_IP },
        { "operator-port", required_argument, NULL, OPERATOR_PORT },
        { "python", required_argument, NULL, PYTHON },
        { "metrics", required_argument, NULL, METRICS },
        { "scheduler", required_argument, NULL, SCHEDULER },
        { "mock", no_argument, NULL, MOCK },
        { "hardware", no_argument, NULL, HARDWARE },
        { "telemetry-only", no_argument, NULL, TELEMETRY_ONLY },
        { "help", no_argument, NULL, 'h' },
        { 0 }
    };
    int option;
    while ((option = getopt_long(argc, argv, "h", arguments, NULL)) != -1) {
        switch (option) {
        case BIND_IP: options->bind_ip = optarg; break;
        case OPERATOR_IP: options->operator_ip = optarg; break;
        case PYTHON: options->python = optarg; break;
        case METRICS: options->metrics = optarg; break;
        case PORT:
            if (!parse_port(optarg, &options->port))
                return false;
            break;
        case OPERATOR_PORT:
            if (!parse_port(optarg, &options->operator_port))
                return false;
            break;
        case SCHEDULER:
            if (strcmp(optarg, "fifo") == 0)
                options->fifo = true;
            else if (strcmp(optarg, "other") == 0)
                options->fifo = false;
            else
                return false;
            break;
        case MOCK: options->mock = true; break;
        case HARDWARE: options->hardware = true; break;
        case TELEMETRY_ONLY: options->telemetry_only = true; break;
        case 'h': usage(stdout, argv[0]); exit(EXIT_SUCCESS);
        default: return false;
        }
    }
    return optind == argc && !(options->mock && options->hardware);
}

static int lifecycle_signals(sigset_t *shutdown_signals)
{
    sigemptyset(shutdown_signals);
    sigaddset(shutdown_signals, SIGINT);
    sigaddset(shutdown_signals, SIGTERM);
    sigset_t blocked = *shutdown_signals;
    sigaddset(&blocked, SIGRTMIN);
    return pthread_sigmask(SIG_BLOCK, &blocked, NULL);
}

static void report_child_exit(int status)
{
    if (WIFEXITED(status))
        fprintf(stderr, "Bridge exited with status %d\n", WEXITSTATUS(status));
    else if (WIFSIGNALED(status))
        fprintf(stderr, "Bridge exited on signal %d\n", WTERMSIG(status));
}

static int supervise(BridgeProcess *child, const sigset_t *signals)
{
    uint64_t startup_deadline = monotonic_ns() + STARTUP_TIMEOUT_MS * NS_PER_MS;
    const struct timespec poll_period = { .tv_nsec = 10000000L };
    bool ready = false;
    for (;;) {
        int signal_number = sigtimedwait(signals, NULL, &poll_period);
        if (signal_number == SIGINT || signal_number == SIGTERM)
            return EXIT_SUCCESS;
        if (signal_number == -1 && errno != EAGAIN && errno != EINTR) {
            perror("Waiting for shutdown signal");
            return EXIT_FAILURE;
        }

        int child_result = process_poll(child);
        if (child_result < 0) {
            perror("Monitoring bridge");
            return EXIT_FAILURE;
        }
        if (child_result > 0) {
            report_child_exit(child->status);
            shared_state_child_exited();
            return EXIT_FAILURE;
        }
        RobotStateSnapshot snapshot = shared_state_snapshot();
        uint64_t now = monotonic_ns();
        if (!ready && snapshot.bridge_initialized) {
            ready = true;
            fputs("Bridge READY with valid telemetry\n", stderr);
        }
        if (!ready && now >= startup_deadline) {
            fputs("Bridge READY timed out\n", stderr);
            return EXIT_FAILURE;
        }
        if (snapshot.runtime_failed)
            return EXIT_FAILURE;

    }
}

static int run_supervisor(const Options *options, const sigset_t *signals)
{
    IpcBridge ipc;
    Teleop teleop;
    BridgeProcess child = { 0 };
    bool queue_open = false, ipc_open = false, teleop_opened = false, metrics_open = false;
    pthread_t workers[3];
    bool use_fifo = options->fifo;
    size_t started_workers = 0;
    int result = EXIT_FAILURE;

    int error = shared_state_init();
    if (error != 0) {
        fprintf(stderr, "Initializing shared state: %s\n", strerror(error));
        return result;
    }

    error = cmd_queue_init();
    if (error != 0) {
        fprintf(stderr, "Initializing command queue: %s\n", strerror(error));
        goto cleanup;
    }
    queue_open = true;
    if (ipc_bridge_open(&ipc) == -1) {
        perror("Creating IPC queues (only one supervisor may own them)");
        goto cleanup;
    }
    ipc_open = true;
    if (teleop_open(&teleop, options->bind_ip, options->port,
                    options->operator_ip, options->operator_port) == -1) {
        perror("Opening teleop socket");
        goto cleanup;
    }
    teleop_opened = true;
    if (process_start(&child, options->python, !options->hardware,
                      options->telemetry_only) == -1) {
        perror("Starting bridge");
        goto cleanup;
    }
    error = metrics_start(options->metrics);
    if (error != 0) {
        fprintf(stderr, "Starting metrics: %s\n", strerror(error));
        goto cleanup;
    }
    metrics_open = true;

    const struct {
        void *(*run)(void *);
        void *argument;
    } worker_config[] = {
        { ipc_worker, &ipc },
        { safety_worker, &use_fifo },
        { teleop_worker, &teleop }
    };
    for (; started_workers < sizeof(worker_config) / sizeof(worker_config[0]); ++started_workers) {
        error = pthread_create(&workers[started_workers], NULL,
                               worker_config[started_workers].run,
                               worker_config[started_workers].argument);
        if (error != 0) {
            fprintf(stderr, "Starting worker: %s\n", strerror(error));
            goto cleanup;
        }
    }
    result = supervise(&child, signals);

cleanup:
    shared_state_shutdown();
    while (started_workers > 0)
        pthread_join(workers[--started_workers], NULL);

    if (child.pid > 0 && process_stop(&child) == -1) {
        perror("Stopping bridge");
        result = EXIT_FAILURE;
    }
    if (metrics_open && metrics_stop() == -1)
        result = EXIT_FAILURE;
    if (teleop_opened)
        teleop_close(&teleop);
    if (ipc_open)
        ipc_bridge_close(&ipc);
    if (queue_open)
        cmd_queue_destroy();
    shared_state_destroy();
    return result;
}

int main(int argc, char **argv)
{
    Options options = {
        .bind_ip = DEFAULT_BIND_ADDRESS, .operator_ip = DEFAULT_OPERATOR_IP,
        .python = "python3", .metrics = "logs/run_metrics.csv",
        .port = DEFAULT_TELEOP_PORT, .operator_port = DEFAULT_OPERATOR_PORT
    };
    if (!parse_options(argc, argv, &options)) {
        usage(stderr, argv[0]);
        return EXIT_FAILURE;
    }
    sigset_t signals;
    int error = lifecycle_signals(&signals);
    if (error != 0) {
        fprintf(stderr, "Blocking signals: %s\n", strerror(error));
        return EXIT_FAILURE;
    }
    if (strcmp(options.metrics, "logs/run_metrics.csv") == 0 &&
        mkdir("logs", 0755) == -1 && errno != EEXIST) {
        perror("Creating metrics directory");
        return EXIT_FAILURE;
    }
    fprintf(stderr, "Starting supervisor in %s mode%s\n", options.hardware ? "hardware" : "mock",
            options.telemetry_only ? " (telemetry only)" : "");
    return run_supervisor(&options, &signals);
}
