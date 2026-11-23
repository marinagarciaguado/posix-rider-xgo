#include "process_manager.h"

#include "robot_config.h"
#include "scheduler_utils.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static int bridge_environment(void)
{
    static const struct {
        const char *name;
        double value;
    } defaults[] = {
        { "XGO_MQ_RECEIVE_TIMEOUT_S", IPC_PERIOD_MS / 1000.0 },
        { "XGO_SUPERVISOR_TIMEOUT_S", TELEMETRY_TIMEOUT_MS / 1000.0 },
        { "XGO_STARTUP_GRACE_S", BRIDGE_STARTUP_GRACE_MS / 1000.0 },
        { "XGO_MAX_COMMAND_AGE_S", COMMAND_MAX_AGE_MS / 1000.0 },
        { "XGO_MQ_SEND_TIMEOUT_S", IPC_PERIOD_MS / 1000.0 },
        { "XGO_TELEMETRY_PERIOD_S", BRIDGE_TELEMETRY_MS / 1000.0 },
        { "XGO_TELEMETRY_MAX_AGE_S", TELEMETRY_TIMEOUT_MS / 1000.0 },
        { "XGO_RECOVERY_MAX_PITCH_DEG", RECOVERY_TILT_DEG },
        { "XGO_RECOVERY_MAX_ROLL_DEG", RECOVERY_TILT_DEG },
        { "XGO_RECOVERY_MIN_BATTERY_PCT", RECOVERY_BATTERY_PCT },
    };
    for (size_t i = 0; i < sizeof(defaults) / sizeof(defaults[0]); ++i) {
        char value[32];
        snprintf(value, sizeof(value), "%.6f", defaults[i].value);
        if (setenv(defaults[i].name, value, 0) == -1)
            return -1;
    }

    return 0;
}

int process_start(BridgeProcess *child, const char *python, bool mock, bool telemetry_only)
{
    char supervisor_pid[32];
    snprintf(supervisor_pid, sizeof(supervisor_pid), "%ld", (long)getpid());
    if (bridge_environment() == -1 ||
        setenv("XGO_SUPERVISOR_PID", supervisor_pid, 1) == -1 ||
        setenv("XGO_TELEMETRY_ONLY", telemetry_only ? "1" : "0", 1) == -1)
        return -1;
    pid_t pid = fork();
    if (pid < 0)
        return -1;
    if (pid == 0) {
        sigset_t unblocked;
        sigemptyset(&unblocked);
        if (sigprocmask(SIG_SETMASK, &unblocked, NULL) == -1)
            _exit(126);
        const char *module = mock ? "tests.mock_xgo_bridge" : "bridge.xgo_bridge";
        char *const arguments[] = { (char *)python, "-B", "-m", (char *)module, NULL };
        execvp(python, arguments);
        perror("Executing Python bridge");
        _exit(127);
    }
    child->pid = pid;
    return 0;
}

/* Main is the only owner of waitpid. Return 1 if reaped, 0 if running. */
int process_poll(BridgeProcess *child)
{
    if (child->pid <= 0)
        return 1;
    pid_t result;
    do {
        result = waitpid(child->pid, &child->status, WNOHANG);
    } while (result == -1 && errno == EINTR);
    if (result == 0)
        return 0;
    if (result == child->pid) {
        child->pid = 0;
        return 1;
    }
    return -1;
}

static int wait_for_exit(BridgeProcess *child)
{
    uint64_t deadline = monotonic_ns() + CHILD_EXIT_TIMEOUT_MS * NS_PER_MS;
    const struct timespec delay = { .tv_nsec = 10000000L };
    do {
        int result = process_poll(child);
        if (result != 0)
            return result;
        nanosleep(&delay, NULL);
    } while (monotonic_ns() < deadline);
    return 0;
}

static int shutdown_result(const BridgeProcess *child, int result)
{
    if (result < 0)
        return -1;
    if (WIFEXITED(child->status) && WEXITSTATUS(child->status) == 0)
        return 0;
    errno = EIO;
    return -1;
}

int process_stop(BridgeProcess *child)
{
    int result = wait_for_exit(child);
    if (result != 0)
        return shutdown_result(child, result);

    if (kill(child->pid, SIGTERM) == -1 && errno != ESRCH)
        return -1;
    result = wait_for_exit(child);
    if (result != 0)
        return shutdown_result(child, result);

    fputs("Bridge failed to exit after SIGTERM; sending SIGKILL\n", stderr);
    if (kill(child->pid, SIGKILL) == -1 && errno != ESRCH)
        return -1;
    result = wait_for_exit(child);
    if (result == 0)
        errno = ETIMEDOUT;
    /* Forced termination is a failed graceful shutdown, even if reaping succeeds. */
    if (result > 0)
        errno = ECANCELED;
    return -1;
}
