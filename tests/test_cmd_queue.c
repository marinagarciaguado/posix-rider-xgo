#include "cmd_queue.h"
#include "robot_config.h"

#include <assert.h>
#include <pthread.h>
#include <sched.h>

enum { COMMANDS = 10000 };

static void *produce_commands(void *unused)
{
    (void)unused;
    for (unsigned i = 0; i < COMMANDS; ++i) {
        QueuedCommand command = { .message.sequence = i, .generation = 2 };
        while (!cmd_queue_push(&command))
            sched_yield();
    }
    return NULL;
}

int main(void)
{
    assert(cmd_queue_init() == 0);
    QueuedCommand command = { 0 };
    assert(!cmd_queue_pop(&command));
    for (unsigned i = 0; i < CMD_QUEUE_CAPACITY; ++i) {
        command.message.sequence = i;
        assert(cmd_queue_push(&command));
    }
    assert(!cmd_queue_push(&command));
    for (unsigned i = 0; i < CMD_QUEUE_CAPACITY; ++i) {
        assert(cmd_queue_pop(&command));
        assert(command.message.sequence == i);
    }
    assert(!cmd_queue_pop(&command));

    assert(cmd_queue_push(&command));
    QueuedCommand current = { .generation = 1 };
    assert(cmd_queue_push(&current));
    cmd_queue_discard_before(1);
    assert(!cmd_queue_push(&command));
    assert(cmd_queue_pop(&command));
    assert(command.generation == 1);
    assert(!cmd_queue_pop(&command));

    pthread_t producer;
    assert(pthread_create(&producer, NULL, produce_commands, NULL) == 0);
    for (unsigned i = 0; i < COMMANDS; ++i) {
        cmd_queue_discard_before(2);
        while (!cmd_queue_pop(&command))
            sched_yield();
        assert(command.generation == 2);
        assert(command.message.sequence == i);
    }
    assert(pthread_join(producer, NULL) == 0);
    assert(!cmd_queue_pop(&command));
    cmd_queue_destroy();
    return 0;
}
