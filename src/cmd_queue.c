#include "cmd_queue.h"

#include "robot_config.h"
#include "scheduler_utils.h"

#include <errno.h>
#include <semaphore.h>

static struct {
    pthread_mutex_t lock;
    sem_t available;
    sem_t free_slots;
    QueuedCommand entries[CMD_QUEUE_CAPACITY];
    unsigned head;
    unsigned tail;
    uint64_t minimum_generation;
} queue;

int cmd_queue_init(void)
{
    int error = init_shared_mutex(&queue.lock);
    if (error != 0)
        return error;
    if (sem_init(&queue.available, 0, 0) == -1) {
        error = errno;
        pthread_mutex_destroy(&queue.lock);
        return error;
    }
    if (sem_init(&queue.free_slots, 0, CMD_QUEUE_CAPACITY) == -1) {
        error = errno;
        sem_destroy(&queue.available);
        pthread_mutex_destroy(&queue.lock);
        return error;
    }
    queue.head = queue.tail = 0;
    queue.minimum_generation = 0;
    return 0;
}

static bool take(sem_t *semaphore)
{
    int result;
    do {
        result = sem_trywait(semaphore);
    } while (result == -1 && errno == EINTR);
    return result == 0;
}

static QueuedCommand remove_head(void)
{
    QueuedCommand command = queue.entries[queue.head];
    queue.head = (queue.head + 1) % CMD_QUEUE_CAPACITY;
    sem_post(&queue.free_slots);
    return command;
}

bool cmd_queue_push(const QueuedCommand *command)
{
    pthread_mutex_lock(&queue.lock);
    bool inserted = command->generation >= queue.minimum_generation &&
                    take(&queue.free_slots);
    if (inserted) {
        queue.entries[queue.tail] = *command;
        queue.tail = (queue.tail + 1) % CMD_QUEUE_CAPACITY;
        sem_post(&queue.available);
    }
    pthread_mutex_unlock(&queue.lock);
    return inserted;
}

bool cmd_queue_pop(QueuedCommand *command)
{
    pthread_mutex_lock(&queue.lock);
    bool available = take(&queue.available);
    if (available)
        *command = remove_head();
    pthread_mutex_unlock(&queue.lock);
    return available;
}

void cmd_queue_discard_before(uint64_t generation)
{
    /* Reserve and release slots under the same lock as push and pop. */
    pthread_mutex_lock(&queue.lock);
    queue.minimum_generation = generation;
    while (take(&queue.available)) {
        if (queue.entries[queue.head].generation >= generation) {
            sem_post(&queue.available);
            break;
        }
        remove_head();
    }
    pthread_mutex_unlock(&queue.lock);
}

void cmd_queue_destroy(void)
{
    sem_destroy(&queue.free_slots);
    sem_destroy(&queue.available);
    pthread_mutex_destroy(&queue.lock);
}
