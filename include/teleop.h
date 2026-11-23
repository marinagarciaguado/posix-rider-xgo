#ifndef XGO_TELEOP_H
#define XGO_TELEOP_H

#include <netinet/in.h>

typedef struct {
    int socket_fd;
    struct sockaddr_in operator_address;
} Teleop;

int teleop_open(Teleop *teleop, const char *bind_ip, unsigned bind_port,
                const char *operator_ip, unsigned operator_port);
void *teleop_worker(void *argument);
void teleop_close(Teleop *teleop);

#endif
