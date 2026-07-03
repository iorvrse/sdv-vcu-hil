#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <netinet/in.h>
typedef struct
{
    int sockfd;
    struct sockaddr_in addr;
} udp_sock_t;

// Initialize UDP sockets
int udp_rx_init(udp_sock_t *rx, uint16_t port_rx);
int udp_tx_init(udp_sock_t *tx, uint16_t src_port, int is_broadcast, const char* target_ip);

// Setup target address (corner or laptop)
void udp_tx_set_target(udp_sock_t *tx, const char *ip, uint16_t port);

// Send frame
int udp_comm_send(udp_sock_t *tx, const struct sockaddr_in *addr, const void *data, size_t len, int flag);

// Receive frame
ssize_t udp_comm_recv(udp_sock_t *rx, void *buf, size_t len, struct sockaddr_in *src, int flag);

// Cleanup
void udp_comm_close(int *sock);

#endif // COMM_H
