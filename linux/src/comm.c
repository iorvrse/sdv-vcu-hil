#include "comm.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

int udp_tx_init(udp_sock_t *tx, uint16_t src_port, int is_broadcast, const char* target_ip) 
{
    memset(tx, 0, sizeof(*tx));
    tx->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx->sockfd < 0) return -1;

    int opt = 1;

    setsockopt(tx->sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (is_broadcast)
    {
        setsockopt(tx->sockfd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    }

    struct sockaddr_in local = {0};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port = htons(src_port);

    if (bind(tx->sockfd, (struct sockaddr *)&local, sizeof(local)) < 0)
    {
        perror("bind tx");
        return -1;
    }

    // Set Target IP secara dinamis
    if (target_ip != NULL)
    {
        inet_aton(target_ip, &tx->addr.sin_addr);
    }

    int snd = 256 * 1024;
    setsockopt(tx->sockfd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof(snd));

    return 0;
}

int udp_rx_init(udp_sock_t *rx, uint16_t port_rx)
{
    rx->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx->sockfd < 0) return -1;

    memset(&rx->addr, 0, sizeof(rx->addr));
    rx->addr.sin_family = AF_INET;
    rx->addr.sin_port = htons(port_rx);
    rx->addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(rx->sockfd, (struct sockaddr *)&rx->addr, sizeof(rx->addr)) < 0)
    {
        perror("bind rx");
        return -1;
    }

    int flags = fcntl(rx->sockfd, F_GETFL, 0);
    fcntl(rx->sockfd, F_SETFL, flags | O_NONBLOCK);

    return 0;
}

void udp_tx_set_target(udp_sock_t *tx, const char *ip, uint16_t port)
{
    memset(&tx->addr, 0, sizeof(tx->addr));
    tx->addr.sin_family = AF_INET;
    tx->addr.sin_port = htons(port);
    inet_aton(ip, &tx->addr.sin_addr);
}

int udp_comm_send(udp_sock_t *tx, const struct sockaddr_in *addr, const void *data, size_t len, int flag)
{
    return sendto(tx->sockfd, data, len, flag, (const struct sockaddr*)addr, sizeof(*addr));
}

ssize_t udp_comm_recv(udp_sock_t *rx, void *buf, size_t len, struct sockaddr_in *src, int flag)
{
    socklen_t addrlen = sizeof(*src);
    return recvfrom(rx->sockfd, buf, len, flag, (struct sockaddr*)src, &addrlen);
}

void udp_comm_close(int *sock)
{
    if (sock != NULL) 
    {
        if (*sock >= 0) 
        {
            close(*sock);
            *sock = -1; 
        }
    }
}