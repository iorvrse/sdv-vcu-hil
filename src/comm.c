#include "comm.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

void fill_vcu_ref_frame(vcu_ref_frame_t *f, uint8_t id,
                        int16_t angleRef, uint16_t speedRef, uint8_t brakeRef,
                        uint8_t seq)
{
    f->header   = REF_FRAME_HEADER;
    f->id       = id;
    f->angleRef = angleRef;
    f->speedRef = speedRef;
    f->brakeRef = brakeRef;
    f->seq      = seq;
}

void fill_ref_calc_frame(calc_frame_t *f, uint8_t id,
                            uint16_t angle_FL, uint16_t angle_FR,
                            uint16_t angle_RL, uint16_t angle_RR,
                            uint16_t speed_FL, uint16_t speed_FR,
                            uint16_t speed_RL, uint16_t speed_RR,
                            uint8_t seq)
{
    f->header   = CALC_REF_FRAME_HEADER;
    f->id       = id;
    f->angle_FL = angle_FL;
    f->angle_FR = angle_FR;
    f->angle_RL = angle_RL;
    f->angle_RR = angle_RR;
    f->speed_FL = speed_FL;
    f->speed_FR = speed_FR;
    f->speed_RL = speed_RL;
    f->speed_RR = speed_RR;
    f->seq      = seq;
}

void fill_corner_frame(corner_frame_t *f, uint8_t header,
                        int16_t angle, uint16_t speed,
                        uint8_t seq)
{
    f->header   = header;
    f->id       = ID_VCU;
    f->angle    = angle;
    f->speed    = speed;
    f->seq      = seq;
}

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