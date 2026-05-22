#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <netinet/in.h>

#define REF_FRAME_HEADER        0xAA
#define CALC_REF_FRAME_HEADER   0xCA
#define ACTUAL_FRAME_HEADER     0xCE

typedef enum
{
    ID_FRONT_LEFT_WHEEL = 0x01,
    ID_FRONT_RIGHT_WHEEL,
    ID_REAR_LEFT_WHEEL,
    ID_REAR_RIGHT_WHEEL,
    ID_PC_MATLAB,
    ID_VCU
} DeviceID;

typedef struct
{
    uint8_t   header;
    uint8_t   id;
    uint16_t  angleRef;
    uint16_t  speedRef;
    uint8_t   brakeRef;
    uint8_t   seq;
} __attribute__((packed)) vcu_ref_frame_t;

typedef struct
{
    uint8_t   header;
    uint8_t   id;
    uint16_t  angleRef_FL;
    uint16_t  angleRef_FR;
    uint16_t  angleRef_RL;
    uint16_t  angleRef_RR;
    uint16_t  VxRef;
    uint8_t   seq;
} __attribute__((packed)) matlab_ref_frame_t;

typedef struct
{
    uint8_t   header;
    uint8_t   id;
    uint16_t  angle_FL;
    uint16_t  angle_FR;
    uint16_t  angle_RL;
    uint16_t  angle_RR;
    uint16_t  speed_FL;
    uint16_t  speed_FR;
    uint16_t  speed_RL;
    uint16_t  speed_RR;
    uint8_t   seq;
} __attribute__((packed)) calc_frame_t;

typedef struct
{
    uint8_t   header;
    uint8_t   id;
    uint16_t  angle;
    uint16_t  speed;
    uint8_t   seq;
} __attribute__((packed)) corner_frame_t;

typedef struct
{
    int sockfd;
    struct sockaddr_in addr;
} udp_sock_t;

// ---------------- Frame Helpers ----------------
void fill_vcu_ref_frame(vcu_ref_frame_t *f, uint8_t id, int16_t angle, uint16_t speed, uint8_t brake, uint8_t seq);
void fill_ref_calc_frame(calc_frame_t *f, uint8_t id,
                            uint16_t angle_FL, uint16_t angle_FR,
                            uint16_t angle_RL, uint16_t angle_RR,
                            uint16_t speed_FL, uint16_t speed_FR,
                            uint16_t speed_RL, uint16_t speed_RR,
                            uint8_t seq);
void fill_corner_frame(corner_frame_t *f, uint8_t header, int16_t angle, uint16_t speed, uint8_t seq);

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
