#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <netinet/in.h>
#include <pthread.h>
#include <time.h>

// Library Utama
#include "comm.h"
#include "steer.h"
#include "Torque_Vectoring.h"

// ==========================================
// KONFIGURASI JARINGAN
// ==========================================
#define CORNER_COUNT 4

#define PORT_RX           5055
#define PORT_TX_PC        5060
#define PORT_LAPTOP       4093

// Alamat IP 4 Roda (FL, FR, RL, RR)
extern const char *IP_CORNERS[CORNER_COUNT];
extern const int PORT_CORNERS[CORNER_COUNT];
extern const int CornerHeaders[CORNER_COUNT];

typedef struct __attribute__((packed)) 
{
    uint8_t  Header;
    uint8_t  id;
    uint16_t Trq;
    uint16_t Vx;
    uint16_t Vx_Wheel;
    uint16_t Vx_Thrl;
    uint16_t Steer_Wheel;
    uint32_t Mzd;
    uint16_t seq;
} VCUToCorner;

typedef struct __attribute__((packed)) {
    uint8_t Header;
    uint8_t id;
    uint16_t Vx_Thrl;
    uint16_t Vx;
    uint16_t Vy;
    uint16_t angWheel_FL;
    uint16_t angWheel_FR;
    uint16_t angWheel_RL;
    uint16_t angWheel_RR;
    uint16_t yawrate;
    uint32_t Fy_FL;
    uint32_t Fy_FR;
    uint32_t Fy_RL;
    uint32_t Fy_RR;
    uint32_t Fz_FL;
    uint32_t Fz_FR;
    uint32_t Fz_RL;
    uint32_t Fz_RR;
    uint8_t seq;
} Matlab2VCU;

typedef struct __attribute__((packed)) {
    uint8_t Header;
    uint8_t id;
    uint16_t Trq;
    uint8_t seq;
} VCU2Corner_TV;

typedef struct __attribute__((packed)) {
    uint8_t Header;
    uint8_t id;
    uint16_t Trq_FL;
    uint16_t Trq_FR;
    uint16_t Trq_RL;
    uint16_t Trq_RR;
    uint8_t seq;
} VCU2Matlab_TV;

// ==========================================
// DEKLARASI GLOBAL
// ==========================================
extern udp_sock_t tx_corner[CORNER_COUNT];
extern udp_sock_t tx_matlab;

// Fungsi Utilitas
void try_set_affinity(pthread_t thread, int cpu_id);
void try_set_realtime(pthread_t thread, int priority);

#endif // MAIN_H