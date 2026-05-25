#ifndef STEER_H
#define STEER_H

#include <stdint.h>

#define VENDOR_ID   0x11ff
#define PRODUCT_ID  0x0511
#define ENDPOINT_IN 0x81
#define BUFFER_SIZE 64
#define TIMEOUT_MS  1000

#define BRAKE_MAX_RAW   255
#define ACCEL_MAX_RAW   255
#define STEER_MAX_RAW   32767

#define BRAKE_MAX_SCALED  100     // 0.0 – 1.0
#define ACCEL_MAX_SCALED  10000   // 0.0 – 100.0 km/h
#define STEER_MAX_SCALED  4000    // -40.0 – 40.0 degree

typedef struct
{
    uint8_t brake;   // 0–100 (resolution 0.01)
    uint16_t accel;   // 0–10000 (resolution 0.01)
    int16_t steer;   // 0–8000 (resolution 0.01)
} steer_data_t;

typedef struct steer_context steer_ctx_t;

steer_ctx_t* steer_init(void);
int          steer_start(steer_ctx_t *ctx);
int          steer_process_events(steer_ctx_t *ctx, int timeout_ms);
void         steer_get_latest_data(steer_ctx_t *ctx, steer_data_t *out);
void         steer_close(steer_ctx_t *ctx);

#endif // STEER_H