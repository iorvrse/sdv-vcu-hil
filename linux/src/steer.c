#include "steer.h"
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct steer_context
{
    libusb_context *usb_ctx;
    libusb_device_handle *devh;
    struct libusb_transfer *tf;
    uint8_t usb_buffer[BUFFER_SIZE];
    
    steer_data_t latest_data;
    int keep_reading;
};

static void LIBUSB_CALL steer_callback(struct libusb_transfer *transfer)
{
    steer_ctx_t *ctx = (steer_ctx_t*)transfer->user_data;

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED && transfer->actual_length >= 8)
    {
        const uint8_t *data = transfer->buffer;
        
        uint8_t brake_raw  = data[4];
        uint8_t accel_raw  = data[5];
        int16_t steer_raw  = (int16_t)((data[7] << 8) | data[6]);

        ctx->latest_data.brake = (uint8_t)((brake_raw * BRAKE_MAX_SCALED) / BRAKE_MAX_RAW);
        ctx->latest_data.accel = (uint16_t)((accel_raw * ACCEL_MAX_SCALED) / ACCEL_MAX_RAW);
        ctx->latest_data.steer = (int16_t)((steer_raw * STEER_MAX_SCALED) / STEER_MAX_RAW);
    }

    // Resubmit transfer to keep the stream alive
    if (ctx->keep_reading)
    {
        libusb_submit_transfer(transfer);
    }
}

steer_ctx_t* steer_init(void)
{
    steer_ctx_t *ctx = (steer_ctx_t *)calloc(1, sizeof(steer_ctx_t));
    if (!ctx) return NULL;

    ctx->keep_reading = 0;

    int r = libusb_init(&ctx->usb_ctx);
    if (r < 0)
    {
        free(ctx);
        return NULL;
    }

    ctx->devh = libusb_open_device_with_vid_pid(ctx->usb_ctx, VENDOR_ID, PRODUCT_ID);
    if (!ctx->devh)
    {
        libusb_exit(ctx->usb_ctx);
        free(ctx);
        return NULL;
    }

    libusb_set_auto_detach_kernel_driver(ctx->devh, 1);

    r = libusb_claim_interface(ctx->devh, 0);
    if (r < 0)
    {
        libusb_close(ctx->devh);
        libusb_exit(ctx->usb_ctx);
        free(ctx);
        return NULL;
    }

    ctx->tf = libusb_alloc_transfer(0);
    if (!ctx->tf)
    {
        steer_close(ctx);
        return NULL;
    }

    return ctx;
}

int steer_start(steer_ctx_t *ctx)
{
    if (!ctx || !ctx->tf || !ctx->devh) return -1;

    ctx->keep_reading = 1;

    libusb_fill_interrupt_transfer(ctx->tf, ctx->devh, ENDPOINT_IN,
                                   ctx->usb_buffer, BUFFER_SIZE,
                                   steer_callback, ctx, TIMEOUT_MS);

    int r = libusb_submit_transfer(ctx->tf);
    if (r < 0)
    {
        ctx->keep_reading = 0;
        return r;
    }
    return 0;
}

int steer_process_events(steer_ctx_t *ctx, int timeout_ms)
{
    if (!ctx || !ctx->usb_ctx) return -1;

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    return libusb_handle_events_timeout_completed(ctx->usb_ctx, &tv, NULL);
}

void steer_get_latest_data(steer_ctx_t *ctx, steer_data_t *out)
{
    if (!ctx || !out) return;
    *out = ctx->latest_data;
}

void steer_close(steer_ctx_t *ctx)
{
    if (!ctx) return;

    ctx->keep_reading = 0;

    if (ctx->tf)
    {
        libusb_cancel_transfer(ctx->tf);
        struct timeval tv = {0, 100000};
        libusb_handle_events_timeout_completed(ctx->usb_ctx, &tv, NULL);
        libusb_free_transfer(ctx->tf);
    }

    if (ctx->devh)
    {
        libusb_release_interface(ctx->devh, 0);
        libusb_close(ctx->devh);
    }

    if (ctx->usb_ctx)
    {
        libusb_exit(ctx->usb_ctx);
    }

    free(ctx);
}