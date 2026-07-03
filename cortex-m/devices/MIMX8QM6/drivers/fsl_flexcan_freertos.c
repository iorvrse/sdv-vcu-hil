/*
 * Minimal FreeRTOS wrapper implementation for FlexCAN.
 */

#include "fsl_flexcan_freertos.h"

#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.flexcan_freertos"
#endif

#ifndef RX_MESSAGE_BUFFER_NUM
#define RX_MESSAGE_BUFFER_NUM (9)
#endif

#ifndef TX_MESSAGE_BUFFER_NUM
#define TX_MESSAGE_BUFFER_NUM (8)
#endif

static void FLEXCAN_RTOS_Callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    flexcan_rtos_handle_t *rtos_handle = (flexcan_rtos_handle_t *)userData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (status)
    {
        case kStatus_FLEXCAN_RxIdle: 
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
                xSemaphoreGiveFromISR(rtos_handle->rxSemaphore, &xHigherPriorityTaskWoken);
            }
            break;
            
        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                xSemaphoreGiveFromISR(rtos_handle->txSemaphore, &xHigherPriorityTaskWoken);
            }
            break;

        default:
            break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Initialize RTOS wrapper and register callback */
status_t FLEXCAN_RTOS_Init(flexcan_rtos_handle_t *handle, flexcan_config_t *cfg, uint32_t sourceClock_Hz)
{
    if ((handle == NULL) || (handle->t_handle == NULL) || (handle->base == NULL))
    {
        return kStatus_InvalidArgument;
    }

    handle->mutex = xSemaphoreCreateMutex();
    if (handle->mutex == NULL)
    {
        return kStatus_Fail;
    }
    
    handle->txSemaphore = xSemaphoreCreateBinary();
    if (handle->txSemaphore == NULL)
    {
        vSemaphoreDelete(handle->mutex);
        return kStatus_Fail;
    }
    
    handle->rxSemaphore = xSemaphoreCreateBinary();
    if (handle->rxSemaphore == NULL)
    {
        vSemaphoreDelete(handle->mutex);
        vSemaphoreDelete(handle->txSemaphore);
        return kStatus_Fail;
    } 

    FLEXCAN_Init(handle->base, cfg, sourceClock_Hz);
    FLEXCAN_TransferCreateHandle(handle->base, handle->t_handle, FLEXCAN_RTOS_Callback, (void *)handle);

    return kStatus_Success;
}

/* Deinit — delete RTOS objects */
status_t FLEXCAN_RTOS_Deinit(flexcan_rtos_handle_t *handle)
{
    FLEXCAN_Deinit(handle->base);
    vSemaphoreDelete(handle->txSemaphore);
    vSemaphoreDelete(handle->rxSemaphore);
    vSemaphoreDelete(handle->mutex);

    return kStatus_Success;
}

/* Blocking send */
status_t FLEXCAN_RTOS_Send(flexcan_rtos_handle_t *handle, flexcan_mb_transfer_t *pMbXfer, TickType_t timeout_ticks)
{
    status_t status;

    if ((handle == NULL) || (pMbXfer == NULL))
    {
        return kStatus_InvalidArgument;
    }

    /* Acquire API lock before starting the transfer */
    if (xSemaphoreTake(handle->mutex, timeout_ticks) != pdTRUE)
    {
        return kStatus_Timeout;
    }

    /* Start non-blocking transfer */
    status = FLEXCAN_TransferSendNonBlocking(handle->base, handle->t_handle, pMbXfer);
    if (status != kStatus_Success)
    {
        (void)xSemaphoreGive(handle->mutex);
        return status;
    }

    if (xSemaphoreTake(handle->txSemaphore, timeout_ticks) != pdTRUE)
    {
        status = kStatus_Timeout;
    }

    (void)xSemaphoreGive(handle->mutex);

    return status;
}

/* Blocking receive */
status_t FLEXCAN_RTOS_Receive(flexcan_rtos_handle_t *handle, flexcan_mb_transfer_t *pMbXfer, TickType_t timeout_ticks)
{
    status_t status;

    if ((handle == NULL) || (pMbXfer == NULL))
    {
        return kStatus_InvalidArgument;
    }

    if (xSemaphoreTake(handle->mutex, timeout_ticks) != pdTRUE)
    {
        return kStatus_Timeout;
    }

    status = FLEXCAN_TransferReceiveNonBlocking(handle->base, handle->t_handle, pMbXfer);
    if (status != kStatus_Success)
    {
        (void)xSemaphoreGive(handle->mutex);
        return status;
    }

    if (xSemaphoreTake(handle->rxSemaphore, timeout_ticks) != pdTRUE)
    {
        status = kStatus_Timeout;
    } 
    else
    {
        status = kStatus_Success;
    }

    (void)xSemaphoreGive(handle->mutex);

    return status;
}
