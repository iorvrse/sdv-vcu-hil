/*
 * Minimal FreeRTOS wrapper for FlexCAN transfer APIs
 */

#ifndef _FSL_FLEXCAN_FREERTOS_H_
#define _FSL_FLEXCAN_FREERTOS_H_

#include "fsl_flexcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name Driver version */
/*@{*/
#define FSL_FLEXCAN_FREERTOS_DRIVER_VERSION (MAKE_VERSION(1, 0, 0))
/*@}*/

/*!
 * @brief FlexCAN RTOS handle.
 */
typedef struct _flexcan_rtos_handle
{
    CAN_Type *base;
    flexcan_handle_t *t_handle;
    SemaphoreHandle_t txSemaphore;
    SemaphoreHandle_t rxSemaphore;
    SemaphoreHandle_t mutex;
} flexcan_rtos_handle_t;

/*!
 * @brief Initialize FlexCAN for RTOS usage.
 *
 * This:
 *  - stores pointers in rtos handle,
 *  - registers the RTOS callback via FLEXCAN_TransferCreateHandle(),
 *  - creates tx/rx Semaphore and the api mutex.
 *
 * @param handle pointer to RTOS handle (allocated by caller)
 * @param t_handle pointer to flexcan_handle_t (allocated by caller)
 * @param cfg pointer to flexcan_rtos_config_t (const)
 * @param sourceClock_Hz flexcan clock source
 * @return status_t (kStatus_Success on success, other if creation failed)
 */
status_t FLEXCAN_RTOS_Init(flexcan_rtos_handle_t *handle, flexcan_config_t *cfg, uint32_t sourceClock_Hz);
/*!
 * @brief Deinitialize RTOS wrapper (delete/free semaphores and mutexes).
 *
 * @param handle pointer to RTOS handle
 */
status_t FLEXCAN_RTOS_Deinit(flexcan_rtos_handle_t *handle);

/*!
 * @brief Blocking send using FlexCAN non-blocking transfer underneath.
 *
 * This function:
 *  - takes the API mutex,
 *  - calls FLEXCAN_TransferSendNonBlocking(),
 *  - waits on txSemaphore,
 *  - releases the mutex and returns status.
 *
 * @param handle pointer to RTOS handle
 * @param pMbXfer pointer to flexcan_mb_transfer_t describing the transfer (frame + mbIdx)
 * @param timeout_ticks max FreeRTOS ticks to wait for completion (portMAX_DELAY allowed)
 * @return status_t (kStatus_Success on OK, kStatus_FLEXCAN_TxBusy if busy, or other error)
 */
status_t FLEXCAN_RTOS_Send(flexcan_rtos_handle_t *handle, flexcan_mb_transfer_t *pMbXfer, TickType_t timeout_ticks);

/*!
 * @brief Blocking receive using FlexCAN non-blocking transfer underneath.
 *
 * Similar to send but waits on rxSemaphore. This wrapper uses
 * FLEXCAN_TransferReceiveNonBlocking for message buffer based receive
 *
 * @param handle pointer to RTOS handle
 * @param pMbXfer pointer to flexcan_mb_transfer_t describing the receive (frame + mbIdx)
 * @param timeout_ticks ticks to wait
 * @return status_t
 */
status_t FLEXCAN_RTOS_Receive(flexcan_rtos_handle_t *handle, flexcan_mb_transfer_t *pMbXfer, TickType_t timeout_ticks);

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_FLEXCAN_FREERTOS_H_ */
