/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "rsc_table.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_irqsteer.h"
#include "fsl_tpm.h"

#include <math.h>

#include "Torque_Vectoring.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// RPMsg M4
#define RPMSG_LITE_LINK_ID          (RL_PLATFORM_IMX8QM_M4_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE       (M40_M41_VRING_BASE)
#define LOCAL_EPT_ADDR              (40U)
#define SH_MEM_TOTAL_SIZE           (6144U)

// RPMsg Linux
#define RPMSG_LITE_LINUX_LINK_ID       (RL_PLATFORM_IMX8QM_M4_A_USER_LINK_ID)
#define RPMSG_LITE_LINUX_SHMEM_BASE    (VDEV1_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING   "rpmsg-raw"
#define LOCAL_LINUX_EPT_ADDR           (30U)

// RTOS Task Priorities & Stacks
#define RPMSG_TASK_PRIORITY         configMAX_PRIORITIES - 1
#define RPMSG_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE * 8

#define TV_TASK_PRIORITY            configMAX_PRIORITIES - 2
#define TV_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE * 4

#define MONITOR_TASK_PRIORITY       configMAX_PRIORITIES - 7
#define MONITOR_TASK_STACK_SIZE     configMINIMAL_STACK_SIZE * 4

/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF

// Timer
#define BOARD_TPM CM4_0__TPM
#define BOARD_TPM_IRQ_NUM M4_0_TPM_IRQn
#define BOARD_TPM_HANDLER M4_0_TPM_IRQHandler
#define TPM_PRESCALER kTPM_Prescale_Divide_128

#define FLOAT_TO_INT(x)     ((int)(x))
#define FLOAT_TO_FRAC(x)    ((int)(((x) < 0.0f ? -(x) : (x)) * 100.0f) % 100)

/*******************************************************************************
 * Variables
 ******************************************************************************/

// RTOS Handles
static TaskHandle_t rpmsg_task_handle = NULL;
static TaskHandle_t tv_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;

// RTOS Queues
static QueueHandle_t xTVJobQueue = NULL;
static QueueHandle_t xTVResultQueue = NULL;

char stats_buffer[512] = {0};

typedef struct
{
    RT_MODEL_Torque_Vectoring_T Torque_Vectoring_M_;
    RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_MPtr;
    DW_Torque_Vectoring_T Torque_Vectoring_DW;
    B_Torque_Vectoring_T Torque_Vectoring_B;

    real_T TV_U_Vx_des;
    real_T TV_U_Vx;
    real_T TV_U_Vy;
    real_T TV_U_AngWheel[4];
    real_T TV_U_r;
    real_T TV_U_Fy[4];
    real_T TV_U_Fz[4];

    real_T TV_Y_Tm[4];
    real_T TV_Y_Fx_opt[4];
    real_T TV_Y_Mx_total;
    real_T TV_Y_Fx_total;
    real_T TV_Y_Mzd;
    real_T TV_Y_r_des;
    real_T TV_Y_beta_des;
    real_T TV_Y_Ca[2];
    real_T TV_Y_beta;
} torque_vectoring_t;

typedef struct __attribute__((packed))
{
    uint16_t Vx_des;
    uint16_t Vx;
    uint16_t Vy;
    uint16_t angWheel[4];
    uint16_t yawRate;
    uint32_t Fy[4];
    uint32_t Fz[4];    
} linux_recv_frame_t, *linux_recv_frame_t_ptr;

typedef struct __attribute__((packed))
{
    uint16_t Tm_ref[4];
    uint32_t Mzd;
} linux_send_frame_t, *linux_send_frame_t_ptr;

linux_recv_frame_t_ptr rx_linux;
linux_send_frame_t_ptr tx_linux;

torque_vectoring_t tv = {
    .Torque_Vectoring_MPtr = &tv.Torque_Vectoring_M_
};

// Resource Profiler Data Structures
typedef struct
{
    char name[8];
    uint32_t runtime;
} task_stats_t;

typedef struct
{
    uint8_t num_tasks;
    uint32_t total_system_runtime;
    size_t free_heap_bytes;
    task_stats_t tasks[4];
} cpu_stats_t;

// Variabel for saving CPU latest cpu profile
cpu_stats_t g_latest_stats = {0};

typedef struct {
    float vx_des;
    float vx_actual;
    float tm_out[4];
} tv_monitor_t;
volatile tv_monitor_t g_latest_tv = {0};

/*******************************************************************************
 * Functions
 ******************************************************************************/
// Torque Vectoring
void Torque_Vectoring_Compute(const linux_recv_frame_t *data)
{
    tv.TV_U_Vx_des  = data->Vx_des;
    tv.TV_U_Vx      = data->Vx / 100.0f;
    tv.TV_U_Vy      = (data->Vy / 100.0f) - 10.0f;

    tv.TV_U_AngWheel[0] = (data->angWheel[0] / 1000.0f) - 0.7f;
    tv.TV_U_AngWheel[1] = (data->angWheel[1] / 1000.0f) - 0.7f;
    tv.TV_U_AngWheel[2] = (data->angWheel[2] / 1000.0f) - 0.7f;
    tv.TV_U_AngWheel[3] = (data->angWheel[3] / 1000.0f) - 0.7f;

    tv.TV_U_r = (data->yawRate / 1000.0f) - 0.7f;

    tv.TV_U_Fy[0] = (data->Fy[0] / 10.0f) - 9000.0f;
    tv.TV_U_Fy[1] = (data->Fy[1] / 10.0f) - 9000.0f;
    tv.TV_U_Fy[2] = (data->Fy[2] / 10.0f) - 9000.0f;
    tv.TV_U_Fy[3] = (data->Fy[3] / 10.0f) - 9000.0f;

    tv.TV_U_Fz[0] = data->Fz[0] / 10.0f;
    tv.TV_U_Fz[1] = data->Fz[1] / 10.0f;
    tv.TV_U_Fz[2] = data->Fz[2] / 10.0f;
    tv.TV_U_Fz[3] = data->Fz[3] / 10.0f;

    Torque_Vectoring_step(tv.Torque_Vectoring_MPtr,
        tv.TV_U_Vx_des, tv.TV_U_Vx, tv.TV_U_Vy,
        tv.TV_U_AngWheel, tv.TV_U_r,
        tv.TV_U_Fy, tv.TV_U_Fz,
        tv.TV_Y_Tm, tv.TV_Y_Fx_opt,
        &tv.TV_Y_Mx_total, &tv.TV_Y_Fx_total,
        &tv.TV_Y_Mzd, &tv.TV_Y_r_des,
        tv.TV_Y_Ca, &tv.TV_Y_beta);
}

static void Peripheral_Init(void)
{
    /* Initialize standard SDK demo application pins */
    sc_ipc_t ipc;
    ipc = BOARD_InitRpc();

    /* Pin initialization */
    BOARD_InitPins(ipc);
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    /* Power up the MU used for RPMSG */
    if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_5B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_0: Error: Failed to power on MU!\r\n");
    }
    if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_7A, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_0: Error: Failed to power on MU!\r\n");
    }

    if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_0: Error: Failed to power on IRQSTEER!\r\n");
    }

    if (sc_pm_set_resource_power_mode(ipc, SC_R_M4_0_TPM, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_0: Error: Failed to power on TPM\r\n");
    }
    if (CLOCK_SetIpFreq(kCLOCK_M4_0_Tpm, SC_1MHZ) == 0)
    {
        LOG_INFO("M4_0: Error: Failed to set TPM frequency\r\n");
    }

    IRQSTEER_Init(IRQSTEER);
    copyResourceTable();
}

// Timer for Resource Monitor
void App_ConfigureTimerForRuntimeStats(void)
{
    tpm_config_t tpmInfo;
    TPM_GetDefaultConfig(&tpmInfo);
    tpmInfo.prescale = TPM_PRESCALER;
    TPM_Init(BOARD_TPM, &tpmInfo);
    TPM_SetTimerPeriod(BOARD_TPM, 0xFFFFFFFFU);
    TPM_StartTimer(BOARD_TPM, kTPM_SystemClock);
}

uint32_t App_GetTimerForRuntimeStats(void)
{
    return TPM_GetCurrentTimerCount(BOARD_TPM);
}

/*******************************************************************************
 * ISR Callbacks
 ******************************************************************************/
static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
    uint32_t *data = (uint32_t *)user_data;
    *data = new_ept;
}

/*******************************************************************************
 * RTOS Tasks
 ******************************************************************************/
static void rpmsg_task(void *pvParameters)
{
    volatile uint32_t remote_addr = 0U;
    struct rpmsg_lite_endpoint *my_ept;
    rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *my_rpmsg;
    rpmsg_ns_handle ns_handle;

    LOG_INFO("M4_0: RPMSG Share Base Addr is 0x%x\r\n", RPMSG_LITE_LINUX_SHMEM_BASE);
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_LINUX_SHMEM_BASE, RPMSG_LITE_LINUX_LINK_ID, RL_NO_FLAGS);
    while (0 == rpmsg_lite_is_link_up(my_rpmsg)) { };
    LOG_INFO("M4_0: Link is up!\r\n");
    
    my_queue  = rpmsg_queue_create(my_rpmsg);
    my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_LINUX_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    
    SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
    LOG_INFO("M4_0: Nameservice announce sent.\r\n");
    
    uint32_t rx_len;
    uint32_t tx_len = sizeof(linux_send_frame_t);

    linux_recv_frame_t incoming_job;
    linux_send_frame_t outgoing_result;

    for (;;)
    {
        // Wait for packet from Linux
        (void)rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char **)&rx_linux, &rx_len, RL_BLOCK);
        
        if (rx_len == sizeof(linux_recv_frame_t))
        {
            // Copy data to local struct to pass to the TV Task
            memcpy(&incoming_job, rx_linux, sizeof(linux_recv_frame_t));
            
            // Send the job to the TV queue, then free the zero-copy buffer immediately
            xQueueSend(xTVJobQueue, &incoming_job, portMAX_DELAY);
            (void)rpmsg_queue_nocopy_free(my_rpmsg, rx_linux);

            // Wait for the TV task to finish calculating
            if (xQueueReceive(xTVResultQueue, &outgoing_result, portMAX_DELAY) == pdPASS)
            {
                // Allocate zero-copy transmit buffer
                tx_linux = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &tx_len, RL_BLOCK);
                
                // Copy calculation results to transmit buffer
                memcpy(tx_linux, &outgoing_result, sizeof(linux_send_frame_t));
                
                // Send back to Linux
                (void)rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_linux, tx_len);
            }
        }
        else 
        {
            // Drop invalid packets safely
            (void)rpmsg_queue_nocopy_free(my_rpmsg, rx_linux);
        }
    }
}

// ---------------------------------------------------------
// TORQUE VECTORING
// ---------------------------------------------------------
static void tv_task(void *param)
{
    linux_recv_frame_t current_job;
    linux_send_frame_t current_result;

    RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_M = tv.Torque_Vectoring_MPtr;
    Torque_Vectoring_M->blockIO = &tv.Torque_Vectoring_B;
    Torque_Vectoring_M->dwork = &tv.Torque_Vectoring_DW;

    Torque_Vectoring_initialize(Torque_Vectoring_M,
        &tv.TV_U_Vx_des, &tv.TV_U_Vx, &tv.TV_U_Vy,
        tv.TV_U_AngWheel, &tv.TV_U_r,
        tv.TV_U_Fy, tv.TV_U_Fz,
        tv.TV_Y_Tm, tv.TV_Y_Fx_opt,
        &tv.TV_Y_Mx_total, &tv.TV_Y_Fx_total,
        &tv.TV_Y_Mzd, &tv.TV_Y_r_des,
        &tv.TV_Y_beta_des, tv.TV_Y_Ca, &tv.TV_Y_beta);

    for(;;)
    {
        // Block until the RPMsg task passes a new set of data
        if (xQueueReceive(xTVJobQueue, &current_job, portMAX_DELAY) == pdPASS)
        {
            // Run the math model
            Torque_Vectoring_Compute(&current_job);

            // Copy for monitoring
            g_latest_tv.vx_des = tv.TV_U_Vx_des;
            g_latest_tv.vx_actual = tv.TV_U_Vx;
            g_latest_tv.tm_out[0] = tv.TV_Y_Tm[0];
            g_latest_tv.tm_out[1] = tv.TV_Y_Tm[1];
            g_latest_tv.tm_out[2] = tv.TV_Y_Tm[2];
            g_latest_tv.tm_out[3] = tv.TV_Y_Tm[3];

            // Pack the results
            current_result.Tm_ref[0] = (uint16_t)((tv.TV_Y_Tm[0] + 120.0f) * 100.0f);
            current_result.Tm_ref[1] = (uint16_t)((tv.TV_Y_Tm[1] + 120.0f) * 100.0f);
            current_result.Tm_ref[2] = (uint16_t)((tv.TV_Y_Tm[2] + 120.0f) * 100.0f);
            current_result.Tm_ref[3] = (uint16_t)((tv.TV_Y_Tm[3] + 120.0f) * 100.0f);

            current_result.Mzd = (uint32_t)(tv.TV_Y_Mzd + 40000);

            // Send back to the RPMsg task
            xQueueSend(xTVResultQueue, &current_result, portMAX_DELAY);
        }
    }
}

// ---------------------------------------------------------
// MONITOR (Resource & CPU Profiling Task)
// ---------------------------------------------------------
static void monitor_task(void *param)
{
    TaskStatus_t tcb_stat;
    cpu_stats_t temp_stats = {0};
    tv_monitor_t temp_tv = {0};
    
    TaskHandle_t idle_task_handle = xTaskGetIdleTaskHandle();
    
    temp_stats.num_tasks = 4;
    strncpy(temp_stats.tasks[0].name, "RPMSG", 7);   temp_stats.tasks[0].name[7] = '\0';
    strncpy(temp_stats.tasks[1].name, "TORQUEV", 7); temp_stats.tasks[1].name[7] = '\0';
    strncpy(temp_stats.tasks[2].name, "MONITOR", 7); temp_stats.tasks[2].name[7] = '\0';
    strncpy(temp_stats.tasks[3].name, "IDLE", 7);    temp_stats.tasks[3].name[7] = '\0';

    // Buffer to measure Delta CPU per 5 seconds
    uint32_t last_total_runtime = 0;
    uint32_t last_task_runtime[4] = {0};

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Take all system metrics
        temp_stats.total_system_runtime = App_GetTimerForRuntimeStats();
        temp_stats.free_heap_bytes = xPortGetFreeHeapSize();

        // PROFILING O(1) - Access TCB directly
        if(rpmsg_task_handle)   { vTaskGetInfo(rpmsg_task_handle, &tcb_stat, pdFALSE, eInvalid);   temp_stats.tasks[0].runtime = tcb_stat.ulRunTimeCounter; }
        if(tv_task_handle)      { vTaskGetInfo(tv_task_handle, &tcb_stat, pdFALSE, eInvalid);      temp_stats.tasks[1].runtime = tcb_stat.ulRunTimeCounter; }
        if(monitor_task_handle) { vTaskGetInfo(monitor_task_handle, &tcb_stat, pdFALSE, eInvalid); temp_stats.tasks[2].runtime = tcb_stat.ulRunTimeCounter; }
        if(idle_task_handle)    { vTaskGetInfo(idle_task_handle, &tcb_stat, pdFALSE, eInvalid);    temp_stats.tasks[3].runtime = tcb_stat.ulRunTimeCounter; }

        // Duplicate the latest stats to global variable
        taskENTER_CRITICAL();
        memcpy(&g_latest_stats, &temp_stats, sizeof(cpu_stats_t));
        temp_tv = g_latest_tv;
        taskEXIT_CRITICAL();

        LOG_INFO("\r\n=== M4_0 Core ===\r\n");
        LOG_INFO("Free Heap RAM : %u bytes\r\n", temp_stats.free_heap_bytes);
        LOG_INFO("Task\t\tTime(us)\tLoad %%\r\n");
        LOG_INFO("----------------------------------------\r\n");

        uint32_t total_delta = temp_stats.total_system_runtime - last_total_runtime;
        if (total_delta == 0) total_delta = 1;

        for (int i = 0; i < temp_stats.num_tasks; i++)
        {
            uint32_t task_delta = temp_stats.tasks[i].runtime - last_task_runtime[i];
            uint32_t percentage = (uint32_t)(((uint64_t)task_delta * 100ULL) / total_delta);

            if (percentage <= 0)
                LOG_INFO("%s\t\t%u\t\t<1%%\r\n", temp_stats.tasks[i].name, temp_stats.tasks[i].runtime);
            else
                LOG_INFO("%s\t\t%u\t\t%u%%\r\n", temp_stats.tasks[i].name, temp_stats.tasks[i].runtime, percentage);

            last_task_runtime[i] = temp_stats.tasks[i].runtime;
        }
        last_total_runtime = temp_stats.total_system_runtime;

        LOG_INFO("----------------------------------------\r\n");
        LOG_INFO("[TV INPUT]\r\n");
        LOG_INFO("Vx Target : %d.%02d\r\n", FLOAT_TO_INT(temp_tv.vx_des), FLOAT_TO_FRAC(temp_tv.vx_des));
        LOG_INFO("Vx Actual : %d.%02d\r\n", FLOAT_TO_INT(temp_tv.vx_actual), FLOAT_TO_FRAC(temp_tv.vx_actual));
        
        LOG_INFO("\r\n[TV OUTPUT (Tm_ref)]\r\n");
        LOG_INFO("Torque FL : %d.%02d Nm\r\n", FLOAT_TO_INT(temp_tv.tm_out[0]), FLOAT_TO_FRAC(temp_tv.tm_out[0]));
        LOG_INFO("Torque FR : %d.%02d Nm\r\n", FLOAT_TO_INT(temp_tv.tm_out[1]), FLOAT_TO_FRAC(temp_tv.tm_out[1]));
        LOG_INFO("Torque RL : %d.%02d Nm\r\n", FLOAT_TO_INT(temp_tv.tm_out[2]), FLOAT_TO_FRAC(temp_tv.tm_out[2]));
        LOG_INFO("Torque RR : %d.%02d Nm\r\n", FLOAT_TO_INT(temp_tv.tm_out[3]), FLOAT_TO_FRAC(temp_tv.tm_out[3]));
        LOG_INFO("========================================\r\n");
    }
}

/*******************************************************************************
 * Main
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    Peripheral_Init();

    /* Create queues before starting tasks */
    xTVJobQueue = xQueueCreate(2, sizeof(linux_recv_frame_t));
    xTVResultQueue = xQueueCreate(2, sizeof(linux_send_frame_t));

    if (xTVJobQueue == NULL || xTVResultQueue == NULL) {
        LOG_INFO("\r\nM4_0: Failed to create RTOS Queues\r\n");
        for(;;) {}
    }

    /* Print the initial banner */
    LOG_INFO("\r\nM4_0: RTOS app starts\r\n");

    if (xTaskCreate(rpmsg_task, "RPMSG", RPMSG_TASK_STACK_SIZE, NULL, RPMSG_TASK_PRIORITY, &rpmsg_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_0: Failed to create RPMsg task\r\n");
        for (;;) {}
    }

    if (xTaskCreate(tv_task, "TV_TASK", TV_TASK_STACK_SIZE, NULL, TV_TASK_PRIORITY, &tv_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_0: Failed to create Torque Vectoring task\r\n");
        for (;;) {}
    }

    if (xTaskCreate(monitor_task, "MONITOR", MONITOR_TASK_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, &monitor_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_0: Failed to create Monitor task\r\n");
        for (;;) {}
    }

    vTaskStartScheduler();

    LOG_INFO("M4_0: Failed to start FreeRTOS.\r\n");
    for (;;)
    {
    }
}
