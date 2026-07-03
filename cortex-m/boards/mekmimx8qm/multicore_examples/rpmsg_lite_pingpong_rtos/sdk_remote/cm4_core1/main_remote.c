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
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_irqsteer.h"
#include "app_srtm.h"
#include "fsl_tpm.h"

#include <math.h>
#include "FWIS.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// RPMsg M4
#define RPMSG_LITE_LINK_ID              (RL_PLATFORM_IMX8QM_M4_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE           (M40_M41_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING   "rpmsg-openamp-demo-channel"
#define LOCAL_EPT_ADDR                  (41U)

// RPMsg Linux
#define RPMSG_LITE_LINUX_LINK_ID            (RL_PLATFORM_IMX8QM_M4_A_USER_LINK_ID)
#define RPMSG_LITE_LINUX_SHMEM_BASE         (VDEV1_VRING_BASE)
#define LOCAL_LINUX_EPT_ADDR                (31)
#define RPMSG_LITE_LINUX_NS_ANNOUNCE_STRING "rpmsg-raw"

// RTOS
#define RPMSG_TASK_PRIORITY         configMAX_PRIORITIES - 1
#define RPMSG_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE * 8

#define FWIS_TASK_PRIORITY          configMAX_PRIORITIES - 2
#define FWIS_TASK_STACK_SIZE        configMINIMAL_STACK_SIZE * 4

#define MONITOR_TASK_PRIORITY       configMAX_PRIORITIES - 7
#define MONITOR_TASK_STACK_SIZE     configMINIMAL_STACK_SIZE * 4

/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF

// Timer & Frequency Control
#define BOARD_TPM           CM4_1__TPM
#define BOARD_TPM_IRQ_NUM   M4_1_TPM_IRQn
#define BOARD_TPM_HANDLER   M4_1_TPM_IRQHandler
#define TPM_PRESCALER       kTPM_Prescale_Divide_128

// 4WIS
#define FWIS_WB     3.1f
#define FWIS_CG     1.55f
#define FWIS_WT     1.49f
#define FWIS_HWB    1.55f
#define FWIS_HWT    0.745f

#define FLOAT_TO_INT(x)     ((int)(x))
#define FLOAT_TO_FRAC(x)    ((int)(((x) < 0.0f ? -(x) : (x)) * 100.0f) % 100)

/*******************************************************************************
 * Variables
 ******************************************************************************/
// RTOS Queues
static QueueHandle_t xFWISJobQueue = NULL;
static QueueHandle_t xFWISResultQueue = NULL;

// RTOS Handles
static TaskHandle_t rpmsg_task_handle = NULL;
static TaskHandle_t fwis_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;

typedef struct __attribute__((packed))
{
    uint16_t steer_angle;
    uint16_t Vx;
} linux_recv_frame_t, *linux_recv_frame_t_ptr;

typedef struct __attribute__((packed))
{
    uint16_t Ang_ref[4];
} linux_send_frame_t, *linux_send_frame_t_ptr;

linux_recv_frame_t_ptr rx_linux;
linux_send_frame_t_ptr tx_linux;

fwis_t fwis;

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
    float steer_deg;
    float vx_steer;
    float ang_ref[4];
} fwis_monitor_t;
volatile fwis_monitor_t g_latest_fwis = {0};

/*******************************************************************************
 * Functions
 ******************************************************************************/
void Peripheral_Init(void)
{
    sc_ipc_t ipc;
    ipc = BOARD_InitRpc();

    BOARD_InitPins(ipc);
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    /* Power up the MU used for RPMSG */
    if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_6B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_1: Error: Failed to power on MU\r\n");
    }
    if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_7B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_1: Error: Failed to power on MU\r\n");
    }

    if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_1, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_1: Error: Failed to power on IRQSTEER!\r\n");
    }

    /* I2C module */
    if (sc_pm_set_resource_power_mode(ipc, SC_R_M4_1_I2C, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_1: Error: Failed to enable lpi2c");
    }
    if (CLOCK_SetIpFreq(kCLOCK_M4_1_Lpi2c, SC_133MHZ) == 0)
    {
        LOG_INFO("M4_1: Error: Failed to set LPI2C frequency\r\n");
    }

    if (sc_pm_set_resource_power_mode(ipc, SC_R_M4_1_TPM, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        LOG_INFO("M4_1: Error: Failed to power on TPM\r\n");
    }
    if (CLOCK_SetIpFreq(kCLOCK_M4_1_Tpm, SC_1MHZ) == 0)
    {
        LOG_INFO("M4_1: Error: Failed to set TPM frequency\r\n");
    }

    IRQSTEER_Init(IRQSTEER);

    APP_SRTM_Init();
    APP_SRTM_StartCommunication();
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

    LOG_INFO("M4_1: RPMSG Share Base Addr is 0x%x\r\n", RPMSG_LITE_LINUX_SHMEM_BASE);
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_LINUX_SHMEM_BASE, RPMSG_LITE_LINUX_LINK_ID, RL_NO_FLAGS);
    while (0 == rpmsg_lite_is_link_up(my_rpmsg)) { };
    LOG_INFO("M4_1: Link is up!\r\n");
    
    my_queue  = rpmsg_queue_create(my_rpmsg);
    my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_LINUX_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    
    SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_LINUX_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
    LOG_INFO("M4_1: Nameservice announce sent.\r\n");
    
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
            xQueueSend(xFWISJobQueue, &incoming_job, portMAX_DELAY);
            (void)rpmsg_queue_nocopy_free(my_rpmsg, rx_linux);

            // Wait for the TV task to finish calculating
            if (xQueueReceive(xFWISResultQueue, &outgoing_result, portMAX_DELAY) == pdPASS)
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
// 4 Wheel Independent Steering
// ---------------------------------------------------------
static void fwis_task(void *param)
{
    linux_recv_frame_t current_job;
    linux_send_frame_t current_result;

    FWIS_Init(&fwis, FWIS_WB, FWIS_CG, FWIS_WT, FWIS_HWB, FWIS_HWT);

    for(;;)
    {
        // Block until the RPMsg task passes a new set of data
        if (xQueueReceive(xFWISJobQueue, &current_job, portMAX_DELAY) == pdPASS)
        {
            float steer_deg = (float)(current_job.steer_angle / 100.0f) - 40.0f;
            float vx_steer  = (float)current_job.Vx / 100.0f;

            FWIS_Compute(&fwis, &steer_deg, &vx_steer);

            g_latest_fwis.steer_deg = steer_deg;
            g_latest_fwis.vx_steer = vx_steer;
            g_latest_fwis.ang_ref[0] = fwis.output[0];
            g_latest_fwis.ang_ref[1] = fwis.output[1];
            g_latest_fwis.ang_ref[2] = fwis.output[2];
            g_latest_fwis.ang_ref[3] = fwis.output[3];

            // Pack the results
            current_result.Ang_ref[0] = (uint16_t)((fwis.output[0] + 40.0f) * 100.0f);
            current_result.Ang_ref[1] = (uint16_t)((fwis.output[1] + 40.0f) * 100.0f);
            current_result.Ang_ref[2] = (uint16_t)((fwis.output[2] + 40.0f) * 100.0f);
            current_result.Ang_ref[3] = (uint16_t)((fwis.output[3] + 40.0f) * 100.0f);

            // Send back to the RPMsg task
            xQueueSend(xFWISResultQueue, &current_result, portMAX_DELAY);
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
    fwis_monitor_t temp_fwis = {0};
    
    TaskHandle_t idle_task_handle = xTaskGetIdleTaskHandle();
    
    temp_stats.num_tasks = 4;
    strncpy(temp_stats.tasks[0].name, "RPMSG", 7);   temp_stats.tasks[0].name[7] = '\0';
    strncpy(temp_stats.tasks[1].name, "4WIS", 7); temp_stats.tasks[1].name[7] = '\0';
    strncpy(temp_stats.tasks[2].name, "MONITOR", 7); temp_stats.tasks[2].name[7] = '\0';
    strncpy(temp_stats.tasks[3].name, "IDLE", 7);    temp_stats.tasks[3].name[7] = '\0';

    // Buffer to measure Delta CPU per 5 seconds (Real-time load)
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
        if(fwis_task_handle)      { vTaskGetInfo(fwis_task_handle, &tcb_stat, pdFALSE, eInvalid);      temp_stats.tasks[1].runtime = tcb_stat.ulRunTimeCounter; }
        if(monitor_task_handle) { vTaskGetInfo(monitor_task_handle, &tcb_stat, pdFALSE, eInvalid); temp_stats.tasks[2].runtime = tcb_stat.ulRunTimeCounter; }
        if(idle_task_handle)    { vTaskGetInfo(idle_task_handle, &tcb_stat, pdFALSE, eInvalid);    temp_stats.tasks[3].runtime = tcb_stat.ulRunTimeCounter; }

        taskENTER_CRITICAL();
        memcpy(&g_latest_stats, &temp_stats, sizeof(cpu_stats_t));
        temp_fwis = g_latest_fwis;
        taskEXIT_CRITICAL();

        LOG_INFO("\r\n=== M4_1 Core ===\r\n");
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

        // Calculation data
        LOG_INFO("----------------------------------------\r\n");
        LOG_INFO("[FWIS INPUT]\r\n");
        LOG_INFO("Steer Deg : %d.%02d\r\n", FLOAT_TO_INT(temp_fwis.steer_deg), FLOAT_TO_FRAC(temp_fwis.steer_deg));
        LOG_INFO("Vx Steer  : %d.%02d\r\n", FLOAT_TO_INT(temp_fwis.vx_steer), FLOAT_TO_FRAC(temp_fwis.vx_steer));
        
        LOG_INFO("\r\n[FWIS OUTPUT]\r\n");
        LOG_INFO("Ang_FL    : %d.%02d\r\n", FLOAT_TO_INT(temp_fwis.ang_ref[0]), FLOAT_TO_FRAC(temp_fwis.ang_ref[0]));
        LOG_INFO("Ang_FR    : %d.%02d\r\n", FLOAT_TO_INT(temp_fwis.ang_ref[1]), FLOAT_TO_FRAC(temp_fwis.ang_ref[1]));
        LOG_INFO("Ang_RL    : %d.%02d\r\n", FLOAT_TO_INT(temp_fwis.ang_ref[2]), FLOAT_TO_FRAC(temp_fwis.ang_ref[2]));
        LOG_INFO("Ang_RR    : %d.%02d\r\n", FLOAT_TO_INT(temp_fwis.ang_ref[3]), FLOAT_TO_FRAC(temp_fwis.ang_ref[3]));
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

    xFWISJobQueue = xQueueCreate(2, sizeof(linux_recv_frame_t));
    xFWISResultQueue = xQueueCreate(2, sizeof(linux_send_frame_t));

    if (xFWISJobQueue == NULL || xFWISResultQueue == NULL)
    {
        LOG_INFO("\r\nM4_1: Failed to create RTOS Queues\r\n");
        for(;;)
        {
        }
    }

    if (xTaskCreate(rpmsg_task, "RPMSG_TASK", RPMSG_TASK_STACK_SIZE, NULL, RPMSG_TASK_PRIORITY, &rpmsg_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_1: Failed to create RPMSG task\r\n");
        for (;;)
        {
        }
    }
    
    if (xTaskCreate(fwis_task, "FWIS_TASK", FWIS_TASK_STACK_SIZE, NULL, FWIS_TASK_PRIORITY, &fwis_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_1: Failed to create 4WIS task\r\n");
        for (;;)
        {
        }
    }

    if (xTaskCreate(monitor_task, "MONITOR", MONITOR_TASK_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, &monitor_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_1: Failed to create Monitor task\r\n");
        for (;;)
        {
        }
    }

    vTaskStartScheduler();

    LOG_INFO("M4_1: Failed to start FreeRTOS.\r\n");
    for (;;)
    {
    }
}
