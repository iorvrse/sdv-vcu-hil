#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

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

// RPMsg M4 configuration
#define RPMSG_LITE_LINK_ID              (RL_PLATFORM_IMX8QM_M4_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE           (M40_M41_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING   "rpmsg-openamp-demo-channel"
#define LOCAL_EPT_ADDR                  (41U)

// RPMsg Linux configuration
#define RPMSG_LITE_LINUX_LINK_ID            (RL_PLATFORM_IMX8QM_M4_A_USER_LINK_ID)
#define RPMSG_LITE_LINUX_SHMEM_BASE         (VDEV1_VRING_BASE)
#define LOCAL_LINUX_EPT_ADDR                (31)
#define RPMSG_LITE_LINUX_NS_ANNOUNCE_STRING "rpmsg-raw"

// RTOS task priorities and stack sizes
#define RPMSG_TASK_PRIORITY         configMAX_PRIORITIES - 1
#define RPMSG_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE * 8

#define MONITOR_TASK_PRIORITY       configMAX_PRIORITIES - 7
#define MONITOR_TASK_STACK_SIZE     configMINIMAL_STACK_SIZE * 4

#define LOG_INFO (void)PRINTF

// Timer and frequency control
#define BOARD_TPM           CM4_1__TPM
#define BOARD_TPM_IRQ_NUM   M4_1_TPM_IRQn
#define BOARD_TPM_HANDLER   M4_1_TPM_IRQHandler
#define TPM_PRESCALER       kTPM_Prescale_Divide_128

// Vehicle parameters for 4WIS
#define FWIS_WB     3.1f
#define FWIS_CG     1.55f
#define FWIS_WT     1.49f
#define FWIS_HWB    1.55f
#define FWIS_HWT    0.745f

// Float formatting helpers
#define FLOAT_SIGN_STR(x)   ((x) < 0.0f ? "-" : "")
#define FLOAT_TO_INT(x)     ((int)((x) < 0.0f ? -(x) : (x)))
#define FLOAT_TO_FRAC(x)    ((int)(((x) < 0.0f ? -(x) : (x)) * 100.0f) % 100)

// RTOS handles
static TaskHandle_t rpmsg_task_handle = NULL;
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

// Resource profiler structures
typedef struct
{
    char name[8];
    uint32_t runtime;
} task_stats_t;

typedef struct
{
    uint8_t num_tasks;
    uint32_t total_system_runtime;
    task_stats_t tasks[3];
} cpu_stats_t;

// Global structs for thread-safe monitoring
cpu_stats_t g_latest_stats = {0};

typedef struct {
    float ang_ref[4];
} fwis_monitor_t;
volatile fwis_monitor_t g_latest_fwis = {0};

// Hardware init
void Peripheral_Init(void)
{
    sc_ipc_t ipc;
    ipc = BOARD_InitRpc();

    BOARD_InitPins(ipc);
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

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

// Runtime stats timer configuration
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

// IPC and processing task
static void rpmsg_task(void *pvParameters)
{
    volatile uint32_t remote_addr = 0U;
    struct rpmsg_lite_endpoint *my_ept;
    rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *my_rpmsg;

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

    // Initialize math model once before the loop
    FWIS_Init(&fwis, FWIS_WB, FWIS_CG, FWIS_WT, FWIS_HWB, FWIS_HWT);

    for (;;)
    {
        // Block for zero-copy packet from Linux
        (void)rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char **)&rx_linux, &rx_len, RL_BLOCK);
        
        if (rx_len == sizeof(linux_recv_frame_t))
        {
            // Convert inputs
            float steer_deg = (float)(rx_linux->steer_angle / 100.0f) - 40.0f;
            float vx_steer  = (float)rx_linux->Vx / 100.0f;

            // Process math directly
            FWIS_Compute(&fwis, &steer_deg, &vx_steer);

            // Update monitor data
            g_latest_fwis.ang_ref[0] = fwis.output[0];
            g_latest_fwis.ang_ref[1] = fwis.output[1];
            g_latest_fwis.ang_ref[2] = fwis.output[2];
            g_latest_fwis.ang_ref[3] = fwis.output[3];

            // Allocate transmit buffer
            tx_linux = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &tx_len, RL_BLOCK);
            
            // Format and pack output data
            tx_linux->Ang_ref[0] = (uint16_t)((fwis.output[0] + 40.0f) * 100.0f);
            tx_linux->Ang_ref[1] = (uint16_t)((fwis.output[1] + 40.0f) * 100.0f);
            tx_linux->Ang_ref[2] = (uint16_t)((fwis.output[2] + 40.0f) * 100.0f);
            tx_linux->Ang_ref[3] = (uint16_t)((fwis.output[3] + 40.0f) * 100.0f);

            // Send payload back
            (void)rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_linux, tx_len);
        }
        
        // Free RX buffer
        if (rx_linux != NULL)
        {
            (void)rpmsg_queue_nocopy_free(my_rpmsg, rx_linux);
        }
    }
}

// System monitoring loop
static void monitor_task(void *param)
{
    TaskStatus_t tcb_stat;
    cpu_stats_t temp_stats = {0};
    fwis_monitor_t temp_fwis = {0};
    
    TaskHandle_t idle_task_handle = xTaskGetIdleTaskHandle();
    
    temp_stats.num_tasks = 3;
    strncpy(temp_stats.tasks[0].name, "RPMSG", 7);   temp_stats.tasks[0].name[7] = '\0';
    strncpy(temp_stats.tasks[1].name, "MONITOR", 7); temp_stats.tasks[1].name[7] = '\0';
    strncpy(temp_stats.tasks[2].name, "IDLE", 7);    temp_stats.tasks[2].name[7] = '\0';

    uint32_t last_total_runtime = 0;
    uint32_t last_task_runtime[3] = {0};

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));

        temp_stats.total_system_runtime = App_GetTimerForRuntimeStats();

        // Extract direct runtime counters
        if(rpmsg_task_handle)   { vTaskGetInfo(rpmsg_task_handle, &tcb_stat, pdFALSE, eInvalid);   temp_stats.tasks[0].runtime = tcb_stat.ulRunTimeCounter; }
        if(monitor_task_handle) { vTaskGetInfo(monitor_task_handle, &tcb_stat, pdFALSE, eInvalid); temp_stats.tasks[1].runtime = tcb_stat.ulRunTimeCounter; }
        if(idle_task_handle)    { vTaskGetInfo(idle_task_handle, &tcb_stat, pdFALSE, eInvalid);    temp_stats.tasks[2].runtime = tcb_stat.ulRunTimeCounter; }

        // Sync local struct copies
        taskENTER_CRITICAL();
        memcpy(&g_latest_stats, &temp_stats, sizeof(cpu_stats_t));
        temp_fwis = g_latest_fwis;
        taskEXIT_CRITICAL();

        LOG_INFO("\r\n=== M4_1 Core ===\r\n");
        LOG_INFO("Task\t\tLoad %%\r\n");
        LOG_INFO("--------------------------------\r\n");

        uint32_t total_delta = temp_stats.total_system_runtime - last_total_runtime;
        if (total_delta == 0) total_delta = 1;

        // Calculate and display load percentages
        for (int i = 0; i < temp_stats.num_tasks; i++)
        {
            uint32_t task_delta = temp_stats.tasks[i].runtime - last_task_runtime[i];
            uint32_t percentage = (uint32_t)(((uint64_t)task_delta * 100ULL) / total_delta);

            if (percentage <= 0)
                LOG_INFO("%s\t\t<1%%\r\n", temp_stats.tasks[i].name);
            else
                LOG_INFO("%s\t\t%u%%\r\n", temp_stats.tasks[i].name, percentage);

            last_task_runtime[i] = temp_stats.tasks[i].runtime;
        }
        last_total_runtime = temp_stats.total_system_runtime;

        // Simplified output data log
        LOG_INFO("--------------------------------\r\n");
        LOG_INFO("Angles: %s%d.%02d, %s%d.%02d, %s%d.%02d, %s%d.%02d\r\n", 
            FLOAT_SIGN_STR(temp_fwis.ang_ref[0]), FLOAT_TO_INT(temp_fwis.ang_ref[0]), FLOAT_TO_FRAC(temp_fwis.ang_ref[0]),
            FLOAT_SIGN_STR(temp_fwis.ang_ref[1]), FLOAT_TO_INT(temp_fwis.ang_ref[1]), FLOAT_TO_FRAC(temp_fwis.ang_ref[1]),
            FLOAT_SIGN_STR(temp_fwis.ang_ref[2]), FLOAT_TO_INT(temp_fwis.ang_ref[2]), FLOAT_TO_FRAC(temp_fwis.ang_ref[2]),
            FLOAT_SIGN_STR(temp_fwis.ang_ref[3]), FLOAT_TO_INT(temp_fwis.ang_ref[3]), FLOAT_TO_FRAC(temp_fwis.ang_ref[3]));
        LOG_INFO("================================\r\n");
    }
}

// Application entry
int main(void)
{
    Peripheral_Init();

    LOG_INFO("\r\nM4_1: RTOS app starts\r\n");

    if (xTaskCreate(rpmsg_task, "RPMSG_TASK", RPMSG_TASK_STACK_SIZE, NULL, RPMSG_TASK_PRIORITY, &rpmsg_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_1: Failed to create RPMSG task\r\n");
        for (;;) {}
    }

    if (xTaskCreate(monitor_task, "MONITOR", MONITOR_TASK_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, &monitor_task_handle) != pdPASS)
    {
        LOG_INFO("\r\nM4_1: Failed to create Monitor task\r\n");
        for (;;) {}
    }

    vTaskStartScheduler();

    LOG_INFO("M4_1: Failed to start FreeRTOS.\r\n");
    for (;;) {}
}