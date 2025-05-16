#include "Task_Manager.h"
#include "Task_Telemetry.h"
// #include "Task_Control.h"
#include "Task_Navi.h"
#include "Task_Protocol.h"
#include "debug_util.h"
#include "HW_Def.h"
#include "Dev_Led.h"
#include "Srv_ComProto.h"
#include "Srv_OsCommon.h"
#include "../DataPipe/DataPipe.h"
#include "shell_port.h"
#include "Storage.h"
#include "Dev_W25Qxx.h"
#include "Dev_W25Qxx_QSPI.h"
#include "cmsis_os.h"

#define TaskControl_Period_Def   20 /* unit: ms period 5ms  50Hz */
#define TaskTelemetry_Period_def 2  /* unit: ms period 2ms  500Hz */
#define TaslLog_Period_Def       5  /* unit: ms period 5ms  200Hz */
#define TaslNavi_Period_Def      5  /* unit: ms period 10ms 200Hz */
#define TaskFrameCTL_Period_Def  5  /* unit: ms period 5ms  200Hz */

osThreadId TaskControl_Handle = NULL;
osThreadId TaskNavi_Handle = NULL;
osThreadId TaskLog_Handle = NULL;
osThreadId TaskTelemetry_Handle = NULL;
osThreadId TaskFrameCTL_Handle = NULL;
osThreadId TaskManager_Handle = NULL;

#define SYS_TAG "[ HARDWARE INFO ] "
#define SYS_INFO(fmt, ...) Debug_Print(&DebugPort, SYS_TAG, fmt, ##__VA_ARGS__)

static StorageDevObj_TypeDef StorageDevObj;

void Task_Manager_Init(void)
{
    DevLED.init(Led1);

    /* vol ADC init */

    /* cur ADC init */
    
    SrvOsCommon.init();

    memset(&StorageDevObj, 0, sizeof(StorageDevObj_TypeDef));
    osThreadDef(ManagerTask, Task_Manager_CreateTask, osPriorityLow, 0, 1024);
    TaskManager_Handle = osThreadCreate(osThread(ManagerTask), NULL);

    osKernelStart();
}

void Task_Manager_CreateTask(void const *arg)
{
    bool init = false;

    DebugPort.free = SrvOsCommon.free;
    DebugPort.malloc = SrvOsCommon.malloc;
    Debug_Port_Init(&DebugPort);

    SYS_INFO("%s\r\n", Select_Hardware);
    SYS_INFO("Hardware Version %d.%d.%d\r\n", HWVer[0], HWVer[1], HWVer[2]);
    SYS_INFO("App", "Start");

    StorageDevObj.chip_type = Flash_Chip_Type;
#if (FLASH_CHIP_STATE == Storage_ChipBus_Spi)
    StorageDevObj.api = (void *)&DevW25Qxx;
#elif (FLASH_CHIP_STATE == Storage_ChipBus_QSpi)
    StorageDevObj.api = (void *)&DevQSPIW25Qxx;
#endif

    while (true)
    {
        if (!init)
        {
            uint32_t sys_time = SrvOsCommon.get_os_ms();
            DEBUG_INFO("Sys Start\r\n");
            DEBUG_INFO("Sys Time: %d\r\n", sys_time);

            DataPipe_Init();
            Storage.init(&StorageDevObj);
            SrvUpgrade.init(NULL);
            SrvComProto.init(SrvComProto_Type_MAV, NULL);
            
            TaskTelemetry_Init(TaskTelemetry_Period_def);
            // TaskControl_Init(TaskControl_Period_Def);

            // TaskBlackBox_Init();
            TaskNavi_Init(TaslNavi_Period_Def);
            // TaskFrameCTL_Init(TaskFrameCTL_Period_Def);

            vTaskSuspendAll();
            // osThreadDef(TelemtryTask, TaskTelemetry_Core, osPriorityHigh, 0, 1024);
            // TaskTelemetry_Handle = osThreadCreate(osThread(TelemtryTask), NULL);

            // osThreadDef(ControlTask, TaskControl_Core, osPriorityHigh, 0, 1024);
            // TaskControl_Handle = osThreadCreate(osThread(ControlTask), NULL);

            osThreadDef(NavTask, TaskNavi_Core, osPriorityRealtime, 0, 4096);
            TaskNavi_Handle = osThreadCreate(osThread(NavTask), NULL);

            // osThreadDef(BlackBoxTask, TaskBlackBox_Core, osPriorityNormal, 0, 4096);
            // TaskLog_Handle = osThreadCreate(osThread(BlackBoxTask), NULL);

            // osThreadDef(FrameCTLTask, TaskFrameCTL_Core, osPriorityNormal, 0, 1024);
            // TaskFrameCTL_Handle = osThreadCreate(osThread(FrameCTLTask), NULL);
            xTaskResumeAll();

            init = true;
        }

        /* run system statistic in this task */
        osDelay(10);
    }
}
