#include "gd32g5x3.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board_support.h"
#include "app_tasks.h"

/*
 * 固件入口。
 * 先完成板级初始化和任务创建，vTaskStartScheduler() 之后由 FreeRTOS
 * 调度充电控制循环和 UART 协议服务。
 */
int main(void)
{
    SystemCoreClockUpdate();
    NVIC_SetPriorityGrouping(0);

    Board_Support_Init();
    App_Tasks_Start();
    vTaskStartScheduler();

    while(1) {
    }
}
