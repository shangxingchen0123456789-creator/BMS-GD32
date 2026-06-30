/*!
    \file    gd32g5x3_it.c
    \brief   中断服务函数

    \version 2026-02-04, V1.5.0, firmware for GD32G5x3
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32g5x3_it.h"
#include "systick.h"
#include "bms_board_config.h"
#include "bms_uart.h"
#include "safety_manager.h"

#define SRAM_ECC_ERROR_HANDLE(s)    do{}while(1)
#define FLASH_ECC_ERROR_HANDLE(s)   do{}while(1)

/*!
    \brief      NMI 异常处理函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    if(SET == syscfg_interrupt_flag_get(SYSCFG_INT_FLAG_TCMSRAMECCME)) {
        SRAM_ECC_ERROR_HANDLE("TCMSRAM multi-bits non-correction ECC error\r\n");
    }else if(SET == syscfg_interrupt_flag_get(SYSCFG_INT_FLAG_SRAM1ECCME)) {
        SRAM_ECC_ERROR_HANDLE("SRAM1 multi-bits non-correction ECC error\r\n");
    }else if(SET == syscfg_interrupt_flag_get(SYSCFG_INT_FLAG_SRAM0ECCME)) {
        SRAM_ECC_ERROR_HANDLE("SRAM0 multi-bits non-correction ECC error\r\n");
    }else if(SET == syscfg_interrupt_flag_get(SYSCFG_INT_FLAG_FLASHECC)){
        FLASH_ECC_ERROR_HANDLE("FLASH ECC error\r\n");
    }else{
        /* NMI 异常进入这里后停在死循环，便于调试。 */
        /* 可能原因：HXTAL 时钟监控 NMI 错误或 NMI 引脚错误。 */
        while(1) {
        }
    }
}

/*!
    \brief      HardFault 异常处理函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* HardFault 异常进入这里后停在死循环，便于调试。 */
    while (1){
    }
}

/*!
    \brief      SVC 异常处理函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void SVC_Handler(void)
//{
//    /* SVC 异常进入这里后停在死循环，便于调试。 */
//    while(1) {
//    }
//}

/*!
    \brief      PendSV 异常处理函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void PendSV_Handler(void)
//{
//    /* PendSV 异常进入这里后停在死循环，便于调试。 */
//    while(1) {
//    }
//}

/*!
    \brief      SysTick 异常处理函数
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void SysTick_Handler(void)
//{
//    Delay_Decrement();
//}

void EXTI10_15_IRQHandler(void)
{
    if(SET == exti_interrupt_flag_get(BMS_AFE_ALERT_EXTI_LINE)) {
        exti_interrupt_flag_clear(BMS_AFE_ALERT_EXTI_LINE);
        if(Safety_Manager_Afe_Alert_Monitor_Enabled() != 0U) {
            Safety_Manager_Handle_External_Fault_Isr(BMS_FAULT_AFE_PROTECTION);
        }
    }

#if BMS_ENABLE_POWER_FAULT_PIN
    if(SET == exti_interrupt_flag_get(BMS_PWM_FAULT_EXTI_LINE)) {
        exti_interrupt_flag_clear(BMS_PWM_FAULT_EXTI_LINE);
        Safety_Manager_Handle_Power_Fault_Isr();
    }
#endif
}

void USART2_IRQHandler(void)
{
    Bms_Uart_Irq_Handler();
}
