/*!
    \file    gd32g5x3_it.h
    \brief   中断服务函数头文件

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

#ifndef GD32G5X3_IT_H
#define GD32G5X3_IT_H

#include "gd32g5x3.h"

/* 中断服务函数声明。 */
/* NMI 异常处理函数。 */
void NMI_Handler(void);
/* HardFault 异常处理函数。 */
void HardFault_Handler(void);
/* SVC 异常处理函数。 */
void SVC_Handler(void);
/* PendSV 异常处理函数。 */
void PendSV_Handler(void);
/* SysTick 异常处理函数。 */
void SysTick_Handler(void);
/* EXTI10~15 共用中断，用于 AFE ALERT 和 FAULT_OC 快速关断。 */
void EXTI10_15_IRQHandler(void);
/* USART2 中断，用于上位机通信非阻塞发送。 */
void USART2_IRQHandler(void);

#endif /* GD32G5X3_IT_H */
