Pular para o conteúdo principal
Google Sala de Aula
Sala de Aula
ELEW31 - Sistemas Microcontrolados - 2025/2
S71
Início
Agenda
Recursos
Minhas inscrições
Pendentes
E
ELEW31 - Sistemas Microcontrolados - 2025/2
S71
2
2024-1&2 ELEB030_ELF41 DIGITAL CIRCUITS
2
2023/1 ELE11 e EEX11 (manhã e tarde) 2023/1-IPLEE- Nelson (by Fabro)
Eletrônica e Eletricidade
Turmas arquivadas
Configurações
Mural
Atividades
Pessoas
Filtro de tópicosFiltro de tópicosFiltro de tópicos
Todos os temas
Material de Sala e Vídeos
Material de Sala e Vídeos
Material
Tópico 08 - Interrupções
Item postado: 14 de ago.
Material
Tópico 07 - GPIO
Item postado: 14 de ago.
Material
Tópico 06 - Fluxogramas
Item postado: 14 de ago.
Material
Tópico 05 - Criando um Projeto e Simulando
Item postado: 14 de ago.
Material
Tópico 04 - Instruções Assembly
Item postado: 14 de ago.
Material
Tópico 03 - Arquitetura
Item postado: 11 de ago.
Material
Tópico 02 - Instalação da IDE e dos Drivers
1
1 comentário
Última edição: 11 de ago.
Material
Tópico 01 - Revisão de Circuitos Digitais
Item postado: 11 de ago.
Material
Tópico 00 - Apresentação
Item postado: 11 de ago.
Atividades Práticas
Atividades Práticas
Atividade
Atividade Prática 1
Atividades Práticas
Data de entrega: 23 de set.
Concluída Atividade
Atividade Prática 0
Atividades Práticas
Data de entrega: 2 de set.
Arquivos Fontes
Arquivos Fontes
Material
Projeto Exemplo Tiva para Keil ver 5
Item postado: 19 de ago.
Material
Arquivos Base para um projeto em branco do Keil
Item postado: 19 de ago.
Material
Projeto Exemplo Tiva para Code Composer Studio
Item postado: 19 de ago.
Material
Arquivo tm4c1294ncpdt.h
Item postado: 9 de set.
Arquivo .h fornecido pelo Professor J. Valvano para consulta facilitada de todos os endereços dos periféricos do microcontrolador.

tm4c1294ncpdt.h
C
Documentação
Documentação
Material
PAT v1.1 - Placa Auxiliar Tiva
Item postado: 11 de ago.
Material
Datasheet do Microcontrolador TM4C1294NCPDT
Item postado: 11 de ago.
Material
Instruções assembly (Referência Rápida)
Item postado: 19 de ago.
Material
Instruções assembly (Conjunto completo)
Item postado: 19 de ago.
Material
Diferenças entre Keil e Code Composer
Item postado: 19 de ago.
Material
User Guide da placa EK-TM4C1294XL
Item postado: 9 de set.
Recursos
Recursos
Material
Add-on do Driver no Keil para Versões acima da 5.29
Item postado: 11 de ago.
Material
Drivers da placa EK-TM4C1294XL
Item postado: 11 de ago.
Material
LM Flash Programmer
Item postado: 11 de ago.
Material
Consertando erro ao debugar no Keil
Item postado: 11 de ago.
Atividade Arquivo tm4c1294ncpdt.h expandida
//*****************************************************************************
//
// tm4c1294ncpdt.h - TM4C1294NCPDT Register Definitions
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#ifndef __TM4C1294NCPDT_H__
#define __TM4C1294NCPDT_H__

//*****************************************************************************
//
// Interrupt assignments
//
//*****************************************************************************
#define INT_GPIOA               16          // GPIO Port A
#define INT_GPIOB               17          // GPIO Port B
#define INT_GPIOC               18          // GPIO Port C
#define INT_GPIOD               19          // GPIO Port D
#define INT_GPIOE               20          // GPIO Port E
#define INT_UART0               21          // UART0
#define INT_UART1               22          // UART1
#define INT_SSI0                23          // SSI0
#define INT_I2C0                24          // I2C0
#define INT_PWM0_FAULT          25          // PWM Fault
#define INT_PWM0_0              26          // PWM Generator 0
#define INT_PWM0_1              27          // PWM Generator 1
#define INT_PWM0_2              28          // PWM Generator 2
#define INT_QEI0                29          // QEI0
#define INT_ADC0SS0             30          // ADC0 Sequence 0
#define INT_ADC0SS1             31          // ADC0 Sequence 1
#define INT_ADC0SS2             32          // ADC0 Sequence 2
#define INT_ADC0SS3             33          // ADC0 Sequence 3
#define INT_WATCHDOG            34          // Watchdog Timers 0 and 1
#define INT_TIMER0A             35          // 16/32-Bit Timer 0A
#define INT_TIMER0B             36          // 16/32-Bit Timer 0B
#define INT_TIMER1A             37          // 16/32-Bit Timer 1A
#define INT_TIMER1B             38          // 16/32-Bit Timer 1B
#define INT_TIMER2A             39          // 16/32-Bit Timer 2A
#define INT_TIMER2B             40          // 16/32-Bit Timer 2B
#define INT_COMP0               41          // Analog Comparator 0
#define INT_COMP1               42          // Analog Comparator 1
#define INT_COMP2               43          // Analog Comparator 2
#define INT_SYSCTL              44          // System Control
#define INT_FLASH               45          // Flash Memory Control
#define INT_GPIOF               46          // GPIO Port F
#define INT_GPIOG               47          // GPIO Port G
#define INT_GPIOH               48          // GPIO Port H
#define INT_UART2               49          // UART2
#define INT_SSI1                50          // SSI1
#define INT_TIMER3A             51          // 16/32-Bit Timer 3A
#define INT_TIMER3B             52          // 16/32-Bit Timer 3B
#define INT_I2C1                53          // I2C1
#define INT_CAN0                54          // CAN 0
#define INT_CAN1                55          // CAN1
#define INT_EMAC0               56          // Ethernet MAC
#define INT_HIBERNATE           57          // HIB
#define INT_USB0                58          // USB MAC
#define INT_PWM0_3              59          // PWM Generator 3
#define INT_UDMA                60          // uDMA 0 Software
#define INT_UDMAERR             61          // uDMA 0 Error
#define INT_ADC1SS0             62          // ADC1 Sequence 0
#define INT_ADC1SS1             63          // ADC1 Sequence 1
#define INT_ADC1SS2             64          // ADC1 Sequence 2
#define INT_ADC1SS3             65          // ADC1 Sequence 3
#define INT_EPI0                66          // EPI 0
#define INT_GPIOJ               67          // GPIO Port J
#define INT_GPIOK               68          // GPIO Port K
#define INT_GPIOL               69          // GPIO Port L
#define INT_SSI2                70          // SSI 2
#define INT_SSI3                71          // SSI 3
#define INT_UART3               72          // UART 3
#define INT_UART4               73          // UART 4
#define INT_UART5               74          // UART 5
#define INT_UART6               75          // UART 6
#define INT_UART7               76          // UART 7
#define INT_I2C2                77          // I2C 2
#define INT_I2C3                78          // I2C 3
#define INT_TIMER4A             79          // Timer 4A
#define INT_TIMER4B             80          // Timer 4B
#define INT_TIMER5A             81          // Timer 5A
#define INT_TIMER5B             82          // Timer 5B
#define INT_SYSEXC              83          // Floating-Point Exception
                                            // (imprecise)
#define INT_I2C4                86          // I2C 4
#define INT_I2C5                87          // I2C 5
#define INT_GPIOM               88          // GPIO Port M
#define INT_GPION               89          // GPIO Port N
#define INT_TAMPER0             91          // Tamper
#define INT_GPIOP0              92          // GPIO Port P (Summary or P0)
#define INT_GPIOP1              93          // GPIO Port P1
#define INT_GPIOP2              94          // GPIO Port P2
#define INT_GPIOP3              95          // GPIO Port P3
#define INT_GPIOP4              96          // GPIO Port P4
#define INT_GPIOP5              97          // GPIO Port P5
#define INT_GPIOP6              98          // GPIO Port P6
#define INT_GPIOP7              99          // GPIO Port P7
#define INT_GPIOQ0              100         // GPIO Port Q (Summary or Q0)
#define INT_GPIOQ1              101         // GPIO Port Q1
#define INT_GPIOQ2              102         // GPIO Port Q2
#define INT_GPIOQ3              103         // GPIO Port Q3
#define INT_GPIOQ4              104         // GPIO Port Q4
#define INT_GPIOQ5              105         // GPIO Port Q5
#define INT_GPIOQ6              106         // GPIO Port Q6
#define INT_GPIOQ7              107         // GPIO Port Q7
#define INT_TIMER6A             114         // 16/32-Bit Timer 6A
#define INT_TIMER6B             115         // 16/32-Bit Timer 6B
#define INT_TIMER7A             116         // 16/32-Bit Timer 7A
#define INT_TIMER7B             117         // 16/32-Bit Timer 7B
#define INT_I2C6                118         // I2C 6
#define INT_I2C7                119         // I2C 7
#define INT_I2C8                125         // I2C 8
#define INT_I2C9                126         // I2C 9

//*****************************************************************************
//
// Watchdog Timer registers (WATCHDOG0)
//
//*****************************************************************************
#define WATCHDOG0_LOAD_R        (*((volatile uint32_t *)0x40000000))
#define WATCHDOG0_VALUE_R       (*((volatile uint32_t *)0x40000004))
#define WATCHDOG0_CTL_R         (*((volatile uint32_t *)0x40000008))
#define WATCHDOG0_ICR_R         (*((volatile uint32_t *)0x4000000C))
#define WATCHDOG0_RIS_R         (*((volatile uint32_t *)0x40000010))
#define WATCHDOG0_MIS_R         (*((volatile uint32_t *)0x40000014))
#define WATCHDOG0_TEST_R        (*((volatile uint32_t *)0x40000418))
#define WATCHDOG0_LOCK_R        (*((volatile uint32_t *)0x40000C00))

//*****************************************************************************
//
// Watchdog Timer registers (WATCHDOG1)
//
//*****************************************************************************
#define WATCHDOG1_LOAD_R        (*((volatile uint32_t *)0x40001000))
#define WATCHDOG1_VALUE_R       (*((volatile uint32_t *)0x40001004))
#define WATCHDOG1_CTL_R         (*((volatile uint32_t *)0x40001008))
#define WATCHDOG1_ICR_R         (*((volatile uint32_t *)0x4000100C))
#define WATCHDOG1_RIS_R         (*((volatile uint32_t *)0x40001010))
#define WATCHDOG1_MIS_R         (*((volatile uint32_t *)0x40001014))
#define WATCHDOG1_TEST_R        (*((volatile uint32_t *)0x40001418))
#define WATCHDOG1_LOCK_R        (*((volatile uint32_t *)0x40001C00))

//*****************************************************************************
//
// SSI registers (SSI0)
//
//*****************************************************************************
#define SSI0_CR0_R              (*((volatile uint32_t *)0x40008000))
#define SSI0_CR1_R              (*((volatile uint32_t *)0x40008004))
#define SSI0_DR_R               (*((volatile uint32_t *)0x40008008))
#define SSI0_SR_R               (*((volatile uint32_t *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile uint32_t *)0x40008010))
#define SSI0_IM_R               (*((volatile uint32_t *)0x40008014))
#define SSI0_RIS_R              (*((volatile uint32_t *)0x40008018))
#define SSI0_MIS_R              (*((volatile uint32_t *)0x4000801C))
#define SSI0_ICR_R              (*((volatile uint32_t *)0x40008020))
#define SSI0_DMACTL_R           (*((volatile uint32_t *)0x40008024))
#define SSI0_PP_R               (*((volatile uint32_t *)0x40008FC0))
#define SSI0_CC_R               (*((volatile uint32_t *)0x40008FC8))

//*****************************************************************************
//
// SSI registers (SSI1)
//
//*****************************************************************************
#define SSI1_CR0_R              (*((volatile uint32_t *)0x40009000))
#define SSI1_CR1_R              (*((volatile uint32_t *)0x40009004))
#define SSI1_DR_R               (*((volatile uint32_t *)0x40009008))
#define SSI1_SR_R               (*((volatile uint32_t *)0x4000900C))
#define SSI1_CPSR_R             (*((volatile uint32_t *)0x40009010))
#define SSI1_IM_R               (*((volatile uint32_t *)0x40009014))
#define SSI1_RIS_R              (*((volatile uint32_t *)0x40009018))
#define SSI1_MIS_R              (*((volatile uint32_t *)0x4000901C))
#define SSI1_ICR_R              (*((volatile uint32_t *)0x40009020))
#define SSI1_DMACTL_R           (*((volatile uint32_t *)0x40009024))
#define SSI1_PP_R               (*((volatile uint32_t *)0x40009FC0))
#define SSI1_CC_R               (*((volatile uint32_t *)0x40009FC8))

//*****************************************************************************
//
// SSI registers (SSI2)
//
//*****************************************************************************
#define SSI2_CR0_R              (*((volatile uint32_t *)0x4000A000))
#define SSI2_CR1_R              (*((volatile uint32_t *)0x4000A004))
#define SSI2_DR_R               (*((volatile uint32_t *)0x4000A008))
#define SSI2_SR_R               (*((volatile uint32_t *)0x4000A00C))
#define SSI2_CPSR_R             (*((volatile uint32_t *)0x4000A010))
#define SSI2_IM_R               (*((volatile uint32_t *)0x4000A014))
#define SSI2_RIS_R              (*((volatile uint32_t *)0x4000A018))
#define SSI2_MIS_R              (*((volatile uint32_t *)0x4000A01C))
#define SSI2_ICR_R              (*((volatile uint32_t *)0x4000A020))
#define SSI2_DMACTL_R           (*((volatile uint32_t *)0x4000A024))
#define SSI2_PP_R               (*((volatile uint32_t *)0x4000AFC0))
#define SSI2_CC_R               (*((volatile uint32_t *)0x4000AFC8))

//*****************************************************************************
//
// SSI registers (SSI3)
//
//*****************************************************************************
#define SSI3_CR0_R              (*((volatile uint32_t *)0x4000B000))
#define SSI3_CR1_R              (*((volatile uint32_t *)0x4000B004))
#define SSI3_DR_R               (*((volatile uint32_t *)0x4000B008))
#define SSI3_SR_R               (*((volatile uint32_t *)0x4000B00C))
#define SSI3_CPSR_R             (*((volatile uint32_t *)0x4000B010))
#define SSI3_IM_R               (*((volatile uint32_t *)0x4000B014))
#define SSI3_RIS_R              (*((volatile uint32_t *)0x4000B018))
#define SSI3_MIS_R              (*((volatile uint32_t *)0x4000B01C))
#define SSI3_ICR_R              (*((volatile uint32_t *)0x4000B020))
#define SSI3_DMACTL_R           (*((volatile uint32_t *)0x4000B024))
#define SSI3_PP_R               (*((volatile uint32_t *)0x4000BFC0))
#define SSI3_CC_R               (*((volatile uint32_t *)0x4000BFC8))

//*****************************************************************************
//
// UART registers (UART0)
//
//*****************************************************************************
#define UART0_DR_R              (*((volatile uint32_t *)0x4000C000))
#define UART0_RSR_R             (*((volatile uint32_t *)0x4000C004))
#define UART0_ECR_R             (*((volatile uint32_t *)0x4000C004))
#define UART0_FR_R              (*((volatile uint32_t *)0x4000C018))
#define UART0_ILPR_R            (*((volatile uint32_t *)0x4000C020))
#define UART0_IBRD_R            (*((volatile uint32_t *)0x4000C024))
#define UART0_FBRD_R            (*((volatile uint32_t *)0x4000C028))
#define UART0_LCRH_R            (*((volatile uint32_t *)0x4000C02C))
#define UART0_CTL_R             (*((volatile uint32_t *)0x4000C030))
#define UART0_IFLS_R            (*((volatile uint32_t *)0x4000C034))
#define UART0_IM_R              (*((volatile uint32_t *)0x4000C038))
#define UART0_RIS_R             (*((volatile uint32_t *)0x4000C03C))
#define UART0_MIS_R             (*((volatile uint32_t *)0x4000C040))
#define UART0_ICR_R             (*((volatile uint32_t *)0x4000C044))
#define UART0_DMACTL_R          (*((volatile uint32_t *)0x4000C048))
#define UART0_9BITADDR_R        (*((volatile uint32_t *)0x4000C0A4))
#define UART0_9BITAMASK_R       (*((volatile uint32_t *)0x4000C0A8))
#define UART0_PP_R              (*((volatile uint32_t *)0x4000CFC0))
#define UART0_CC_R              (*((volatile uint32_t *)0x4000CFC8))

//*****************************************************************************
//
// UART registers (UART1)
//
//*****************************************************************************
#define UART1_DR_R              (*((volatile uint32_t *)0x4000D000))
#define UART1_RSR_R             (*((volatile uint32_t *)0x4000D004))
#define UART1_ECR_R             (*((volatile uint32_t *)0x4000D004))
#define UART1_FR_R              (*((volatile uint32_t *)0x4000D018))
#define UART1_ILPR_R            (*((volatile uint32_t *)0x4000D020))
#define UART1_IBRD_R            (*((volatile uint32_t *)0x4000D024))
#define UART1_FBRD_R            (*((volatile uint32_t *)0x4000D028))
#define UART1_LCRH_R            (*((volatile uint32_t *)0x4000D02C))
#define UART1_CTL_R             (*((volatile uint32_t *)0x4000D030))
#define UART1_IFLS_R            (*((volatile uint32_t *)0x4000D034))
#define UART1_IM_R              (*((volatile uint32_t *)0x4000D038))
#define UART1_RIS_R             (*((volatile uint32_t *)0x4000D03C))
#define UART1_MIS_R             (*((volatile uint32_t *)0x4000D040))
#define UART1_ICR_R             (*((volatile uint32_t *)0x4000D044))
#define UART1_DMACTL_R          (*((volatile uint32_t *)0x4000D048))
#define UART1_9BITADDR_R        (*((volatile uint32_t *)0x4000D0A4))
#define UART1_9BITAMASK_R       (*((volatile uint32_t *)0x4000D0A8))
#define UART1_PP_R              (*((volatile uint32_t *)0x4000DFC0))
#define UART1_CC_R              (*((volatile uint32_t *)0x4000DFC8))

//*****************************************************************************
//
// UART registers (UART2)
//
//*****************************************************************************
#define UART2_DR_R              (*((volatile uint32_t *)0x4000E000))
#define UART2_RSR_R             (*((volatile uint32_t *)0x4000E004))
#define UART2_ECR_R             (*((volatile uint32_t *)0x4000E004))
#define UART2_FR_R              (*((volatile uint32_t *)0x4000E018))
#define UART2_ILPR_R            (*((volatile uint32_t *)0x4000E020))
#define UART2_IBRD_R            (*((volatile uint32_t *)0x4000E024))
#define UART2_FBRD_R            (*((volatile uint32_t *)0x4000E028))
#define UART2_LCRH_R            (*((volatile uint32_t *)0x4000E02C))
#define UART2_CTL_R             (*((volatile uint32_t *)0x4000E030))
#define UART2_IFLS_R            (*((volatile uint32_t *)0x4000E034))
#define UART2_IM_R              (*((volatile uint32_t *)0x4000E038))
#define UART2_RIS_R             (*((volatile uint32_t *)0x4000E03C))
#define UART2_MIS_R             (*((volatile uint32_t *)0x4000E040))
#define UART2_ICR_R             (*((volatile uint32_t *)0x4000E044))
#define UART2_DMACTL_R          (*((volatile uint32_t *)0x4000E048))
#define UART2_9BITADDR_R        (*((volatile uint32_t *)0x4000E0A4))
#define UART2_9BITAMASK_R       (*((volatile uint32_t *)0x4000E0A8))
#define UART2_PP_R              (*((volatile uint32_t *)0x4000EFC0))
#define UART2_CC_R              (*((volatile uint32_t *)0x4000EFC8))

//*****************************************************************************
//
// UART registers (UART3)
//
//*****************************************************************************
#define UART3_DR_R              (*((volatile uint32_t *)0x4000F000))
#define UART3_RSR_R             (*((volatile uint32_t *)0x4000F004))
#define UART3_ECR_R             (*((volatile uint32_t *)0x4000F004))
#define UART3_FR_R              (*((volatile uint32_t *)0x4000F018))
#define UART3_ILPR_R            (*((volatile uint32_t *)0x4000F020))
#define UART3_IBRD_R            (*((volatile uint32_t *)0x4000F024))
#define UART3_FBRD_R            (*((volatile uint32_t *)0x4000F028))
#define UART3_LCRH_R            (*((volatile uint32_t *)0x4000F02C))
#define UART3_CTL_R             (*((volatile uint32_t *)0x4000F030))
#define UART3_IFLS_R            (*((volatile uint32_t *)0x4000F034))
#define UART3_IM_R              (*((volatile uint32_t *)0x4000F038))
#define UART3_RIS_R             (*((volatile uint32_t *)0x4000F03C))
#define UART3_MIS_R             (*((volatile uint32_t *)0x4000F040))
#define UART3_ICR_R             (*((volatile uint32_t *)0x4000F044))
#define UART3_DMACTL_R          (*((volatile uint32_t *)0x4000F048))
#define UART3_9BITADDR_R        (*((volatile uint32_t *)0x4000F0A4))
#define UART3_9BITAMASK_R       (*((volatile uint32_t *)0x4000F0A8))
#define UART3_PP_R              (*((volatile uint32_t *)0x4000FFC0))
#define UART3_CC_R              (*((volatile uint32_t *)0x4000FFC8))

//*****************************************************************************
//
// UART registers (UART4)
//
//*****************************************************************************
#define UART4_DR_R              (*((volatile uint32_t *)0x40010000))
#define UART4_RSR_R             (*((volatile uint32_t *)0x40010004))
#define UART4_ECR_R             (*((volatile uint32_t *)0x40010004))
#define UART4_FR_R              (*((volatile uint32_t *)0x40010018))
#define UART4_ILPR_R            (*((volatile uint32_t *)0x40010020))
#define UART4_IBRD_R            (*((volatile uint32_t *)0x40010024))
#define UART4_FBRD_R            (*((volatile uint32_t *)0x40010028))
#define UART4_LCRH_R            (*((volatile uint32_t *)0x4001002C))
#define UART4_CTL_R             (*((volatile uint32_t *)0x40010030))
#define UART4_IFLS_R            (*((volatile uint32_t *)0x40010034))
#define UART4_IM_R              (*((volatile uint32_t *)0x40010038))
#define UART4_RIS_R             (*((volatile uint32_t *)0x4001003C))
#define UART4_MIS_R             (*((volatile uint32_t *)0x40010040))
#define UART4_ICR_R             (*((volatile uint32_t *)0x40010044))
#define UART4_DMACTL_R          (*((volatile uint32_t *)0x40010048))
#define UART4_9BITADDR_R        (*((volatile uint32_t *)0x400100A4))
#define UART4_9BITAMASK_R       (*((volatile uint32_t *)0x400100A8))
#define UART4_PP_R              (*((volatile uint32_t *)0x40010FC0))
#define UART4_CC_R              (*((volatile uint32_t *)0x40010FC8))

//*****************************************************************************
//
// UART registers (UART5)
//
//*****************************************************************************
#define UART5_DR_R              (*((volatile uint32_t *)0x40011000))
#define UART5_RSR_R             (*((volatile uint32_t *)0x40011004))
#define UART5_ECR_R             (*((volatile uint32_t *)0x40011004))
#define UART5_FR_R              (*((volatile uint32_t *)0x40011018))
#define UART5_ILPR_R            (*((volatile uint32_t *)0x40011020))
#define UART5_IBRD_R            (*((volatile uint32_t *)0x40011024))
#define UART5_FBRD_R            (*((volatile uint32_t *)0x40011028))
#define UART5_LCRH_R            (*((volatile uint32_t *)0x4001102C))
#define UART5_CTL_R             (*((volatile uint32_t *)0x40011030))
#define UART5_IFLS_R            (*((volatile uint32_t *)0x40011034))
#define UART5_IM_R              (*((volatile uint32_t *)0x40011038))
#define UART5_RIS_R             (*((volatile uint32_t *)0x4001103C))
#define UART5_MIS_R             (*((volatile uint32_t *)0x40011040))
#define UART5_ICR_R             (*((volatile uint32_t *)0x40011044))
#define UART5_DMACTL_R          (*((volatile uint32_t *)0x40011048))
#define UART5_9BITADDR_R        (*((volatile uint32_t *)0x400110A4))
#define UART5_9BITAMASK_R       (*((volatile uint32_t *)0x400110A8))
#define UART5_PP_R              (*((volatile uint32_t *)0x40011FC0))
#define UART5_CC_R              (*((volatile uint32_t *)0x40011FC8))

//*****************************************************************************
//
// UART registers (UART6)
//
//*****************************************************************************
#define UART6_DR_R              (*((volatile uint32_t *)0x40012000))
#define UART6_RSR_R             (*((volatile uint32_t *)0x40012004))
#define UART6_ECR_R             (*((volatile uint32_t *)0x40012004))
#define UART6_FR_R              (*((volatile uint32_t *)0x40012018))
#define UART6_ILPR_R            (*((volatile uint32_t *)0x40012020))
#define UART6_IBRD_R            (*((volatile uint32_t *)0x40012024))
#define UART6_FBRD_R            (*((volatile uint32_t *)0x40012028))
#define UART6_LCRH_R            (*((volatile uint32_t *)0x4001202C))
#define UART6_CTL_R             (*((volatile uint32_t *)0x40012030))
#define UART6_IFLS_R            (*((volatile uint32_t *)0x40012034))
#define UART6_IM_R              (*((volatile uint32_t *)0x40012038))
#define UART6_RIS_R             (*((volatile uint32_t *)0x4001203C))
#define UART6_MIS_R             (*((volatile uint32_t *)0x40012040))
#define UART6_ICR_R             (*((volatile uint32_t *)0x40012044))
#define UART6_DMACTL_R          (*((volatile uint32_t *)0x40012048))
#define UART6_9BITADDR_R        (*((volatile uint32_t *)0x400120A4))
#define UART6_9BITAMASK_R       (*((volatile uint32_t *)0x400120A8))
#define UART6_PP_R              (*((volatile uint32_t *)0x40012FC0))
#define UART6_CC_R              (*((volatile uint32_t *)0x40012FC8))

//*****************************************************************************
//
// UART registers (UART7)
//
//*****************************************************************************
#define UART7_DR_R              (*((volatile uint32_t *)0x40013000))
#define UART7_RSR_R             (*((volatile uint32_t *)0x40013004))
#define UART7_ECR_R             (*((volatile uint32_t *)0x40013004))
#define UART7_FR_R              (*((volatile uint32_t *)0x40013018))
#define UART7_ILPR_R            (*((volatile uint32_t *)0x40013020))
#define UART7_IBRD_R            (*((volatile uint32_t *)0x40013024))
#define UART7_FBRD_R            (*((volatile uint32_t *)0x40013028))
#define UART7_LCRH_R            (*((volatile uint32_t *)0x4001302C))
#define UART7_CTL_R             (*((volatile uint32_t *)0x40013030))
#define UART7_IFLS_R            (*((volatile uint32_t *)0x40013034))
#define UART7_IM_R              (*((volatile uint32_t *)0x40013038))
#define UART7_RIS_R             (*((volatile uint32_t *)0x4001303C))
#define UART7_MIS_R             (*((volatile uint32_t *)0x40013040))
#define UART7_ICR_R             (*((volatile uint32_t *)0x40013044))
#define UART7_DMACTL_R          (*((volatile uint32_t *)0x40013048))
#define UART7_9BITADDR_R        (*((volatile uint32_t *)0x400130A4))
#define UART7_9BITAMASK_R       (*((volatile uint32_t *)0x400130A8))
#define UART7_PP_R              (*((volatile uint32_t *)0x40013FC0))
#define UART7_CC_R              (*((volatile uint32_t *)0x40013FC8))

//*****************************************************************************
//
// I2C registers (I2C0)
//
//*****************************************************************************
#define I2C0_MSA_R              (*((volatile uint32_t *)0x40020000))
#define I2C0_MCS_R              (*((volatile uint32_t *)0x40020004))
#define I2C0_MDR_R              (*((volatile uint32_t *)0x40020008))
#define I2C0_MTPR_R             (*((volatile uint32_t *)0x4002000C))
#define I2C0_MIMR_R             (*((volatile uint32_t *)0x40020010))
#define I2C0_MRIS_R             (*((volatile uint32_t *)0x40020014))
#define I2C0_MMIS_R             (*((volatile uint32_t *)0x40020018))
#define I2C0_MICR_R             (*((volatile uint32_t *)0x4002001C))
#define I2C0_MCR_R              (*((volatile uint32_t *)0x40020020))
#define I2C0_MCLKOCNT_R         (*((volatile uint32_t *)0x40020024))
#define I2C0_MBMON_R            (*((volatile uint32_t *)0x4002002C))
#define I2C0_MBLEN_R            (*((volatile uint32_t *)0x40020030))
#define I2C0_MBCNT_R            (*((volatile uint32_t *)0x40020034))
#define I2C0_SOAR_R             (*((volatile uint32_t *)0x40020800))
#define I2C0_SCSR_R             (*((volatile uint32_t *)0x40020804))
#define I2C0_SDR_R              (*((volatile uint32_t *)0x40020808))
#define I2C0_SIMR_R             (*((volatile uint32_t *)0x4002080C))
#define I2C0_SRIS_R             (*((volatile uint32_t *)0x40020810))
#define I2C0_SMIS_R             (*((volatile uint32_t *)0x40020814))
#define I2C0_SICR_R             (*((volatile uint32_t *)0x40020818))
#define I2C0_SOAR2_R            (*((volatile uint32_t *)0x4002081C))
#define I2C0_SACKCTL_R          (*((volatile uint32_t *)0x40020820))
#define I2C0_FIFODATA_R         (*((volatile uint32_t *)0x40020F00))
#define I2C0_FIFOCTL_R          (*((volatile uint32_t *)0x40020F04))
#define I2C0_FIFOSTATUS_R       (*((volatile uint32_t *)0x40020F08))
#define I2C0_PP_R               (*((volatile uint32_t *)0x40020FC0))
#define I2C0_PC_R               (*((volatile uint32_t *)0x40020FC4))

//*****************************************************************************
//
// I2C registers (I2C1)
//
//*****************************************************************************
#define I2C1_MSA_R              (*((volatile uint32_t *)0x40021000))
#define I2C1_MCS_R              (*((volatile uint32_t *)0x40021004))
#define I2C1_MDR_R              (*((volatile uint32_t *)0x40021008))
#define I2C1_MTPR_R             (*((volatile uint32_t *)0x4002100C))
#define I2C1_MIMR_R             (*((volatile uint32_t *)0x40021010))
#define I2C1_MRIS_R             (*((volatile uint32_t *)0x40021014))
#define I2C1_MMIS_R             (*((volatile uint32_t *)0x40021018))
#define I2C1_MICR_R             (*((volatile uint32_t *)0x4002101C))
#define I2C1_MCR_R              (*((volatile uint32_t *)0x40021020))
#define I2C1_MCLKOCNT_R         (*((volatile uint32_t *)0x40021024))
#define I2C1_MBMON_R            (*((volatile uint32_t *)0x4002102C))
#define I2C1_MBLEN_R            (*((volatile uint32_t *)0x40021030))
#define I2C1_MBCNT_R            (*((volatile uint32_t *)0x40021034))
#define I2C1_SOAR_R             (*((volatile uint32_t *)0x40021800))
#define I2C1_SCSR_R             (*((volatile uint32_t *)0x40021804))
#define I2C1_SDR_R              (*((volatile uint32_t *)0x40021808))
#define I2C1_SIMR_R             (*((volatile uint32_t *)0x4002180C))
#define I2C1_SRIS_R             (*((volatile uint32_t *)0x40021810))
#define I2C1_SMIS_R             (*((volatile uint32_t *)0x40021814))
#define I2C1_SICR_R             (*((volatile uint32_t *)0x40021818))
#define I2C1_SOAR2_R            (*((volatile uint32_t *)0x4002181C))
#define I2C1_SACKCTL_R          (*((volatile uint32_t *)0x40021820))
#define I2C1_FIFODATA_R         (*((volatile uint32_t *)0x40021F00))
#define I2C1_FIFOCTL_R          (*((volatile uint32_t *)0x40021F04))
#define I2C1_FIFOSTATUS_R       (*((volatile uint32_t *)0x40021F08))
#define I2C1_PP_R               (*((volatile uint32_t *)0x40021FC0))
#define I2C1_PC_R               (*((volatile uint32_t *)0x40021FC4))

//*****************************************************************************
//
// I2C registers (I2C2)
//
//*****************************************************************************
#define I2C2_MSA_R              (*((volatile uint32_t *)0x40022000))
#define I2C2_MCS_R              (*((volatile uint32_t *)0x40022004))
#define I2C2_MDR_R              (*((volatile uint32_t *)0x40022008))
#define I2C2_MTPR_R             (*((volatile uint32_t *)0x4002200C))
#define I2C2_MIMR_R             (*((volatile uint32_t *)0x40022010))
#define I2C2_MRIS_R             (*((volatile uint32_t *)0x40022014))
#define I2C2_MMIS_R             (*((volatile uint32_t *)0x40022018))
#define I2C2_MICR_R             (*((volatile uint32_t *)0x4002201C))
#define I2C2_MCR_R              (*((volatile uint32_t *)0x40022020))
#define I2C2_MCLKOCNT_R         (*((volatile uint32_t *)0x40022024))
#define I2C2_MBMON_R            (*((volatile uint32_t *)0x4002202C))
#define I2C2_MBLEN_R            (*((volatile uint32_t *)0x40022030))
#define I2C2_MBCNT_R            (*((volatile uint32_t *)0x40022034))
#define I2C2_SOAR_R             (*((volatile uint32_t *)0x40022800))
#define I2C2_SCSR_R             (*((volatile uint32_t *)0x40022804))
#define I2C2_SDR_R              (*((volatile uint32_t *)0x40022808))
#define I2C2_SIMR_R             (*((volatile uint32_t *)0x4002280C))
#define I2C2_SRIS_R             (*((volatile uint32_t *)0x40022810))
#define I2C2_SMIS_R             (*((volatile uint32_t *)0x40022814))
#define I2C2_SICR_R             (*((volatile uint32_t *)0x40022818))
#define I2C2_SOAR2_R            (*((volatile uint32_t *)0x4002281C))
#define I2C2_SACKCTL_R          (*((volatile uint32_t *)0x40022820))
#define I2C2_FIFODATA_R         (*((volatile uint32_t *)0x40022F00))
#define I2C2_FIFOCTL_R          (*((volatile uint32_t *)0x40022F04))
#define I2C2_FIFOSTATUS_R       (*((volatile uint32_t *)0x40022F08))
#define I2C2_PP_R               (*((volatile uint32_t *)0x40022FC0))
#define I2C2_PC_R               (*((volatile uint32_t *)0x40022FC4))

//*****************************************************************************
//
// I2C registers (I2C3)
//
//*****************************************************************************
#define I2C3_MSA_R              (*((volatile uint32_t *)0x40023000))
#define I2C3_MCS_R              (*((volatile uint32_t *)0x40023004))
#define I2C3_MDR_R              (*((volatile uint32_t *)0x40023008))
#define I2C3_MTPR_R             (*((volatile uint32_t *)0x4002300C))
#define I2C3_MIMR_R             (*((volatile uint32_t *)0x40023010))
#define I2C3_MRIS_R             (*((volatile uint32_t *)0x40023014))
#define I2C3_MMIS_R             (*((volatile uint32_t *)0x40023018))
#define I2C3_MICR_R             (*((volatile uint32_t *)0x4002301C))
#define I2C3_MCR_R              (*((volatile uint32_t *)0x40023020))
#define I2C3_MCLKOCNT_R         (*((volatile uint32_t *)0x40023024))
#define I2C3_MBMON_R            (*((volatile uint32_t *)0x4002302C))
#define I2C3_MBLEN_R            (*((volatile uint32_t *)0x40023030))
#define I2C3_MBCNT_R            (*((volatile uint32_t *)0x40023034))
#define I2C3_SOAR_R             (*((volatile uint32_t *)0x40023800))
#define I2C3_SCSR_R             (*((volatile uint32_t *)0x40023804))
#define I2C3_SDR_R              (*((volatile uint32_t *)0x40023808))
#define I2C3_SIMR_R             (*((volatile uint32_t *)0x4002380C))
#define I2C3_SRIS_R             (*((volatile uint32_t *)0x40023810))
#define I2C3_SMIS_R             (*((volatile uint32_t *)0x40023814))
#define I2C3_SICR_R             (*((volatile uint32_t *)0x40023818))
#define I2C3_SOAR2_R            (*((volatile uint32_t *)0x4002381C))
#define I2C3_SACKCTL_R          (*((volatile uint32_t *)0x40023820))
#define I2C3_FIFODATA_R         (*((volatile uint32_t *)0x40023F00))
#define I2C3_FIFOCTL_R          (*((volatile uint32_t *)0x40023F04))
#define I2C3_FIFOSTATUS_R       (*((volatile uint32_t *)0x40023F08))
#define I2C3_PP_R               (*((volatile uint32_t *)0x40023FC0))
#define I2C3_PC_R               (*((volatile uint32_t *)0x40023FC4))

//*****************************************************************************
//
// PWM registers (PWM0)
//
//*****************************************************************************
#define PWM0_CTL_R              (*((volatile uint32_t *)0x40028000))
#define PWM0_SYNC_R             (*((volatile uint32_t *)0x40028004))
#define PWM0_ENABLE_R           (*((volatile uint32_t *)0x40028008))
#define PWM0_INVERT_R           (*((volatile uint32_t *)0x4002800C))
#define PWM0_FAULT_R            (*((volatile uint32_t *)0x40028010))
#define PWM0_INTEN_R            (*((volatile uint32_t *)0x40028014))
#define PWM0_RIS_R              (*((volatile uint32_t *)0x40028018))
#define PWM0_ISC_R              (*((volatile uint32_t *)0x4002801C))
#define PWM0_STATUS_R           (*((volatile uint32_t *)0x40028020))
#define PWM0_FAULTVAL_R         (*((volatile uint32_t *)0x40028024))
#define PWM0_ENUPD_R            (*((volatile uint32_t *)0x40028028))
#define PWM0_0_CTL_R            (*((volatile uint32_t *)0x40028040))
#define PWM0_0_INTEN_R          (*((volatile uint32_t *)0x40028044))
#define PWM0_0_RIS_R            (*((volatile uint32_t *)0x40028048))
#define PWM0_0_ISC_R            (*((volatile uint32_t *)0x4002804C))
#define PWM0_0_LOAD_R           (*((volatile uint32_t *)0x40028050))
#define PWM0_0_COUNT_R          (*((volatile uint32_t *)0x40028054))
#define PWM0_0_CMPA_R           (*((volatile uint32_t *)0x40028058))
#define PWM0_0_CMPB_R           (*((volatile uint32_t *)0x4002805C))
#define PWM0_0_GENA_R           (*((volatile uint32_t *)0x40028060))
#define PWM0_0_GENB_R           (*((volatile uint32_t *)0x40028064))
#define PWM0_0_DBCTL_R          (*((volatile uint32_t *)0x40028068))
#define PWM0_0_DBRISE_R         (*((volatile uint32_t *)0x4002806C))
#define PWM0_0_DBFALL_R         (*((volatile uint32_t *)0x40028070))
#define PWM0_0_FLTSRC0_R        (*((volatile uint32_t *)0x40028074))
#define PWM0_0_FLTSRC1_R        (*((volatile uint32_t *)0x40028078))
#define PWM0_0_MINFLTPER_R      (*((volatile uint32_t *)0x4002807C))
#define PWM0_1_CTL_R            (*((volatile uint32_t *)0x40028080))
#define PWM0_1_INTEN_R          (*((volatile uint32_t *)0x40028084))
#define PWM0_1_RIS_R            (*((volatile uint32_t *)0x40028088))
#define PWM0_1_ISC_R            (*((volatile uint32_t *)0x4002808C))
#define PWM0_1_LOAD_R           (*((volatile uint32_t *)0x40028090))
#define PWM0_1_COUNT_R          (*((volatile uint32_t *)0x40028094))
#define PWM0_1_CMPA_R           (*((volatile uint32_t *)0x40028098))
#define PWM0_1_CMPB_R           (*((volatile uint32_t *)0x4002809C))
#define PWM0_1_GENA_R           (*((volatile uint32_t *)0x400280A0))
#define PWM0_1_GENB_R           (*((volatile uint32_t *)0x400280A4))
#define PWM0_1_DBCTL_R          (*((volatile uint32_t *)0x400280A8))
#define PWM0_1_DBRISE_R         (*((volatile uint32_t *)0x400280AC))
#define PWM0_1_DBFALL_R         (*((volatile uint32_t *)0x400280B0))
#define PWM0_1_FLTSRC0_R        (*((volatile uint32_t *)0x400280B4))
#define PWM0_1_FLTSRC1_R        (*((volatile uint32_t *)0x400280B8))
#define PWM0_1_MINFLTPER_R      (*((volatile uint32_t *)0x400280BC))
#define PWM0_2_CTL_R            (*((volatile uint32_t *)0x400280C0))
#define PWM0_2_INTEN_R          (*((volatile uint32_t *)0x400280C4))
#define PWM0_2_RIS_R            (*((volatile uint32_t *)0x400280C8))
#define PWM0_2_ISC_R            (*((volatile uint32_t *)0x400280CC))
#define PWM0_2_LOAD_R           (*((volatile uint32_t *)0x400280D0))
#define PWM0_2_COUNT_R          (*((volatile uint32_t *)0x400280D4))
#define PWM0_2_CMPA_R           (*((volatile uint32_t *)0x400280D8))
#define PWM0_2_CMPB_R           (*((volatile uint32_t *)0x400280DC))
#define PWM0_2_GENA_R           (*((volatile uint32_t *)0x400280E0))
#define PWM0_2_GENB_R           (*((volatile uint32_t *)0x400280E4))
#define PWM0_2_DBCTL_R          (*((volatile uint32_t *)0x400280E8))
#define PWM0_2_DBRISE_R         (*((volatile uint32_t *)0x400280EC))
#define PWM0_2_DBFALL_R         (*((volatile uint32_t *)0x400280F0))
#define PWM0_2_FLTSRC0_R        (*((volatile uint32_t *)0x400280F4))
#define PWM0_2_FLTSRC1_R        (*((volatile uint32_t *)0x400280F8))
#define PWM0_2_MINFLTPER_R      (*((volatile uint32_t *)0x400280FC))
#define PWM0_3_CTL_R            (*((volatile uint32_t *)0x40028100))
#define PWM0_3_INTEN_R          (*((volatile uint32_t *)0x40028104))
#define PWM0_3_RIS_R            (*((volatile uint32_t *)0x40028108))
#define PWM0_3_ISC_R            (*((volatile uint32_t *)0x4002810C))
#define PWM0_3_LOAD_R           (*((volatile uint32_t *)0x40028110))
#define PWM0_3_COUNT_R          (*((volatile uint32_t *)0x40028114))
#define PWM0_3_CMPA_R           (*((volatile uint32_t *)0x40028118))
#define PWM0_3_CMPB_R           (*((volatile uint32_t *)0x4002811C))
#define PWM0_3_GENA_R           (*((volatile uint32_t *)0x40028120))
#define PWM0_3_GENB_R           (*((volatile uint32_t *)0x40028124))
#define PWM0_3_DBCTL_R          (*((volatile uint32_t *)0x40028128))
#define PWM0_3_DBRISE_R         (*((volatile uint32_t *)0x4002812C))
#define PWM0_3_DBFALL_R         (*((volatile uint32_t *)0x40028130))
#define PWM0_3_FLTSRC0_R        (*((volatile uint32_t *)0x40028134))
#define PWM0_3_FLTSRC1_R        (*((volatile uint32_t *)0x40028138))
#define PWM0_3_MINFLTPER_R      (*((volatile uint32_t *)0x4002813C))
#define PWM0_0_FLTSEN_R         (*((volatile uint32_t *)0x40028800))
#define PWM0_0_FLTSTAT0_R       (*((volatile uint32_t *)0x40028804))
#define PWM0_0_FLTSTAT1_R       (*((volatile uint32_t *)0x40028808))
#define PWM0_1_FLTSEN_R         (*((volatile uint32_t *)0x40028880))
#define PWM0_1_FLTSTAT0_R       (*((volatile uint32_t *)0x40028884))
#define PWM0_1_FLTSTAT1_R       (*((volatile uint32_t *)0x40028888))
#define PWM0_2_FLTSEN_R         (*((volatile uint32_t *)0x40028900))
#define PWM0_2_FLTSTAT0_R       (*((volatile uint32_t *)0x40028904))
#define PWM0_2_FLTSTAT1_R       (*((volatile uint32_t *)0x40028908))
#define PWM0_3_FLTSEN_R         (*((volatile uint32_t *)0x40028980))
#define PWM0_3_FLTSTAT0_R       (*((volatile uint32_t *)0x40028984))
#define PWM0_3_FLTSTAT1_R       (*((volatile uint32_t *)0x40028988))
#define PWM0_PP_R               (*((volatile uint32_t *)0x40028FC0))
#define PWM0_CC_R               (*((volatile uint32_t *)0x40028FC8))

//*****************************************************************************
//
// QEI registers (QEI0)
//
//*****************************************************************************
#define QEI0_CTL_R              (*((volatile uint32_t *)0x4002C000))
#define QEI0_STAT_R             (*((volatile uint32_t *)0x4002C004))
#define QEI0_POS_R              (*((volatile uint32_t *)0x4002C008))
#define QEI0_MAXPOS_R           (*((volatile uint32_t *)0x4002C00C))
#define QEI0_LOAD_R             (*((volatile uint32_t *)0x4002C010))
#define QEI0_TIME_R             (*((volatile uint32_t *)0x4002C014))
#define QEI0_COUNT_R            (*((volatile uint32_t *)0x4002C018))
#define QEI0_SPEED_R            (*((volatile uint32_t *)0x4002C01C))
#define QEI0_INTEN_R            (*((volatile uint32_t *)0x4002C020))
#define QEI0_RIS_R              (*((volatile uint32_t *)0x4002C024))
#define QEI0_ISC_R              (*((volatile uint32_t *)0x4002C028))

//*****************************************************************************
//
// Timer registers (TIMER0)
//
//*****************************************************************************
#define TIMER0_CFG_R            (*((volatile uint32_t *)0x40030000))
#define TIMER0_TAMR_R           (*((volatile uint32_t *)0x40030004))
#define TIMER0_TBMR_R           (*((volatile uint32_t *)0x40030008))
#define TIMER0_CTL_R            (*((volatile uint32_t *)0x4003000C))
#define TIMER0_SYNC_R           (*((volatile uint32_t *)0x40030010))
#define TIMER0_IMR_R            (*((volatile uint32_t *)0x40030018))
#define TIMER0_RIS_R            (*((volatile uint32_t *)0x4003001C))
#define TIMER0_MIS_R            (*((volatile uint32_t *)0x40030020))
#define TIMER0_ICR_R            (*((volatile uint32_t *)0x40030024))
#define TIMER0_TAILR_R          (*((volatile uint32_t *)0x40030028))
#define TIMER0_TBILR_R          (*((volatile uint32_t *)0x4003002C))
#define TIMER0_TAMATCHR_R       (*((volatile uint32_t *)0x40030030))
#define TIMER0_TBMATCHR_R       (*((volatile uint32_t *)0x40030034))
#define TIMER0_TAPR_R           (*((volatile uint32_t *)0x40030038))
#define TIMER0_TBPR_R           (*((volatile uint32_t *)0x4003003C))
#define TIMER0_TAPMR_R          (*((volatile uint32_t *)0x40030040))
#define TIMER0_TBPMR_R          (*((volatile uint32_t *)0x40030044))
#define TIMER0_TAR_R            (*((volatile uint32_t *)0x40030048))
#define TIMER0_TBR_R            (*((volatile uint32_t *)0x4003004C))
#define TIMER0_TAV_R            (*((volatile uint32_t *)0x40030050))
#define TIMER0_TBV_R            (*((volatile uint32_t *)0x40030054))
#define TIMER0_RTCPD_R          (*((volatile uint32_t *)0x40030058))
#define TIMER0_TAPS_R           (*((volatile uint32_t *)0x4003005C))
#define TIMER0_TBPS_R           (*((volatile uint32_t *)0x40030060))
#define TIMER0_DMAEV_R          (*((volatile uint32_t *)0x4003006C))
#define TIMER0_ADCEV_R          (*((volatile uint32_t *)0x40030070))
#define TIMER0_PP_R             (*((volatile uint32_t *)0x40030FC0))
#define TIMER0_CC_R             (*((volatile uint32_t *)0x40030FC8))

//*****************************************************************************
//
// Timer registers (TIMER1)
//
//*****************************************************************************
#define TIMER1_CFG_R            (*((volatile uint32_t *)0x40031000))
#define TIMER1_TAMR_R           (*((volatile uint32_t *)0x40031004))
#define TIMER1_TBMR_R           (*((volatile uint32_t *)0x40031008))
#define TIMER1_CTL_R            (*((volatile uint32_t *)0x4003100C))
#define TIMER1_SYNC_R           (*((volatile uint32_t *)0x40031010))
#define TIMER1_IMR_R            (*((volatile uint32_t *)0x40031018))
#define TIMER1_RIS_R            (*((volatile uint32_t *)0x4003101C))
#define TIMER1_MIS_R            (*((volatile uint32_t *)0x40031020))
#define TIMER1_ICR_R            (*((volatile uint32_t *)0x40031024))
#define TIMER1_TAILR_R          (*((volatile uint32_t *)0x40031028))
#define TIMER1_TBILR_R          (*((volatile uint32_t *)0x4003102C))
#define TIMER1_TAMATCHR_R       (*((volatile uint32_t *)0x40031030))
#define TIMER1_TBMATCHR_R       (*((volatile uint32_t *)0x40031034))
#define TIMER1_TAPR_R           (*((volatile uint32_t *)0x40031038))
#define TIMER1_TBPR_R           (*((volatile uint32_t *)0x4003103C))
#define TIMER1_TAPMR_R          (*((volatile uint32_t *)0x40031040))
#define TIMER1_TBPMR_R          (*((volatile uint32_t *)0x40031044))
#define TIMER1_TAR_R            (*((volatile uint32_t *)0x40031048))
#define TIMER1_TBR_R            (*((volatile uint32_t *)0x4003104C))
#define TIMER1_TAV_R            (*((volatile uint32_t *)0x40031050))
#define TIMER1_TBV_R            (*((volatile uint32_t *)0x40031054))
#define TIMER1_RTCPD_R          (*((volatile uint32_t *)0x40031058))
#define TIMER1_TAPS_R           (*((volatile uint32_t *)0x4003105C))
#define TIMER1_TBPS_R           (*((volatile uint32_t *)0x40031060))
#define TIMER1_DMAEV_R          (*((volatile uint32_t *)0x4003106C))
#define TIMER1_ADCEV_R          (*((volatile uint32_t *)0x40031070))
#define TIMER1_PP_R             (*((volatile uint32_t *)0x40031FC0))
#define TIMER1_CC_R             (*((volatile uint32_t *)0x40031FC8))

//*****************************************************************************
//
// Timer registers (TIMER2)
//
//*****************************************************************************
#define TIMER2_CFG_R            (*((volatile uint32_t *)0x40032000))
#define TIMER2_TAMR_R           (*((volatile uint32_t *)0x40032004))
#define TIMER2_TBMR_R           (*((volatile uint32_t *)0x40032008))
#define TIMER2_CTL_R            (*((volatile uint32_t *)0x4003200C))
#define TIMER2_SYNC_R           (*((volatile uint32_t *)0x40032010))
#define TIMER2_IMR_R            (*((volatile uint32_t *)0x40032018))
#define TIMER2_RIS_R            (*((volatile uint32_t *)0x4003201C))
#define TIMER2_MIS_R            (*((volatile uint32_t *)0x40032020))
#define TIMER2_ICR_R            (*((volatile uint32_t *)0x40032024))
#define TIMER2_TAILR_R          (*((volatile uint32_t *)0x40032028))
#define TIMER2_TBILR_R          (*((volatile uint32_t *)0x4003202C))
#define TIMER2_TAMATCHR_R       (*((volatile uint32_t *)0x40032030))
#define TIMER2_TBMATCHR_R       (*((volatile uint32_t *)0x40032034))
#define TIMER2_TAPR_R           (*((volatile uint32_t *)0x40032038))
#define TIMER2_TBPR_R           (*((volatile uint32_t *)0x4003203C))
#define TIMER2_TAPMR_R          (*((volatile uint32_t *)0x40032040))
#define TIMER2_TBPMR_R          (*((volatile uint32_t *)0x40032044))
#define TIMER2_TAR_R            (*((volatile uint32_t *)0x40032048))
#define TIMER2_TBR_R            (*((volatile uint32_t *)0x4003204C))
#define TIMER2_TAV_R            (*((volatile uint32_t *)0x40032050))
#define TIMER2_TBV_R            (*((volatile uint32_t *)0x40032054))
#define TIMER2_RTCPD_R          (*((volatile uint32_t *)0x40032058))
#define TIMER2_TAPS_R           (*((volatile uint32_t *)0x4003205C))
#define TIMER2_TBPS_R           (*((volatile uint32_t *)0x40032060))
#define TIMER2_DMAEV_R          (*((volatile uint32_t *)0x4003206C))
#define TIMER2_ADCEV_R          (*((volatile uint32_t *)0x40032070))
#define TIMER2_PP_R             (*((volatile uint32_t *)0x40032FC0))
#define TIMER2_CC_R             (*((volatile uint32_t *)0x40032FC8))

//*****************************************************************************
//
// Timer registers (TIMER3)
//
//*****************************************************************************
#define TIMER3_CFG_R            (*((volatile uint32_t *)0x40033000))
#define TIMER3_TAMR_R           (*((volatile uint32_t *)0x40033004))
#define TIMER3_TBMR_R           (*((volatile uint32_t *)0x40033008))
#define TIMER3_CTL_R            (*((volatile uint32_t *)0x4003300C))
#define TIMER3_SYNC_R           (*((volatile uint32_t *)0x40033010))
#define TIMER3_IMR_R            (*((volatile uint32_t *)0x40033018))
#define TIMER3_RIS_R            (*((volatile uint32_t *)0x4003301C))
#define TIMER3_MIS_R            (*((volatile uint32_t *)0x40033020))
#define TIMER3_ICR_R            (*((volatile uint32_t *)0x40033024))
#define TIMER3_TAILR_R          (*((volatile uint32_t *)0x40033028))
#define TIMER3_TBILR_R          (*((volatile uint32_t *)0x4003302C))
#define TIMER3_TAMATCHR_R       (*((volatile uint32_t *)0x40033030))
#define TIMER3_TBMATCHR_R       (*((volatile uint32_t *)0x40033034))
#define TIMER3_TAPR_R           (*((volatile uint32_t *)0x40033038))
#define TIMER3_TBPR_R           (*((volatile uint32_t *)0x4003303C))
#define TIMER3_TAPMR_R          (*((volatile uint32_t *)0x40033040))
#define TIMER3_TBPMR_R          (*((volatile uint32_t *)0x40033044))
#define TIMER3_TAR_R            (*((volatile uint32_t *)0x40033048))
#define TIMER3_TBR_R            (*((volatile uint32_t *)0x4003304C))
#define TIMER3_TAV_R            (*((volatile uint32_t *)0x40033050))
#define TIMER3_TBV_R            (*((volatile uint32_t *)0x40033054))
#define TIMER3_RTCPD_R          (*((volatile uint32_t *)0x40033058))
#define TIMER3_TAPS_R           (*((volatile uint32_t *)0x4003305C))
#define TIMER3_TBPS_R           (*((volatile uint32_t *)0x40033060))
#define TIMER3_DMAEV_R          (*((volatile uint32_t *)0x4003306C))
#define TIMER3_ADCEV_R          (*((volatile uint32_t *)0x40033070))
#define TIMER3_PP_R             (*((volatile uint32_t *)0x40033FC0))
#define TIMER3_CC_R             (*((volatile uint32_t *)0x40033FC8))

//*****************************************************************************
//
// Timer registers (TIMER4)
//
//*****************************************************************************
#define TIMER4_CFG_R            (*((volatile uint32_t *)0x40034000))
#define TIMER4_TAMR_R           (*((volatile uint32_t *)0x40034004))
#define TIMER4_TBMR_R           (*((volatile uint32_t *)0x40034008))
#define TIMER4_CTL_R            (*((volatile uint32_t *)0x4003400C))
#define TIMER4_SYNC_R           (*((volatile uint32_t *)0x40034010))
#define TIMER4_IMR_R            (*((volatile uint32_t *)0x40034018))
#define TIMER4_RIS_R            (*((volatile uint32_t *)0x4003401C))
#define TIMER4_MIS_R            (*((volatile uint32_t *)0x40034020))
#define TIMER4_ICR_R            (*((volatile uint32_t *)0x40034024))
#define TIMER4_TAILR_R          (*((volatile uint32_t *)0x40034028))
#define TIMER4_TBILR_R          (*((volatile uint32_t *)0x4003402C))
#define TIMER4_TAMATCHR_R       (*((volatile uint32_t *)0x40034030))
#define TIMER4_TBMATCHR_R       (*((volatile uint32_t *)0x40034034))
#define TIMER4_TAPR_R           (*((volatile uint32_t *)0x40034038))
#define TIMER4_TBPR_R           (*((volatile uint32_t *)0x4003403C))
#define TIMER4_TAPMR_R          (*((volatile uint32_t *)0x40034040))
#define TIMER4_TBPMR_R          (*((volatile uint32_t *)0x40034044))
#define TIMER4_TAR_R            (*((volatile uint32_t *)0x40034048))
#define TIMER4_TBR_R            (*((volatile uint32_t *)0x4003404C))
#define TIMER4_TAV_R            (*((volatile uint32_t *)0x40034050))
#define TIMER4_TBV_R            (*((volatile uint32_t *)0x40034054))
#define TIMER4_RTCPD_R          (*((volatile uint32_t *)0x40034058))
#define TIMER4_TAPS_R           (*((volatile uint32_t *)0x4003405C))
#define TIMER4_TBPS_R           (*((volatile uint32_t *)0x40034060))
#define TIMER4_DMAEV_R          (*((volatile uint32_t *)0x4003406C))
#define TIMER4_ADCEV_R          (*((volatile uint32_t *)0x40034070))
#define TIMER4_PP_R             (*((volatile uint32_t *)0x40034FC0))
#define TIMER4_CC_R             (*((volatile uint32_t *)0x40034FC8))

//*****************************************************************************
//
// Timer registers (TIMER5)
//
//*****************************************************************************
#define TIMER5_CFG_R            (*((volatile uint32_t *)0x40035000))
#define TIMER5_TAMR_R           (*((volatile uint32_t *)0x40035004))
#define TIMER5_TBMR_R           (*((volatile uint32_t *)0x40035008))
#define TIMER5_CTL_R            (*((volatile uint32_t *)0x4003500C))
#define TIMER5_SYNC_R           (*((volatile uint32_t *)0x40035010))
#define TIMER5_IMR_R            (*((volatile uint32_t *)0x40035018))
#define TIMER5_RIS_R            (*((volatile uint32_t *)0x4003501C))
#define TIMER5_MIS_R            (*((volatile uint32_t *)0x40035020))
#define TIMER5_ICR_R            (*((volatile uint32_t *)0x40035024))
#define TIMER5_TAILR_R          (*((volatile uint32_t *)0x40035028))
#define TIMER5_TBILR_R          (*((volatile uint32_t *)0x4003502C))
#define TIMER5_TAMATCHR_R       (*((volatile uint32_t *)0x40035030))
#define TIMER5_TBMATCHR_R       (*((volatile uint32_t *)0x40035034))
#define TIMER5_TAPR_R           (*((volatile uint32_t *)0x40035038))
#define TIMER5_TBPR_R           (*((volatile uint32_t *)0x4003503C))
#define TIMER5_TAPMR_R          (*((volatile uint32_t *)0x40035040))
#define TIMER5_TBPMR_R          (*((volatile uint32_t *)0x40035044))
#define TIMER5_TAR_R            (*((volatile uint32_t *)0x40035048))
#define TIMER5_TBR_R            (*((volatile uint32_t *)0x4003504C))
#define TIMER5_TAV_R            (*((volatile uint32_t *)0x40035050))
#define TIMER5_TBV_R            (*((volatile uint32_t *)0x40035054))
#define TIMER5_RTCPD_R          (*((volatile uint32_t *)0x40035058))
#define TIMER5_TAPS_R           (*((volatile uint32_t *)0x4003505C))
#define TIMER5_TBPS_R           (*((volatile uint32_t *)0x40035060))
#define TIMER5_DMAEV_R          (*((volatile uint32_t *)0x4003506C))
#define TIMER5_ADCEV_R          (*((volatile uint32_t *)0x40035070))
#define TIMER5_PP_R             (*((volatile uint32_t *)0x40035FC0))
#define TIMER5_CC_R             (*((volatile uint32_t *)0x40035FC8))

//*****************************************************************************
//
// ADC registers (ADC0)
//
//*****************************************************************************
#define ADC0_ACTSS_R            (*((volatile uint32_t *)0x40038000))
#define ADC0_RIS_R              (*((volatile uint32_t *)0x40038004))
#define ADC0_IM_R               (*((volatile uint32_t *)0x40038008))
#define ADC0_ISC_R              (*((volatile uint32_t *)0x4003800C))
#define ADC0_OSTAT_R            (*((volatile uint32_t *)0x40038010))
#define ADC0_EMUX_R             (*((volatile uint32_t *)0x40038014))
#define ADC0_USTAT_R            (*((volatile uint32_t *)0x40038018))
#define ADC0_TSSEL_R            (*((volatile uint32_t *)0x4003801C))
#define ADC0_SSPRI_R            (*((volatile uint32_t *)0x40038020))
#define ADC0_SPC_R              (*((volatile uint32_t *)0x40038024))
#define ADC0_PSSI_R             (*((volatile uint32_t *)0x40038028))
#define ADC0_SAC_R              (*((volatile uint32_t *)0x40038030))
#define ADC0_DCISC_R            (*((volatile uint32_t *)0x40038034))
#define ADC0_CTL_R              (*((volatile uint32_t *)0x40038038))
#define ADC0_SSMUX0_R           (*((volatile uint32_t *)0x40038040))
#define ADC0_SSCTL0_R           (*((volatile uint32_t *)0x40038044))
#define ADC0_SSFIFO0_R          (*((volatile uint32_t *)0x40038048))
#define ADC0_SSFSTAT0_R         (*((volatile uint32_t *)0x4003804C))
#define ADC0_SSOP0_R            (*((volatile uint32_t *)0x40038050))
#define ADC0_SSDC0_R            (*((volatile uint32_t *)0x40038054))
#define ADC0_SSEMUX0_R          (*((volatile uint32_t *)0x40038058))
#define ADC0_SSTSH0_R           (*((volatile uint32_t *)0x4003805C))
#define ADC0_SSMUX1_R           (*((volatile uint32_t *)0x40038060))
#define ADC0_SSCTL1_R           (*((volatile uint32_t *)0x40038064))
#define ADC0_SSFIFO1_R          (*((volatile uint32_t *)0x40038068))
#define ADC0_SSFSTAT1_R         (*((volatile uint32_t *)0x4003806C))
#define ADC0_SSOP1_R            (*((volatile uint32_t *)0x40038070))
#define ADC0_SSDC1_R            (*((volatile uint32_t *)0x40038074))
#define ADC0_SSEMUX1_R          (*((volatile uint32_t *)0x40038078))
#define ADC0_SSTSH1_R           (*((volatile uint32_t *)0x4003807C))
#define ADC0_SSMUX2_R           (*((volatile uint32_t *)0x40038080))
#define ADC0_SSCTL2_R           (*((volatile uint32_t *)0x40038084))
#define ADC0_SSFIFO2_R          (*((volatile uint32_t *)0x40038088))
#define ADC0_SSFSTAT2_R         (*((volatile uint32_t *)0x4003808C))
#define ADC0_SSOP2_R            (*((volatile uint32_t *)0x40038090))
#define ADC0_SSDC2_R            (*((volatile uint32_t *)0x40038094))
#define ADC0_SSEMUX2_R          (*((volatile uint32_t *)0x40038098))
#define ADC0_SSTSH2_R           (*((volatile uint32_t *)0x4003809C))
#define ADC0_SSMUX3_R           (*((volatile uint32_t *)0x400380A0))
#define ADC0_SSCTL3_R           (*((volatile uint32_t *)0x400380A4))
#define ADC0_SSFIFO3_R          (*((volatile uint32_t *)0x400380A8))
#define ADC0_SSFSTAT3_R         (*((volatile uint32_t *)0x400380AC))
#define ADC0_SSOP3_R            (*((volatile uint32_t *)0x400380B0))
#define ADC0_SSDC3_R            (*((volatile uint32_t *)0x400380B4))
#define ADC0_SSEMUX3_R          (*((volatile uint32_t *)0x400380B8))
#define ADC0_SSTSH3_R           (*((volatile uint32_t *)0x400380BC))
#define ADC0_DCRIC_R            (*((volatile uint32_t *)0x40038D00))
#define ADC0_DCCTL0_R           (*((volatile uint32_t *)0x40038E00))
#define ADC0_DCCTL1_R           (*((volatile uint32_t *)0x40038E04))
#define ADC0_DCCTL2_R           (*((volatile uint32_t *)0x40038E08))
#define ADC0_DCCTL3_R           (*((volatile uint32_t *)0x40038E0C))
#define ADC0_DCCTL4_R           (*((volatile uint32_t *)0x40038E10))
#define ADC0_DCCTL5_R           (*((volatile uint32_t *)0x40038E14))
#define ADC0_DCCTL6_R           (*((volatile uint32_t *)0x40038E18))
#define ADC0_DCCTL7_R           (*((volatile uint32_t *)0x40038E1C))
#define ADC0_DCCMP0_R           (*((volatile uint32_t *)0x40038E40))
#define ADC0_DCCMP1_R           (*((volatile uint32_t *)0x40038E44))
#define ADC0_DCCMP2_R           (*((volatile uint32_t *)0x40038E48))
#define ADC0_DCCMP3_R           (*((volatile uint32_t *)0x40038E4C))
#define ADC0_DCCMP4_R           (*((volatile uint32_t *)0x40038E50))
#define ADC0_DCCMP5_R           (*((volatile uint32_t *)0x40038E54))
#define ADC0_DCCMP6_R           (*((volatile uint32_t *)0x40038E58))
#define ADC0_DCCMP7_R           (*((volatile uint32_t *)0x40038E5C))
#define ADC0_PP_R               (*((volatile uint32_t *)0x40038FC0))
#define ADC0_PC_R               (*((volatile uint32_t *)0x40038FC4))
#define ADC0_CC_R               (*((volatile uint32_t *)0x40038FC8))

//*****************************************************************************
//
// ADC registers (ADC1)
//
//*****************************************************************************
#define ADC1_ACTSS_R            (*((volatile uint32_t *)0x40039000))
#define ADC1_RIS_R              (*((volatile uint32_t *)0x40039004))
#define ADC1_IM_R               (*((volatile uint32_t *)0x40039008))
#define ADC1_ISC_R              (*((volatile uint32_t *)0x4003900C))
#define ADC1_OSTAT_R            (*((volatile uint32_t *)0x40039010))
#define ADC1_EMUX_R             (*((volatile uint32_t *)0x40039014))
#define ADC1_USTAT_R            (*((volatile uint32_t *)0x40039018))
#define ADC1_TSSEL_R            (*((volatile uint32_t *)0x4003901C))
#define ADC1_SSPRI_R            (*((volatile uint32_t *)0x40039020))
#define ADC1_SPC_R              (*((volatile uint32_t *)0x40039024))
#define ADC1_PSSI_R             (*((volatile uint32_t *)0x40039028))
#define ADC1_SAC_R              (*((volatile uint32_t *)0x40039030))
#define ADC1_DCISC_R            (*((volatile uint32_t *)0x40039034))
#define ADC1_CTL_R              (*((volatile uint32_t *)0x40039038))
#define ADC1_SSMUX0_R           (*((volatile uint32_t *)0x40039040))
#define ADC1_SSCTL0_R           (*((volatile uint32_t *)0x40039044))
#define ADC1_SSFIFO0_R          (*((volatile uint32_t *)0x40039048))
#define ADC1_SSFSTAT0_R         (*((volatile uint32_t *)0x4003904C))
#define ADC1_SSOP0_R            (*((volatile uint32_t *)0x40039050))
#define ADC1_SSDC0_R            (*((volatile uint32_t *)0x40039054))
#define ADC1_SSEMUX0_R          (*((volatile uint32_t *)0x40039058))
#define ADC1_SSTSH0_R           (*((volatile uint32_t *)0x4003905C))
#define ADC1_SSMUX1_R           (*((volatile uint32_t *)0x40039060))
#define ADC1_SSCTL1_R           (*((volatile uint32_t *)0x40039064))
#define ADC1_SSFIFO1_R          (*((volatile uint32_t *)0x40039068))
#define ADC1_SSFSTAT1_R         (*((volatile uint32_t *)0x4003906C))
#define ADC1_SSOP1_R            (*((volatile uint32_t *)0x40039070))
#define ADC1_SSDC1_R            (*((volatile uint32_t *)0x40039074))
#define ADC1_SSEMUX1_R          (*((volatile uint32_t *)0x40039078))
#define ADC1_SSTSH1_R           (*((volatile uint32_t *)0x4003907C))
#define ADC1_SSMUX2_R           (*((volatile uint32_t *)0x40039080))
#define ADC1_SSCTL2_R           (*((volatile uint32_t *)0x40039084))
#define ADC1_SSFIFO2_R          (*((volatile uint32_t *)0x40039088))
#define ADC1_SSFSTAT2_R         (*((volatile uint32_t *)0x4003908C))
#define ADC1_SSOP2_R            (*((volatile uint32_t *)0x40039090))
#define ADC1_SSDC2_R            (*((volatile uint32_t *)0x40039094))
#define ADC1_SSEMUX2_R          (*((volatile uint32_t *)0x40039098))
#define ADC1_SSTSH2_R           (*((volatile uint32_t *)0x4003909C))
#define ADC1_SSMUX3_R           (*((volatile uint32_t *)0x400390A0))
#define ADC1_SSCTL3_R           (*((volatile uint32_t *)0x400390A4))
#define ADC1_SSFIFO3_R          (*((volatile uint32_t *)0x400390A8))
#define ADC1_SSFSTAT3_R         (*((volatile uint32_t *)0x400390AC))
#define ADC1_SSOP3_R            (*((volatile uint32_t *)0x400390B0))
#define ADC1_SSDC3_R            (*((volatile uint32_t *)0x400390B4))
#define ADC1_SSEMUX3_R          (*((volatile uint32_t *)0x400390B8))
#define ADC1_SSTSH3_R           (*((volatile uint32_t *)0x400390BC))
#define ADC1_DCRIC_R            (*((volatile uint32_t *)0x40039D00))
#define ADC1_DCCTL0_R           (*((volatile uint32_t *)0x40039E00))
#define ADC1_DCCTL1_R           (*((volatile uint32_t *)0x40039E04))
#define ADC1_DCCTL2_R           (*((volatile uint32_t *)0x40039E08))
#define ADC1_DCCTL3_R           (*((volatile uint32_t *)0x40039E0C))
#define ADC1_DCCTL4_R           (*((volatile uint32_t *)0x40039E10))
#define ADC1_DCCTL5_R           (*((volatile uint32_t *)0x40039E14))
#define ADC1_DCCTL6_R           (*((volatile uint32_t *)0x40039E18))
#define ADC1_DCCTL7_R           (*((volatile uint32_t *)0x40039E1C))
#define ADC1_DCCMP0_R           (*((volatile uint32_t *)0x40039E40))
#define ADC1_DCCMP1_R           (*((volatile uint32_t *)0x40039E44))
#define ADC1_DCCMP2_R           (*((volatile uint32_t *)0x40039E48))
#define ADC1_DCCMP3_R           (*((volatile uint32_t *)0x40039E4C))
#define ADC1_DCCMP4_R           (*((volatile uint32_t *)0x40039E50))
#define ADC1_DCCMP5_R           (*((volatile uint32_t *)0x40039E54))
#define ADC1_DCCMP6_R           (*((volatile uint32_t *)0x40039E58))
#define ADC1_DCCMP7_R           (*((volatile uint32_t *)0x40039E5C))
#define ADC1_PP_R               (*((volatile uint32_t *)0x40039FC0))
#define ADC1_PC_R               (*((volatile uint32_t *)0x40039FC4))
#define ADC1_CC_R               (*((volatile uint32_t *)0x40039FC8))

//*****************************************************************************
//
// Comparator registers (COMP)
//
//*****************************************************************************
#define COMP_ACMIS_R            (*((volatile uint32_t *)0x4003C000))
#define COMP_ACRIS_R            (*((volatile uint32_t *)0x4003C004))
#define COMP_ACINTEN_R          (*((volatile uint32_t *)0x4003C008))
#define COMP_ACREFCTL_R         (*((volatile uint32_t *)0x4003C010))
#define COMP_ACSTAT0_R          (*((volatile uint32_t *)0x4003C020))
#define COMP_ACCTL0_R           (*((volatile uint32_t *)0x4003C024))
#define COMP_ACSTAT1_R          (*((volatile uint32_t *)0x4003C040))
#define COMP_ACCTL1_R           (*((volatile uint32_t *)0x4003C044))
#define COMP_ACSTAT2_R          (*((volatile uint32_t *)0x4003C060))
#define COMP_ACCTL2_R           (*((volatile uint32_t *)0x4003C064))
#define COMP_PP_R               (*((volatile uint32_t *)0x4003CFC0))

//*****************************************************************************
//
// CAN registers (CAN0)
//
//*****************************************************************************
#define CAN0_CTL_R              (*((volatile uint32_t *)0x40040000))
#define CAN0_STS_R              (*((volatile uint32_t *)0x40040004))
#define CAN0_ERR_R              (*((volatile uint32_t *)0x40040008))
#define CAN0_BIT_R              (*((volatile uint32_t *)0x4004000C))
#define CAN0_INT_R              (*((volatile uint32_t *)0x40040010))
#define CAN0_TST_R              (*((volatile uint32_t *)0x40040014))
#define CAN0_BRPE_R             (*((volatile uint32_t *)0x40040018))
#define CAN0_IF1CRQ_R           (*((volatile uint32_t *)0x40040020))
#define CAN0_IF1CMSK_R          (*((volatile uint32_t *)0x40040024))
#define CAN0_IF1MSK1_R          (*((volatile uint32_t *)0x40040028))
#define CAN0_IF1MSK2_R          (*((volatile uint32_t *)0x4004002C))
#define CAN0_IF1ARB1_R          (*((volatile uint32_t *)0x40040030))
#define CAN0_IF1ARB2_R          (*((volatile uint32_t *)0x40040034))
#define CAN0_IF1MCTL_R          (*((volatile uint32_t *)0x40040038))
#define CAN0_IF1DA1_R           (*((volatile uint32_t *)0x4004003C))
#define CAN0_IF1DA2_R           (*((volatile uint32_t *)0x40040040))
#define CAN0_IF1DB1_R           (*((volatile uint32_t *)0x40040044))
#define CAN0_IF1DB2_R           (*((volatile uint32_t *)0x40040048))
#define CAN0_IF2CRQ_R           (*((volatile uint32_t *)0x40040080))
#define CAN0_IF2CMSK_R          (*((volatile uint32_t *)0x40040084))
#define CAN0_IF2MSK1_R          (*((volatile uint32_t *)0x40040088))
#define CAN0_IF2MSK2_R          (*((volatile uint32_t *)0x4004008C))
#define CAN0_IF2ARB1_R          (*((volatile uint32_t *)0x40040090))
#define CAN0_IF2ARB2_R          (*((volatile uint32_t *)0x40040094))
#define CAN0_IF2MCTL_R          (*((volatile uint32_t *)0x40040098))
#define CAN0_IF2DA1_R           (*((volatile uint32_t *)0x4004009C))
#define CAN0_IF2DA2_R           (*((volatile uint32_t *)0x400400A0))
#define CAN0_IF2DB1_R           (*((volatile uint32_t *)0x400400A4))
#define CAN0_IF2DB2_R           (*((volatile uint32_t *)0x400400A8))
#define CAN0_TXRQ1_R            (*((volatile uint32_t *)0x40040100))
#define CAN0_TXRQ2_R            (*((volatile uint32_t *)0x40040104))
#define CAN0_NWDA1_R            (*((volatile uint32_t *)0x40040120))
#define CAN0_NWDA2_R            (*((volatile uint32_t *)0x40040124))
#define CAN0_MSG1INT_R          (*((volatile uint32_t *)0x40040140))
#define CAN0_MSG2INT_R          (*((volatile uint32_t *)0x40040144))
#define CAN0_MSG1VAL_R          (*((volatile uint32_t *)0x40040160))
#define CAN0_MSG2VAL_R          (*((volatile uint32_t *)0x40040164))

//*****************************************************************************
//
// CAN registers (CAN1)
//
//*****************************************************************************
#define CAN1_CTL_R              (*((volatile uint32_t *)0x40041000))
#define CAN1_STS_R              (*((volatile uint32_t *)0x40041004))
#define CAN1_ERR_R              (*((volatile uint32_t *)0x40041008))
#define CAN1_BIT_R              (*((volatile uint32_t *)0x4004100C))
#define CAN1_INT_R              (*((volatile uint32_t *)0x40041010))
#define CAN1_TST_R              (*((volatile uint32_t *)0x40041014))
#define CAN1_BRPE_R             (*((volatile uint32_t *)0x40041018))
#define CAN1_IF1CRQ_R           (*((volatile uint32_t *)0x40041020))
#define CAN1_IF1CMSK_R          (*((volatile uint32_t *)0x40041024))
#define CAN1_IF1MSK1_R          (*((volatile uint32_t *)0x40041028))
#define CAN1_IF1MSK2_R          (*((volatile uint32_t *)0x4004102C))
#define CAN1_IF1ARB1_R          (*((volatile uint32_t *)0x40041030))
#define CAN1_IF1ARB2_R          (*((volatile uint32_t *)0x40041034))
#define CAN1_IF1MCTL_R          (*((volatile uint32_t *)0x40041038))
#define CAN1_IF1DA1_R           (*((volatile uint32_t *)0x4004103C))
#define CAN1_IF1DA2_R           (*((volatile uint32_t *)0x40041040))
#define CAN1_IF1DB1_R           (*((volatile uint32_t *)0x40041044))
#define CAN1_IF1DB2_R           (*((volatile uint32_t *)0x40041048))
#define CAN1_IF2CRQ_R           (*((volatile uint32_t *)0x40041080))
#define CAN1_IF2CMSK_R          (*((volatile uint32_t *)0x40041084))
#define CAN1_IF2MSK1_R          (*((volatile uint32_t *)0x40041088))
#define CAN1_IF2MSK2_R          (*((volatile uint32_t *)0x4004108C))
#define CAN1_IF2ARB1_R          (*((volatile uint32_t *)0x40041090))
#define CAN1_IF2ARB2_R          (*((volatile uint32_t *)0x40041094))
#define CAN1_IF2MCTL_R          (*((volatile uint32_t *)0x40041098))
#define CAN1_IF2DA1_R           (*((volatile uint32_t *)0x4004109C))
#define CAN1_IF2DA2_R           (*((volatile uint32_t *)0x400410A0))
#define CAN1_IF2DB1_R           (*((volatile uint32_t *)0x400410A4))
#define CAN1_IF2DB2_R           (*((volatile uint32_t *)0x400410A8))
#define CAN1_TXRQ1_R            (*((volatile uint32_t *)0x40041100))
#define CAN1_TXRQ2_R            (*((volatile uint32_t *)0x40041104))
#define CAN1_NWDA1_R            (*((volatile uint32_t *)0x40041120))
#define CAN1_NWDA2_R            (*((volatile uint32_t *)0x40041124))
#define CAN1_MSG1INT_R          (*((volatile uint32_t *)0x40041140))
#define CAN1_MSG2INT_R          (*((volatile uint32_t *)0x40041144))
#define CAN1_MSG1VAL_R          (*((volatile uint32_t *)0x40041160))
#define CAN1_MSG2VAL_R          (*((volatile uint32_t *)0x40041164))

//*****************************************************************************
//
// Univeral Serial Bus registers (USB0)
//
//*****************************************************************************
#define USB0_FADDR_R            (*((volatile uint8_t *)0x40050000))
#define USB0_POWER_R            (*((volatile uint8_t *)0x40050001))
#define USB0_TXIS_R             (*((volatile uint16_t *)0x40050002))
#define USB0_RXIS_R             (*((volatile uint16_t *)0x40050004))
#define USB0_TXIE_R             (*((volatile uint16_t *)0x40050006))
#define USB0_RXIE_R             (*((volatile uint16_t *)0x40050008))
#define USB0_IS_R               (*((volatile uint8_t *)0x4005000A))
#define USB0_IE_R               (*((volatile uint8_t *)0x4005000B))
#define USB0_FRAME_R            (*((volatile uint16_t *)0x4005000C))
#define USB0_EPIDX_R            (*((volatile uint8_t *)0x4005000E))
#define USB0_TEST_R             (*((volatile uint8_t *)0x4005000F))
#define USB0_FIFO0_R            (*((volatile uint32_t *)0x40050020))
#define USB0_FIFO1_R            (*((volatile uint32_t *)0x40050024))
#define USB0_FIFO2_R            (*((volatile uint32_t *)0x40050028))
#define USB0_FIFO3_R            (*((volatile uint32_t *)0x4005002C))
#define USB0_FIFO4_R            (*((volatile uint32_t *)0x40050030))
#define USB0_FIFO5_R            (*((volatile uint32_t *)0x40050034))
#define USB0_FIFO6_R            (*((volatile uint32_t *)0x40050038))
#define USB0_FIFO7_R            (*((volatile uint32_t *)0x4005003C))
#define USB0_DEVCTL_R           (*((volatile uint8_t *)0x40050060))
#define USB0_CCONF_R            (*((volatile uint8_t *)0x40050061))
#define USB0_TXFIFOSZ_R         (*((volatile uint8_t *)0x40050062))
#define USB0_RXFIFOSZ_R         (*((volatile uint8_t *)0x40050063))
#define USB0_TXFIFOADD_R        (*((volatile uint16_t *)0x40050064))
#define USB0_RXFIFOADD_R        (*((volatile uint16_t *)0x40050066))
#define USB0_ULPIVBUSCTL_R      (*((volatile uint8_t *)0x40050070))
#define USB0_ULPIREGDATA_R      (*((volatile uint8_t *)0x40050074))
#define USB0_ULPIREGADDR_R      (*((volatile uint8_t *)0x40050075))
#define USB0_ULPIREGCTL_R       (*((volatile uint8_t *)0x40050076))
#define USB0_EPINFO_R           (*((volatile uint8_t *)0x40050078))
#define USB0_RAMINFO_R          (*((volatile uint8_t *)0x40050079))
#define USB0_CONTIM_R           (*((volatile uint8_t *)0x4005007A))
#define USB0_VPLEN_R            (*((volatile uint8_t *)0x4005007B))
#define USB0_HSEOF_R            (*((volatile uint8_t *)0x4005007C))
#define USB0_FSEOF_R            (*((volatile uint8_t *)0x4005007D))
#define USB0_LSEOF_R            (*((volatile uint8_t *)0x4005007E))
#define USB0_TXFUNCADDR0_R      (*((volatile uint8_t *)0x40050080))
#define USB0_TXHUBADDR0_R       (*((volatile uint8_t *)0x40050082))
#define USB0_TXHUBPORT0_R       (*((volatile uint8_t *)0x40050083))
#define USB0_TXFUNCADDR1_R      (*((volatile uint8_t *)0x40050088))
#define USB0_TXHUBADDR1_R       (*((volatile uint8_t *)0x4005008A))
#define USB0_TXHUBPORT1_R       (*((volatile uint8_t *)0x4005008B))
#define USB0_RXFUNCADDR1_R      (*((volatile uint8_t *)0x4005008C))
#define USB0_RXHUBADDR1_R       (*((volatile uint8_t *)0x4005008E))
#define USB0_RXHUBPORT1_R       (*((volatile uint8_t *)0x4005008F))
#define USB0_TXFUNCADDR2_R      (*((volatile uint8_t *)0x40050090))
#define USB0_TXHUBADDR2_R       (*((volatile uint8_t *)0x40050092))
#define USB0_TXHUBPORT2_R       (*((volatile uint8_t *)0x40050093))
#define USB0_RXFUNCADDR2_R      (*((volatile uint8_t *)0x40050094))
#define USB0_RXHUBADDR2_R       (*((volatile uint8_t *)0x40050096))
#define USB0_RXHUBPORT2_R       (*((volatile uint8_t *)0x40050097))
#define USB0_TXFUNCADDR3_R      (*((volatile uint8_t *)0x40050098))
#define USB0_TXHUBADDR3_R       (*((volatile uint8_t *)0x4005009A))
#define USB0_TXHUBPORT3_R       (*((volatile uint8_t *)0x4005009B))
#define USB0_RXFUNCADDR3_R      (*((volatile uint8_t *)0x4005009C))
#define USB0_RXHUBADDR3_R       (*((volatile uint8_t *)0x4005009E))
#define USB0_RXHUBPORT3_R       (*((volatile uint8_t *)0x4005009F))
#define USB0_TXFUNCADDR4_R      (*((volatile uint8_t *)0x400500A0))
#define USB0_TXHUBADDR4_R       (*((volatile uint8_t *)0x400500A2))
#define USB0_TXHUBPORT4_R       (*((volatile uint8_t *)0x400500A3))
#define USB0_RXFUNCADDR4_R      (*((volatile uint8_t *)0x400500A4))
#define USB0_RXHUBADDR4_R       (*((volatile uint8_t *)0x400500A6))
#define USB0_RXHUBPORT4_R       (*((volatile uint8_t *)0x400500A7))
#define USB0_TXFUNCADDR5_R      (*((volatile uint8_t *)0x400500A8))
#define USB0_TXHUBADDR5_R       (*((volatile uint8_t *)0x400500AA))
#define USB0_TXHUBPORT5_R       (*((volatile uint8_t *)0x400500AB))
#define USB0_RXFUNCADDR5_R      (*((volatile uint8_t *)0x400500AC))
#define USB0_RXHUBADDR5_R       (*((volatile uint8_t *)0x400500AE))
#define USB0_RXHUBPORT5_R       (*((volatile uint8_t *)0x400500AF))
#define USB0_TXFUNCADDR6_R      (*((volatile uint8_t *)0x400500B0))
#define USB0_TXHUBADDR6_R       (*((volatile uint8_t *)0x400500B2))
#define USB0_TXHUBPORT6_R       (*((volatile uint8_t *)0x400500B3))
#define USB0_RXFUNCADDR6_R      (*((volatile uint8_t *)0x400500B4))
#define USB0_RXHUBADDR6_R       (*((volatile uint8_t *)0x400500B6))
#define USB0_RXHUBPORT6_R       (*((volatile uint8_t *)0x400500B7))
#define USB0_TXFUNCADDR7_R      (*((volatile uint8_t *)0x400500B8))
#define USB0_TXHUBADDR7_R       (*((volatile uint8_t *)0x400500BA))
#define USB0_TXHUBPORT7_R       (*((volatile uint8_t *)0x400500BB))
#define USB0_RXFUNCADDR7_R      (*((volatile uint8_t *)0x400500BC))
#define USB0_RXHUBADDR7_R       (*((volatile uint8_t *)0x400500BE))
#define USB0_RXHUBPORT7_R       (*((volatile uint8_t *)0x400500BF))
#define USB0_CSRL0_R            (*((volatile uint8_t *)0x40050102))
#define USB0_CSRH0_R            (*((volatile uint8_t *)0x40050103))
#define USB0_COUNT0_R           (*((volatile uint8_t *)0x40050108))
#define USB0_TYPE0_R            (*((volatile uint8_t *)0x4005010A))
#define USB0_NAKLMT_R           (*((volatile uint8_t *)0x4005010B))
#define USB0_TXMAXP1_R          (*((volatile uint16_t *)0x40050110))
#define USB0_TXCSRL1_R          (*((volatile uint8_t *)0x40050112))
#define USB0_TXCSRH1_R          (*((volatile uint8_t *)0x40050113))
#define USB0_RXMAXP1_R          (*((volatile uint16_t *)0x40050114))
#define USB0_RXCSRL1_R          (*((volatile uint8_t *)0x40050116))
#define USB0_RXCSRH1_R          (*((volatile uint8_t *)0x40050117))
#define USB0_RXCOUNT1_R         (*((volatile uint16_t *)0x40050118))
#define USB0_TXTYPE1_R          (*((volatile uint8_t *)0x4005011A))
#define USB0_TXINTERVAL1_R      (*((volatile uint8_t *)0x4005011B))
#define USB0_RXTYPE1_R          (*((volatile uint8_t *)0x4005011C))
#define USB0_RXINTERVAL1_R      (*((volatile uint8_t *)0x4005011D))
#define USB0_TXMAXP2_R          (*((volatile uint16_t *)0x40050120))
#define USB0_TXCSRL2_R          (*((volatile uint8_t *)0x40050122))
#define USB0_TXCSRH2_R          (*((volatile uint8_t *)0x40050123))
#define USB0_RXMAXP2_R          (*((volatile uint16_t *)0x40050124))
#define USB0_RXCSRL2_R          (*((volatile uint8_t *)0x40050126))
#define USB0_RXCSRH2_R          (*((volatile uint8_t *)0x40050127))
#define USB0_RXCOUNT2_R         (*((volatile uint16_t *)0x40050128))
#define USB0_TXTYPE2_R          (*((volatile uint8_t *)0x4005012A))
#define USB0_TXINTERVAL2_R      (*((volatile uint8_t *)0x4005012B))
#define USB0_RXTYPE2_R          (*((volatile uint8_t *)0x4005012C))
#define USB0_RXINTERVAL2_R      (*((volatile uint8_t *)0x4005012D))
#define USB0_TXMAXP3_R          (*((volatile uint16_t *)0x40050130))
#define USB0_TXCSRL3_R          (*((volatile uint8_t *)0x40050132))
#define USB0_TXCSRH3_R          (*((volatile uint8_t *)0x40050133))
#define USB0_RXMAXP3_R          (*((volatile uint16_t *)0x40050134))
#define USB0_RXCSRL3_R          (*((volatile uint8_t *)0x40050136))
#define USB0_RXCSRH3_R          (*((volatile uint8_t *)0x40050137))
#define USB0_RXCOUNT3_R         (*((volatile uint16_t *)0x40050138))
#define USB0_TXTYPE3_R          (*((volatile uint8_t *)0x4005013A))
#define USB0_TXINTERVAL3_R      (*((volatile uint8_t *)0x4005013B))
#define USB0_RXTYPE3_R          (*((volatile uint8_t *)0x4005013C))
#define USB0_RXINTERVAL3_R      (*((volatile uint8_t *)0x4005013D))
#define USB0_TXMAXP4_R          (*((volatile uint16_t *)0x40050140))
#define USB0_TXCSRL4_R          (*((volatile uint8_t *)0x40050142))
#define USB0_TXCSRH4_R          (*((volatile uint8_t *)0x40050143))
#define USB0_RXMAXP4_R          (*((volatile uint16_t *)0x40050144))
#define USB0_RXCSRL4_R          (*((volatile uint8_t *)0x40050146))
#define USB0_RXCSRH4_R          (*((volatile uint8_t *)0x40050147))
#define USB0_RXCOUNT4_R         (*((volatile uint16_t *)0x40050148))
#define USB0_TXTYPE4_R          (*((volatile uint8_t *)0x4005014A))
#define USB0_TXINTERVAL4_R      (*((volatile uint8_t *)0x4005014B))
#define USB0_RXTYPE4_R          (*((volatile uint8_t *)0x4005014C))
#define USB0_RXINTERVAL4_R      (*((volatile uint8_t *)0x4005014D))
#define USB0_TXMAXP5_R          (*((volatile uint16_t *)0x40050150))
#define USB0_TXCSRL5_R          (*((volatile uint8_t *)0x40050152))
#define USB0_TXCSRH5_R          (*((volatile uint8_t *)0x40050153))
#define USB0_RXMAXP5_R          (*((volatile uint16_t *)0x40050154))
#define USB0_RXCSRL5_R          (*((volatile uint8_t *)0x40050156))
#define USB0_RXCSRH5_R          (*((volatile uint8_t *)0x40050157))
#define USB0_RXCOUNT5_R         (*((volatile uint16_t *)0x40050158))
#define USB0_TXTYPE5_R          (*((volatile uint8_t *)0x4005015A))
#define USB0_TXINTERVAL5_R      (*((volatile uint8_t *)0x4005015B))
#define USB0_RXTYPE5_R          (*((volatile uint8_t *)0x4005015C))
#define USB0_RXINTERVAL5_R      (*((volatile uint8_t *)0x4005015D))
#define USB0_TXMAXP6_R          (*((volatile uint16_t *)0x40050160))
#define USB0_TXCSRL6_R          (*((volatile uint8_t *)0x40050162))
#define USB0_TXCSRH6_R          (*((volatile uint8_t *)0x40050163))
#define USB0_RXMAXP6_R          (*((volatile uint16_t *)0x40050164))
#define USB0_RXCSRL6_R          (*((volatile uint8_t *)0x40050166))
#define USB0_RXCSRH6_R          (*((volatile uint8_t *)0x40050167))
#define USB0_RXCOUNT6_R         (*((volatile uint16_t *)0x40050168))
#define USB0_TXTYPE6_R          (*((volatile uint8_t *)0x4005016A))
#define USB0_TXINTERVAL6_R      (*((volatile uint8_t *)0x4005016B))
#define USB0_RXTYPE6_R          (*((volatile uint8_t *)0x4005016C))
#define USB0_RXINTERVAL6_R      (*((volatile uint8_t *)0x4005016D))
#define USB0_TXMAXP7_R          (*((volatile uint16_t *)0x40050170))
#define USB0_TXCSRL7_R          (*((volatile uint8_t *)0x40050172))
#define USB0_TXCSRH7_R          (*((volatile uint8_t *)0x40050173))
#define USB0_RXMAXP7_R          (*((volatile uint16_t *)0x40050174))
#define USB0_RXCSRL7_R          (*((volatile uint8_t *)0x40050176))
#define USB0_RXCSRH7_R          (*((volatile uint8_t *)0x40050177))
#define USB0_RXCOUNT7_R         (*((volatile uint16_t *)0x40050178))
#define USB0_TXTYPE7_R          (*((volatile uint8_t *)0x4005017A))
#define USB0_TXINTERVAL7_R      (*((volatile uint8_t *)0x4005017B))
#define USB0_RXTYPE7_R          (*((volatile uint8_t *)0x4005017C))
#define USB0_RXINTERVAL7_R      (*((volatile uint8_t *)0x4005017D))
#define USB0_DMAINTR_R          (*((volatile uint8_t *)0x40050200))
#define USB0_DMACTL0_R          (*((volatile uint16_t *)0x40050204))
#define USB0_DMAADDR0_R         (*((volatile uint32_t *)0x40050208))
#define USB0_DMACOUNT0_R        (*((volatile uint32_t *)0x4005020C))
#define USB0_DMACTL1_R          (*((volatile uint16_t *)0x40050214))
#define USB0_DMAADDR1_R         (*((volatile uint32_t *)0x40050218))
#define USB0_DMACOUNT1_R        (*((volatile uint32_t *)0x4005021C))
#define USB0_DMACTL2_R          (*((volatile uint16_t *)0x40050224))
#define USB0_DMAADDR2_R         (*((volatile uint32_t *)0x40050228))
#define USB0_DMACOUNT2_R        (*((volatile uint32_t *)0x4005022C))
#define USB0_DMACTL3_R          (*((volatile uint16_t *)0x40050234))
#define USB0_DMAADDR3_R         (*((volatile uint32_t *)0x40050238))
#define USB0_DMACOUNT3_R        (*((volatile uint32_t *)0x4005023C))
#define USB0_DMACTL4_R          (*((volatile uint16_t *)0x40050244))
#define USB0_DMAADDR4_R         (*((volatile uint32_t *)0x40050248))
#define USB0_DMACOUNT4_R        (*((volatile uint32_t *)0x4005024C))
#define USB0_DMACTL5_R          (*((volatile uint16_t *)0x40050254))
#define USB0_DMAADDR5_R         (*((volatile uint32_t *)0x40050258))
#define USB0_DMACOUNT5_R        (*((volatile uint32_t *)0x4005025C))
#define USB0_DMACTL6_R          (*((volatile uint16_t *)0x40050264))
#define USB0_DMAADDR6_R         (*((volatile uint32_t *)0x40050268))
#define USB0_DMACOUNT6_R        (*((volatile uint32_t *)0x4005026C))
#define USB0_DMACTL7_R          (*((volatile uint16_t *)0x40050274))
#define USB0_DMAADDR7_R         (*((volatile uint32_t *)0x40050278))
#define USB0_DMACOUNT7_R        (*((volatile uint32_t *)0x4005027C))
#define USB0_RQPKTCOUNT1_R      (*((volatile uint16_t *)0x40050304))
#define USB0_RQPKTCOUNT2_R      (*((volatile uint16_t *)0x40050308))
#define USB0_RQPKTCOUNT3_R      (*((volatile uint16_t *)0x4005030C))
#define USB0_RQPKTCOUNT4_R      (*((volatile uint16_t *)0x40050310))
#define USB0_RQPKTCOUNT5_R      (*((volatile uint16_t *)0x40050314))
#define USB0_RQPKTCOUNT6_R      (*((volatile uint16_t *)0x40050318))
#define USB0_RQPKTCOUNT7_R      (*((volatile uint16_t *)0x4005031C))
#define USB0_RXDPKTBUFDIS_R     (*((volatile uint16_t *)0x40050340))
#define USB0_TXDPKTBUFDIS_R     (*((volatile uint16_t *)0x40050342))
#define USB0_CTO_R              (*((volatile uint16_t *)0x40050344))
#define USB0_HHSRTN_R           (*((volatile uint16_t *)0x40050346))
#define USB0_HSBT_R             (*((volatile uint16_t *)0x40050348))
#define USB0_LPMATTR_R          (*((volatile uint16_t *)0x40050360))
#define USB0_LPMCNTRL_R         (*((volatile uint8_t *)0x40050362))
#define USB0_LPMIM_R            (*((volatile uint8_t *)0x40050363))
#define USB0_LPMRIS_R           (*((volatile uint8_t *)0x40050364))
#define USB0_LPMFADDR_R         (*((volatile uint8_t *)0x40050365))
#define USB0_EPC_R              (*((volatile uint32_t *)0x40050400))
#define USB0_EPCRIS_R           (*((volatile uint32_t *)0x40050404))
#define USB0_EPCIM_R            (*((volatile uint32_t *)0x40050408))
#define USB0_EPCISC_R           (*((volatile uint32_t *)0x4005040C))
#define USB0_DRRIS_R            (*((volatile uint32_t *)0x40050410))
#define USB0_DRIM_R             (*((volatile uint32_t *)0x40050414))
#define USB0_DRISC_R            (*((volatile uint32_t *)0x40050418))
#define USB0_GPCS_R             (*((volatile uint32_t *)0x4005041C))
#define USB0_VDC_R              (*((volatile uint32_t *)0x40050430))
#define USB0_VDCRIS_R           (*((volatile uint32_t *)0x40050434))
#define USB0_VDCIM_R            (*((volatile uint32_t *)0x40050438))
#define USB0_VDCISC_R           (*((volatile uint32_t *)0x4005043C))
#define USB0_PP_R               (*((volatile uint32_t *)0x40050FC0))
#define USB0_PC_R               (*((volatile uint32_t *)0x40050FC4))
#define USB0_CC_R               (*((volatile uint32_t *)0x40050FC8))

//*****************************************************************************
//
// GPIO registers (PORTA AHB)
//
//*****************************************************************************
#define GPIO_PORTA_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x40058000)
#define GPIO_PORTA_AHB_DATA_R   (*((volatile uint32_t *)0x400583FC))
#define GPIO_PORTA_AHB_DIR_R    (*((volatile uint32_t *)0x40058400))
#define GPIO_PORTA_AHB_IS_R     (*((volatile uint32_t *)0x40058404))
#define GPIO_PORTA_AHB_IBE_R    (*((volatile uint32_t *)0x40058408))
#define GPIO_PORTA_AHB_IEV_R    (*((volatile uint32_t *)0x4005840C))
#define GPIO_PORTA_AHB_IM_R     (*((volatile uint32_t *)0x40058410))
#define GPIO_PORTA_AHB_RIS_R    (*((volatile uint32_t *)0x40058414))
#define GPIO_PORTA_AHB_MIS_R    (*((volatile uint32_t *)0x40058418))
#define GPIO_PORTA_AHB_ICR_R    (*((volatile uint32_t *)0x4005841C))
#define GPIO_PORTA_AHB_AFSEL_R  (*((volatile uint32_t *)0x40058420))
#define GPIO_PORTA_AHB_DR2R_R   (*((volatile uint32_t *)0x40058500))
#define GPIO_PORTA_AHB_DR4R_R   (*((volatile uint32_t *)0x40058504))
#define GPIO_PORTA_AHB_DR8R_R   (*((volatile uint32_t *)0x40058508))
#define GPIO_PORTA_AHB_ODR_R    (*((volatile uint32_t *)0x4005850C))
#define GPIO_PORTA_AHB_PUR_R    (*((volatile uint32_t *)0x40058510))
#define GPIO_PORTA_AHB_PDR_R    (*((volatile uint32_t *)0x40058514))
#define GPIO_PORTA_AHB_SLR_R    (*((volatile uint32_t *)0x40058518))
#define GPIO_PORTA_AHB_DEN_R    (*((volatile uint32_t *)0x4005851C))
#define GPIO_PORTA_AHB_LOCK_R   (*((volatile uint32_t *)0x40058520))
#define GPIO_PORTA_AHB_CR_R     (*((volatile uint32_t *)0x40058524))
#define GPIO_PORTA_AHB_AMSEL_R  (*((volatile uint32_t *)0x40058528))
#define GPIO_PORTA_AHB_PCTL_R   (*((volatile uint32_t *)0x4005852C))
#define GPIO_PORTA_AHB_ADCCTL_R (*((volatile uint32_t *)0x40058530))
#define GPIO_PORTA_AHB_DMACTL_R (*((volatile uint32_t *)0x40058534))
#define GPIO_PORTA_AHB_SI_R     (*((volatile uint32_t *)0x40058538))
#define GPIO_PORTA_AHB_DR12R_R  (*((volatile uint32_t *)0x4005853C))
#define GPIO_PORTA_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x40058540))
#define GPIO_PORTA_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x40058544))
#define GPIO_PORTA_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x40058548))
#define GPIO_PORTA_AHB_PP_R     (*((volatile uint32_t *)0x40058FC0))
#define GPIO_PORTA_AHB_PC_R     (*((volatile uint32_t *)0x40058FC4))

//*****************************************************************************
//
// GPIO registers (PORTB AHB)
//
//*****************************************************************************
#define GPIO_PORTB_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x40059000)
#define GPIO_PORTB_AHB_DATA_R   (*((volatile uint32_t *)0x400593FC))
#define GPIO_PORTB_AHB_DIR_R    (*((volatile uint32_t *)0x40059400))
#define GPIO_PORTB_AHB_IS_R     (*((volatile uint32_t *)0x40059404))
#define GPIO_PORTB_AHB_IBE_R    (*((volatile uint32_t *)0x40059408))
#define GPIO_PORTB_AHB_IEV_R    (*((volatile uint32_t *)0x4005940C))
#define GPIO_PORTB_AHB_IM_R     (*((volatile uint32_t *)0x40059410))
#define GPIO_PORTB_AHB_RIS_R    (*((volatile uint32_t *)0x40059414))
#define GPIO_PORTB_AHB_MIS_R    (*((volatile uint32_t *)0x40059418))
#define GPIO_PORTB_AHB_ICR_R    (*((volatile uint32_t *)0x4005941C))
#define GPIO_PORTB_AHB_AFSEL_R  (*((volatile uint32_t *)0x40059420))
#define GPIO_PORTB_AHB_DR2R_R   (*((volatile uint32_t *)0x40059500))
#define GPIO_PORTB_AHB_DR4R_R   (*((volatile uint32_t *)0x40059504))
#define GPIO_PORTB_AHB_DR8R_R   (*((volatile uint32_t *)0x40059508))
#define GPIO_PORTB_AHB_ODR_R    (*((volatile uint32_t *)0x4005950C))
#define GPIO_PORTB_AHB_PUR_R    (*((volatile uint32_t *)0x40059510))
#define GPIO_PORTB_AHB_PDR_R    (*((volatile uint32_t *)0x40059514))
#define GPIO_PORTB_AHB_SLR_R    (*((volatile uint32_t *)0x40059518))
#define GPIO_PORTB_AHB_DEN_R    (*((volatile uint32_t *)0x4005951C))
#define GPIO_PORTB_AHB_LOCK_R   (*((volatile uint32_t *)0x40059520))
#define GPIO_PORTB_AHB_CR_R     (*((volatile uint32_t *)0x40059524))
#define GPIO_PORTB_AHB_AMSEL_R  (*((volatile uint32_t *)0x40059528))
#define GPIO_PORTB_AHB_PCTL_R   (*((volatile uint32_t *)0x4005952C))
#define GPIO_PORTB_AHB_ADCCTL_R (*((volatile uint32_t *)0x40059530))
#define GPIO_PORTB_AHB_DMACTL_R (*((volatile uint32_t *)0x40059534))
#define GPIO_PORTB_AHB_SI_R     (*((volatile uint32_t *)0x40059538))
#define GPIO_PORTB_AHB_DR12R_R  (*((volatile uint32_t *)0x4005953C))
#define GPIO_PORTB_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x40059540))
#define GPIO_PORTB_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x40059544))
#define GPIO_PORTB_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x40059548))
#define GPIO_PORTB_AHB_PP_R     (*((volatile uint32_t *)0x40059FC0))
#define GPIO_PORTB_AHB_PC_R     (*((volatile uint32_t *)0x40059FC4))

//*****************************************************************************
//
// GPIO registers (PORTC AHB)
//
//*****************************************************************************
#define GPIO_PORTC_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x4005A000)
#define GPIO_PORTC_AHB_DATA_R   (*((volatile uint32_t *)0x4005A3FC))
#define GPIO_PORTC_AHB_DIR_R    (*((volatile uint32_t *)0x4005A400))
#define GPIO_PORTC_AHB_IS_R     (*((volatile uint32_t *)0x4005A404))
#define GPIO_PORTC_AHB_IBE_R    (*((volatile uint32_t *)0x4005A408))
#define GPIO_PORTC_AHB_IEV_R    (*((volatile uint32_t *)0x4005A40C))
#define GPIO_PORTC_AHB_IM_R     (*((volatile uint32_t *)0x4005A410))
#define GPIO_PORTC_AHB_RIS_R    (*((volatile uint32_t *)0x4005A414))
#define GPIO_PORTC_AHB_MIS_R    (*((volatile uint32_t *)0x4005A418))
#define GPIO_PORTC_AHB_ICR_R    (*((volatile uint32_t *)0x4005A41C))
#define GPIO_PORTC_AHB_AFSEL_R  (*((volatile uint32_t *)0x4005A420))
#define GPIO_PORTC_AHB_DR2R_R   (*((volatile uint32_t *)0x4005A500))
#define GPIO_PORTC_AHB_DR4R_R   (*((volatile uint32_t *)0x4005A504))
#define GPIO_PORTC_AHB_DR8R_R   (*((volatile uint32_t *)0x4005A508))
#define GPIO_PORTC_AHB_ODR_R    (*((volatile uint32_t *)0x4005A50C))
#define GPIO_PORTC_AHB_PUR_R    (*((volatile uint32_t *)0x4005A510))
#define GPIO_PORTC_AHB_PDR_R    (*((volatile uint32_t *)0x4005A514))
#define GPIO_PORTC_AHB_SLR_R    (*((volatile uint32_t *)0x4005A518))
#define GPIO_PORTC_AHB_DEN_R    (*((volatile uint32_t *)0x4005A51C))
#define GPIO_PORTC_AHB_LOCK_R   (*((volatile uint32_t *)0x4005A520))
#define GPIO_PORTC_AHB_CR_R     (*((volatile uint32_t *)0x4005A524))
#define GPIO_PORTC_AHB_AMSEL_R  (*((volatile uint32_t *)0x4005A528))
#define GPIO_PORTC_AHB_PCTL_R   (*((volatile uint32_t *)0x4005A52C))
#define GPIO_PORTC_AHB_ADCCTL_R (*((volatile uint32_t *)0x4005A530))
#define GPIO_PORTC_AHB_DMACTL_R (*((volatile uint32_t *)0x4005A534))
#define GPIO_PORTC_AHB_SI_R     (*((volatile uint32_t *)0x4005A538))
#define GPIO_PORTC_AHB_DR12R_R  (*((volatile uint32_t *)0x4005A53C))
#define GPIO_PORTC_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x4005A540))
#define GPIO_PORTC_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x4005A544))
#define GPIO_PORTC_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x4005A548))
#define GPIO_PORTC_AHB_PP_R     (*((volatile uint32_t *)0x4005AFC0))
#define GPIO_PORTC_AHB_PC_R     (*((volatile uint32_t *)0x4005AFC4))

//*****************************************************************************
//
// GPIO registers (PORTD AHB)
//
//*****************************************************************************
#define GPIO_PORTD_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x4005B000)
#define GPIO_PORTD_AHB_DATA_R   (*((volatile uint32_t *)0x4005B3FC))
#define GPIO_PORTD_AHB_DIR_R    (*((volatile uint32_t *)0x4005B400))
#define GPIO_PORTD_AHB_IS_R     (*((volatile uint32_t *)0x4005B404))
#define GPIO_PORTD_AHB_IBE_R    (*((volatile uint32_t *)0x4005B408))
#define GPIO_PORTD_AHB_IEV_R    (*((volatile uint32_t *)0x4005B40C))
#define GPIO_PORTD_AHB_IM_R     (*((volatile uint32_t *)0x4005B410))
#define GPIO_PORTD_AHB_RIS_R    (*((volatile uint32_t *)0x4005B414))
#define GPIO_PORTD_AHB_MIS_R    (*((volatile uint32_t *)0x4005B418))
#define GPIO_PORTD_AHB_ICR_R    (*((volatile uint32_t *)0x4005B41C))
#define GPIO_PORTD_AHB_AFSEL_R  (*((volatile uint32_t *)0x4005B420))
#define GPIO_PORTD_AHB_DR2R_R   (*((volatile uint32_t *)0x4005B500))
#define GPIO_PORTD_AHB_DR4R_R   (*((volatile uint32_t *)0x4005B504))
#define GPIO_PORTD_AHB_DR8R_R   (*((volatile uint32_t *)0x4005B508))
#define GPIO_PORTD_AHB_ODR_R    (*((volatile uint32_t *)0x4005B50C))
#define GPIO_PORTD_AHB_PUR_R    (*((volatile uint32_t *)0x4005B510))
#define GPIO_PORTD_AHB_PDR_R    (*((volatile uint32_t *)0x4005B514))
#define GPIO_PORTD_AHB_SLR_R    (*((volatile uint32_t *)0x4005B518))
#define GPIO_PORTD_AHB_DEN_R    (*((volatile uint32_t *)0x4005B51C))
#define GPIO_PORTD_AHB_LOCK_R   (*((volatile uint32_t *)0x4005B520))
#define GPIO_PORTD_AHB_CR_R     (*((volatile uint32_t *)0x4005B524))
#define GPIO_PORTD_AHB_AMSEL_R  (*((volatile uint32_t *)0x4005B528))
#define GPIO_PORTD_AHB_PCTL_R   (*((volatile uint32_t *)0x4005B52C))
#define GPIO_PORTD_AHB_ADCCTL_R (*((volatile uint32_t *)0x4005B530))
#define GPIO_PORTD_AHB_DMACTL_R (*((volatile uint32_t *)0x4005B534))
#define GPIO_PORTD_AHB_SI_R     (*((volatile uint32_t *)0x4005B538))
#define GPIO_PORTD_AHB_DR12R_R  (*((volatile uint32_t *)0x4005B53C))
#define GPIO_PORTD_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x4005B540))
#define GPIO_PORTD_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x4005B544))
#define GPIO_PORTD_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x4005B548))
#define GPIO_PORTD_AHB_PP_R     (*((volatile uint32_t *)0x4005BFC0))
#define GPIO_PORTD_AHB_PC_R     (*((volatile uint32_t *)0x4005BFC4))

//*****************************************************************************
//
// GPIO registers (PORTE AHB)
//
//*****************************************************************************
#define GPIO_PORTE_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x4005C000)
#define GPIO_PORTE_AHB_DATA_R   (*((volatile uint32_t *)0x4005C3FC))
#define GPIO_PORTE_AHB_DIR_R    (*((volatile uint32_t *)0x4005C400))
#define GPIO_PORTE_AHB_IS_R     (*((volatile uint32_t *)0x4005C404))
#define GPIO_PORTE_AHB_IBE_R    (*((volatile uint32_t *)0x4005C408))
#define GPIO_PORTE_AHB_IEV_R    (*((volatile uint32_t *)0x4005C40C))
#define GPIO_PORTE_AHB_IM_R     (*((volatile uint32_t *)0x4005C410))
#define GPIO_PORTE_AHB_RIS_R    (*((volatile uint32_t *)0x4005C414))
#define GPIO_PORTE_AHB_MIS_R    (*((volatile uint32_t *)0x4005C418))
#define GPIO_PORTE_AHB_ICR_R    (*((volatile uint32_t *)0x4005C41C))
#define GPIO_PORTE_AHB_AFSEL_R  (*((volatile uint32_t *)0x4005C420))
#define GPIO_PORTE_AHB_DR2R_R   (*((volatile uint32_t *)0x4005C500))
#define GPIO_PORTE_AHB_DR4R_R   (*((volatile uint32_t *)0x4005C504))
#define GPIO_PORTE_AHB_DR8R_R   (*((volatile uint32_t *)0x4005C508))
#define GPIO_PORTE_AHB_ODR_R    (*((volatile uint32_t *)0x4005C50C))
#define GPIO_PORTE_AHB_PUR_R    (*((volatile uint32_t *)0x4005C510))
#define GPIO_PORTE_AHB_PDR_R    (*((volatile uint32_t *)0x4005C514))
#define GPIO_PORTE_AHB_SLR_R    (*((volatile uint32_t *)0x4005C518))
#define GPIO_PORTE_AHB_DEN_R    (*((volatile uint32_t *)0x4005C51C))
#define GPIO_PORTE_AHB_LOCK_R   (*((volatile uint32_t *)0x4005C520))
#define GPIO_PORTE_AHB_CR_R     (*((volatile uint32_t *)0x4005C524))
#define GPIO_PORTE_AHB_AMSEL_R  (*((volatile uint32_t *)0x4005C528))
#define GPIO_PORTE_AHB_PCTL_R   (*((volatile uint32_t *)0x4005C52C))
#define GPIO_PORTE_AHB_ADCCTL_R (*((volatile uint32_t *)0x4005C530))
#define GPIO_PORTE_AHB_DMACTL_R (*((volatile uint32_t *)0x4005C534))
#define GPIO_PORTE_AHB_SI_R     (*((volatile uint32_t *)0x4005C538))
#define GPIO_PORTE_AHB_DR12R_R  (*((volatile uint32_t *)0x4005C53C))
#define GPIO_PORTE_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x4005C540))
#define GPIO_PORTE_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x4005C544))
#define GPIO_PORTE_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x4005C548))
#define GPIO_PORTE_AHB_PP_R     (*((volatile uint32_t *)0x4005CFC0))
#define GPIO_PORTE_AHB_PC_R     (*((volatile uint32_t *)0x4005CFC4))

//*****************************************************************************
//
// GPIO registers (PORTF AHB)
//
//*****************************************************************************
#define GPIO_PORTF_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x4005D000)
#define GPIO_PORTF_AHB_DATA_R   (*((volatile uint32_t *)0x4005D3FC))
#define GPIO_PORTF_AHB_DIR_R    (*((volatile uint32_t *)0x4005D400))
#define GPIO_PORTF_AHB_IS_R     (*((volatile uint32_t *)0x4005D404))
#define GPIO_PORTF_AHB_IBE_R    (*((volatile uint32_t *)0x4005D408))
#define GPIO_PORTF_AHB_IEV_R    (*((volatile uint32_t *)0x4005D40C))
#define GPIO_PORTF_AHB_IM_R     (*((volatile uint32_t *)0x4005D410))
#define GPIO_PORTF_AHB_RIS_R    (*((volatile uint32_t *)0x4005D414))
#define GPIO_PORTF_AHB_MIS_R    (*((volatile uint32_t *)0x4005D418))
#define GPIO_PORTF_AHB_ICR_R    (*((volatile uint32_t *)0x4005D41C))
#define GPIO_PORTF_AHB_AFSEL_R  (*((volatile uint32_t *)0x4005D420))
#define GPIO_PORTF_AHB_DR2R_R   (*((volatile uint32_t *)0x4005D500))
#define GPIO_PORTF_AHB_DR4R_R   (*((volatile uint32_t *)0x4005D504))
#define GPIO_PORTF_AHB_DR8R_R   (*((volatile uint32_t *)0x4005D508))
#define GPIO_PORTF_AHB_ODR_R    (*((volatile uint32_t *)0x4005D50C))
#define GPIO_PORTF_AHB_PUR_R    (*((volatile uint32_t *)0x4005D510))
#define GPIO_PORTF_AHB_PDR_R    (*((volatile uint32_t *)0x4005D514))
#define GPIO_PORTF_AHB_SLR_R    (*((volatile uint32_t *)0x4005D518))
#define GPIO_PORTF_AHB_DEN_R    (*((volatile uint32_t *)0x4005D51C))
#define GPIO_PORTF_AHB_LOCK_R   (*((volatile uint32_t *)0x4005D520))
#define GPIO_PORTF_AHB_CR_R     (*((volatile uint32_t *)0x4005D524))
#define GPIO_PORTF_AHB_AMSEL_R  (*((volatile uint32_t *)0x4005D528))
#define GPIO_PORTF_AHB_PCTL_R   (*((volatile uint32_t *)0x4005D52C))
#define GPIO_PORTF_AHB_ADCCTL_R (*((volatile uint32_t *)0x4005D530))
#define GPIO_PORTF_AHB_DMACTL_R (*((volatile uint32_t *)0x4005D534))
#define GPIO_PORTF_AHB_SI_R     (*((volatile uint32_t *)0x4005D538))
#define GPIO_PORTF_AHB_DR12R_R  (*((volatile uint32_t *)0x4005D53C))
#define GPIO_PORTF_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x4005D540))
#define GPIO_PORTF_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x4005D544))
#define GPIO_PORTF_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x4005D548))
#define GPIO_PORTF_AHB_PP_R     (*((volatile uint32_t *)0x4005DFC0))
#define GPIO_PORTF_AHB_PC_R     (*((volatile uint32_t *)0x4005DFC4))

//*****************************************************************************
//
// GPIO registers (PORTG AHB)
//
//*****************************************************************************
#define GPIO_PORTG_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x4005E000)
#define GPIO_PORTG_AHB_DATA_R   (*((volatile uint32_t *)0x4005E3FC))
#define GPIO_PORTG_AHB_DIR_R    (*((volatile uint32_t *)0x4005E400))
#define GPIO_PORTG_AHB_IS_R     (*((volatile uint32_t *)0x4005E404))
#define GPIO_PORTG_AHB_IBE_R    (*((volatile uint32_t *)0x4005E408))
#define GPIO_PORTG_AHB_IEV_R    (*((volatile uint32_t *)0x4005E40C))
#define GPIO_PORTG_AHB_IM_R     (*((volatile uint32_t *)0x4005E410))
#define GPIO_PORTG_AHB_RIS_R    (*((volatile uint32_t *)0x4005E414))
#define GPIO_PORTG_AHB_MIS_R    (*((volatile uint32_t *)0x4005E418))
#define GPIO_PORTG_AHB_ICR_R    (*((volatile uint32_t *)0x4005E41C))
#define GPIO_PORTG_AHB_AFSEL_R  (*((volatile uint32_t *)0x4005E420))
#define GPIO_PORTG_AHB_DR2R_R   (*((volatile uint32_t *)0x4005E500))
#define GPIO_PORTG_AHB_DR4R_R   (*((volatile uint32_t *)0x4005E504))
#define GPIO_PORTG_AHB_DR8R_R   (*((volatile uint32_t *)0x4005E508))
#define GPIO_PORTG_AHB_ODR_R    (*((volatile uint32_t *)0x4005E50C))
#define GPIO_PORTG_AHB_PUR_R    (*((volatile uint32_t *)0x4005E510))
#define GPIO_PORTG_AHB_PDR_R    (*((volatile uint32_t *)0x4005E514))
#define GPIO_PORTG_AHB_SLR_R    (*((volatile uint32_t *)0x4005E518))
#define GPIO_PORTG_AHB_DEN_R    (*((volatile uint32_t *)0x4005E51C))
#define GPIO_PORTG_AHB_LOCK_R   (*((volatile uint32_t *)0x4005E520))
#define GPIO_PORTG_AHB_CR_R     (*((volatile uint32_t *)0x4005E524))
#define GPIO_PORTG_AHB_AMSEL_R  (*((volatile uint32_t *)0x4005E528))
#define GPIO_PORTG_AHB_PCTL_R   (*((volatile uint32_t *)0x4005E52C))
#define GPIO_PORTG_AHB_ADCCTL_R (*((volatile uint32_t *)0x4005E530))
#define GPIO_PORTG_AHB_DMACTL_R (*((volatile uint32_t *)0x4005E534))
#define GPIO_PORTG_AHB_SI_R     (*((volatile uint32_t *)0x4005E538))
#define GPIO_PORTG_AHB_DR12R_R  (*((volatile uint32_t *)0x4005E53C))
#define GPIO_PORTG_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x4005E540))
#define GPIO_PORTG_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x4005E544))
#define GPIO_PORTG_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x4005E548))
#define GPIO_PORTG_AHB_PP_R     (*((volatile uint32_t *)0x4005EFC0))
#define GPIO_PORTG_AHB_PC_R     (*((volatile uint32_t *)0x4005EFC4))

//*****************************************************************************
//
// GPIO registers (PORTH AHB)
//
//*****************************************************************************
#define GPIO_PORTH_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x4005F000)
#define GPIO_PORTH_AHB_DATA_R   (*((volatile uint32_t *)0x4005F3FC))
#define GPIO_PORTH_AHB_DIR_R    (*((volatile uint32_t *)0x4005F400))
#define GPIO_PORTH_AHB_IS_R     (*((volatile uint32_t *)0x4005F404))
#define GPIO_PORTH_AHB_IBE_R    (*((volatile uint32_t *)0x4005F408))
#define GPIO_PORTH_AHB_IEV_R    (*((volatile uint32_t *)0x4005F40C))
#define GPIO_PORTH_AHB_IM_R     (*((volatile uint32_t *)0x4005F410))
#define GPIO_PORTH_AHB_RIS_R    (*((volatile uint32_t *)0x4005F414))
#define GPIO_PORTH_AHB_MIS_R    (*((volatile uint32_t *)0x4005F418))
#define GPIO_PORTH_AHB_ICR_R    (*((volatile uint32_t *)0x4005F41C))
#define GPIO_PORTH_AHB_AFSEL_R  (*((volatile uint32_t *)0x4005F420))
#define GPIO_PORTH_AHB_DR2R_R   (*((volatile uint32_t *)0x4005F500))
#define GPIO_PORTH_AHB_DR4R_R   (*((volatile uint32_t *)0x4005F504))
#define GPIO_PORTH_AHB_DR8R_R   (*((volatile uint32_t *)0x4005F508))
#define GPIO_PORTH_AHB_ODR_R    (*((volatile uint32_t *)0x4005F50C))
#define GPIO_PORTH_AHB_PUR_R    (*((volatile uint32_t *)0x4005F510))
#define GPIO_PORTH_AHB_PDR_R    (*((volatile uint32_t *)0x4005F514))
#define GPIO_PORTH_AHB_SLR_R    (*((volatile uint32_t *)0x4005F518))
#define GPIO_PORTH_AHB_DEN_R    (*((volatile uint32_t *)0x4005F51C))
#define GPIO_PORTH_AHB_LOCK_R   (*((volatile uint32_t *)0x4005F520))
#define GPIO_PORTH_AHB_CR_R     (*((volatile uint32_t *)0x4005F524))
#define GPIO_PORTH_AHB_AMSEL_R  (*((volatile uint32_t *)0x4005F528))
#define GPIO_PORTH_AHB_PCTL_R   (*((volatile uint32_t *)0x4005F52C))
#define GPIO_PORTH_AHB_ADCCTL_R (*((volatile uint32_t *)0x4005F530))
#define GPIO_PORTH_AHB_DMACTL_R (*((volatile uint32_t *)0x4005F534))
#define GPIO_PORTH_AHB_SI_R     (*((volatile uint32_t *)0x4005F538))
#define GPIO_PORTH_AHB_DR12R_R  (*((volatile uint32_t *)0x4005F53C))
#define GPIO_PORTH_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x4005F540))
#define GPIO_PORTH_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x4005F544))
#define GPIO_PORTH_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x4005F548))
#define GPIO_PORTH_AHB_PP_R     (*((volatile uint32_t *)0x4005FFC0))
#define GPIO_PORTH_AHB_PC_R     (*((volatile uint32_t *)0x4005FFC4))

//*****************************************************************************
//
// GPIO registers (PORTJ AHB)
//
//*****************************************************************************
#define GPIO_PORTJ_AHB_DATA_BITS_R                                            \
                                ((volatile uint32_t *)0x40060000)
#define GPIO_PORTJ_AHB_DATA_R   (*((volatile uint32_t *)0x400603FC))
#define GPIO_PORTJ_AHB_DIR_R    (*((volatile uint32_t *)0x40060400))
#define GPIO_PORTJ_AHB_IS_R     (*((volatile uint32_t *)0x40060404))
#define GPIO_PORTJ_AHB_IBE_R    (*((volatile uint32_t *)0x40060408))
#define GPIO_PORTJ_AHB_IEV_R    (*((volatile uint32_t *)0x4006040C))
#define GPIO_PORTJ_AHB_IM_R     (*((volatile uint32_t *)0x40060410))
#define GPIO_PORTJ_AHB_RIS_R    (*((volatile uint32_t *)0x40060414))
#define GPIO_PORTJ_AHB_MIS_R    (*((volatile uint32_t *)0x40060418))
#define GPIO_PORTJ_AHB_ICR_R    (*((volatile uint32_t *)0x4006041C))
#define GPIO_PORTJ_AHB_AFSEL_R  (*((volatile uint32_t *)0x40060420))
#define GPIO_PORTJ_AHB_DR2R_R   (*((volatile uint32_t *)0x40060500))
#define GPIO_PORTJ_AHB_DR4R_R   (*((volatile uint32_t *)0x40060504))
#define GPIO_PORTJ_AHB_DR8R_R   (*((volatile uint32_t *)0x40060508))
#define GPIO_PORTJ_AHB_ODR_R    (*((volatile uint32_t *)0x4006050C))
#define GPIO_PORTJ_AHB_PUR_R    (*((volatile uint32_t *)0x40060510))
#define GPIO_PORTJ_AHB_PDR_R    (*((volatile uint32_t *)0x40060514))
#define GPIO_PORTJ_AHB_SLR_R    (*((volatile uint32_t *)0x40060518))
#define GPIO_PORTJ_AHB_DEN_R    (*((volatile uint32_t *)0x4006051C))
#define GPIO_PORTJ_AHB_LOCK_R   (*((volatile uint32_t *)0x40060520))
#define GPIO_PORTJ_AHB_CR_R     (*((volatile uint32_t *)0x40060524))
#define GPIO_PORTJ_AHB_AMSEL_R  (*((volatile uint32_t *)0x40060528))
#define GPIO_PORTJ_AHB_PCTL_R   (*((volatile uint32_t *)0x4006052C))
#define GPIO_PORTJ_AHB_ADCCTL_R (*((volatile uint32_t *)0x40060530))
#define GPIO_PORTJ_AHB_DMACTL_R (*((volatile uint32_t *)0x40060534))
#define GPIO_PORTJ_AHB_SI_R     (*((volatile uint32_t *)0x40060538))
#define GPIO_PORTJ_AHB_DR12R_R  (*((volatile uint32_t *)0x4006053C))
#define GPIO_PORTJ_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x40060540))
#define GPIO_PORTJ_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x40060544))
#define GPIO_PORTJ_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x40060548))
#define GPIO_PORTJ_AHB_PP_R     (*((volatile uint32_t *)0x40060FC0))
#define GPIO_PORTJ_AHB_PC_R     (*((volatile uint32_t *)0x40060FC4))

//*****************************************************************************
//
// GPIO registers (PORTK)
//
//*****************************************************************************
#define GPIO_PORTK_DATA_BITS_R  ((volatile uint32_t *)0x40061000)
#define GPIO_PORTK_DATA_R       (*((volatile uint32_t *)0x400613FC))
#define GPIO_PORTK_DIR_R        (*((volatile uint32_t *)0x40061400))
#define GPIO_PORTK_IS_R         (*((volatile uint32_t *)0x40061404))
#define GPIO_PORTK_IBE_R        (*((volatile uint32_t *)0x40061408))
#define GPIO_PORTK_IEV_R        (*((volatile uint32_t *)0x4006140C))
#define GPIO_PORTK_IM_R         (*((volatile uint32_t *)0x40061410))
#define GPIO_PORTK_RIS_R        (*((volatile uint32_t *)0x40061414))
#define GPIO_PORTK_MIS_R        (*((volatile uint32_t *)0x40061418))
#define GPIO_PORTK_ICR_R        (*((volatile uint32_t *)0x4006141C))
#define GPIO_PORTK_AFSEL_R      (*((volatile uint32_t *)0x40061420))
#define GPIO_PORTK_DR2R_R       (*((volatile uint32_t *)0x40061500))
#define GPIO_PORTK_DR4R_R       (*((volatile uint32_t *)0x40061504))
#define GPIO_PORTK_DR8R_R       (*((volatile uint32_t *)0x40061508))
#define GPIO_PORTK_ODR_R        (*((volatile uint32_t *)0x4006150C))
#define GPIO_PORTK_PUR_R        (*((volatile uint32_t *)0x40061510))
#define GPIO_PORTK_PDR_R        (*((volatile uint32_t *)0x40061514))
#define GPIO_PORTK_SLR_R        (*((volatile uint32_t *)0x40061518))
#define GPIO_PORTK_DEN_R        (*((volatile uint32_t *)0x4006151C))
#define GPIO_PORTK_LOCK_R       (*((volatile uint32_t *)0x40061520))
#define GPIO_PORTK_CR_R         (*((volatile uint32_t *)0x40061524))
#define GPIO_PORTK_AMSEL_R      (*((volatile uint32_t *)0x40061528))
#define GPIO_PORTK_PCTL_R       (*((volatile uint32_t *)0x4006152C))
#define GPIO_PORTK_ADCCTL_R     (*((volatile uint32_t *)0x40061530))
#define GPIO_PORTK_DMACTL_R     (*((volatile uint32_t *)0x40061534))
#define GPIO_PORTK_SI_R         (*((volatile uint32_t *)0x40061538))
#define GPIO_PORTK_DR12R_R      (*((volatile uint32_t *)0x4006153C))
#define GPIO_PORTK_WAKEPEN_R    (*((volatile uint32_t *)0x40061540))
#define GPIO_PORTK_WAKELVL_R    (*((volatile uint32_t *)0x40061544))
#define GPIO_PORTK_WAKESTAT_R   (*((volatile uint32_t *)0x40061548))
#define GPIO_PORTK_PP_R         (*((volatile uint32_t *)0x40061FC0))
#define GPIO_PORTK_PC_R         (*((volatile uint32_t *)0x40061FC4))

//*****************************************************************************
//
// GPIO registers (PORTL)
//
//*****************************************************************************
#define GPIO_PORTL_DATA_BITS_R  ((volatile uint32_t *)0x40062000)
#define GPIO_PORTL_DATA_R       (*((volatile uint32_t *)0x400623FC))
#define GPIO_PORTL_DIR_R        (*((volatile uint32_t *)0x40062400))
#define GPIO_PORTL_IS_R         (*((volatile uint32_t *)0x40062404))
#define GPIO_PORTL_IBE_R        (*((volatile uint32_t *)0x40062408))
#define GPIO_PORTL_IEV_R        (*((volatile uint32_t *)0x4006240C))
#define GPIO_PORTL_IM_R         (*((volatile uint32_t *)0x40062410))
#define GPIO_PORTL_RIS_R        (*((volatile uint32_t *)0x40062414))
#define GPIO_PORTL_MIS_R        (*((volatile uint32_t *)0x40062418))
#define GPIO_PORTL_ICR_R        (*((volatile uint32_t *)0x4006241C))
#define GPIO_PORTL_AFSEL_R      (*((volatile uint32_t *)0x40062420))
#define GPIO_PORTL_DR2R_R       (*((volatile uint32_t *)0x40062500))
#define GPIO_PORTL_DR4R_R       (*((volatile uint32_t *)0x40062504))
#define GPIO_PORTL_DR8R_R       (*((volatile uint32_t *)0x40062508))
#define GPIO_PORTL_ODR_R        (*((volatile uint32_t *)0x4006250C))
#define GPIO_PORTL_PUR_R        (*((volatile uint32_t *)0x40062510))
#define GPIO_PORTL_PDR_R        (*((volatile uint32_t *)0x40062514))
#define GPIO_PORTL_SLR_R        (*((volatile uint32_t *)0x40062518))
#define GPIO_PORTL_DEN_R        (*((volatile uint32_t *)0x4006251C))
#define GPIO_PORTL_LOCK_R       (*((volatile uint32_t *)0x40062520))
#define GPIO_PORTL_CR_R         (*((volatile uint32_t *)0x40062524))
#define GPIO_PORTL_AMSEL_R      (*((volatile uint32_t *)0x40062528))
#define GPIO_PORTL_PCTL_R       (*((volatile uint32_t *)0x4006252C))
#define GPIO_PORTL_ADCCTL_R     (*((volatile uint32_t *)0x40062530))
#define GPIO_PORTL_DMACTL_R     (*((volatile uint32_t *)0x40062534))
#define GPIO_PORTL_SI_R         (*((volatile uint32_t *)0x40062538))
#define GPIO_PORTL_DR12R_R      (*((volatile uint32_t *)0x4006253C))
#define GPIO_PORTL_WAKEPEN_R    (*((volatile uint32_t *)0x40062540))
#define GPIO_PORTL_WAKELVL_R    (*((volatile uint32_t *)0x40062544))
#define GPIO_PORTL_WAKESTAT_R   (*((volatile uint32_t *)0x40062548))
#define GPIO_PORTL_PP_R         (*((volatile uint32_t *)0x40062FC0))
#define GPIO_PORTL_PC_R         (*((volatile uint32_t *)0x40062FC4))

//*****************************************************************************
//
// GPIO registers (PORTM)
//
//*****************************************************************************
#define GPIO_PORTM_DATA_BITS_R  ((volatile uint32_t *)0x40063000)
#define GPIO_PORTM_DATA_R       (*((volatile uint32_t *)0x400633FC))
#define GPIO_PORTM_DIR_R        (*((volatile uint32_t *)0x40063400))
#define GPIO_PORTM_IS_R         (*((volatile uint32_t *)0x40063404))
#define GPIO_PORTM_IBE_R        (*((volatile uint32_t *)0x40063408))
#define GPIO_PORTM_IEV_R        (*((volatile uint32_t *)0x4006340C))
#define GPIO_PORTM_IM_R         (*((volatile uint32_t *)0x40063410))
#define GPIO_PORTM_RIS_R        (*((volatile uint32_t *)0x40063414))
#define GPIO_PORTM_MIS_R        (*((volatile uint32_t *)0x40063418))
#define GPIO_PORTM_ICR_R        (*((volatile uint32_t *)0x4006341C))
#define GPIO_PORTM_AFSEL_R      (*((volatile uint32_t *)0x40063420))
#define GPIO_PORTM_DR2R_R       (*((volatile uint32_t *)0x40063500))
#define GPIO_PORTM_DR4R_R       (*((volatile uint32_t *)0x40063504))
#define GPIO_PORTM_DR8R_R       (*((volatile uint32_t *)0x40063508))
#define GPIO_PORTM_ODR_R        (*((volatile uint32_t *)0x4006350C))
#define GPIO_PORTM_PUR_R        (*((volatile uint32_t *)0x40063510))
#define GPIO_PORTM_PDR_R        (*((volatile uint32_t *)0x40063514))
#define GPIO_PORTM_SLR_R        (*((volatile uint32_t *)0x40063518))
#define GPIO_PORTM_DEN_R        (*((volatile uint32_t *)0x4006351C))
#define GPIO_PORTM_LOCK_R       (*((volatile uint32_t *)0x40063520))
#define GPIO_PORTM_CR_R         (*((volatile uint32_t *)0x40063524))
#define GPIO_PORTM_AMSEL_R      (*((volatile uint32_t *)0x40063528))
#define GPIO_PORTM_PCTL_R       (*((volatile uint32_t *)0x4006352C))
#define GPIO_PORTM_ADCCTL_R     (*((volatile uint32_t *)0x40063530))
#define GPIO_PORTM_DMACTL_R     (*((volatile uint32_t *)0x40063534))
#define GPIO_PORTM_SI_R         (*((volatile uint32_t *)0x40063538))
#define GPIO_PORTM_DR12R_R      (*((volatile uint32_t *)0x4006353C))
#define GPIO_PORTM_WAKEPEN_R    (*((volatile uint32_t *)0x40063540))
#define GPIO_PORTM_WAKELVL_R    (*((volatile uint32_t *)0x40063544))
#define GPIO_PORTM_WAKESTAT_R   (*((volatile uint32_t *)0x40063548))
#define GPIO_PORTM_PP_R         (*((volatile uint32_t *)0x40063FC0))
#define GPIO_PORTM_PC_R         (*((volatile uint32_t *)0x40063FC4))

//*****************************************************************************
//
// GPIO registers (PORTN)
//
//*****************************************************************************
#define GPIO_PORTN_DATA_BITS_R  ((volatile uint32_t *)0x40064000)
#define GPIO_PORTN_DATA_R       (*((volatile uint32_t *)0x400643FC))
#define GPIO_PORTN_DIR_R        (*((volatile uint32_t *)0x40064400))
#define GPIO_PORTN_IS_R         (*((volatile uint32_t *)0x40064404))
#define GPIO_PORTN_IBE_R        (*((volatile uint32_t *)0x40064408))
#define GPIO_PORTN_IEV_R        (*((volatile uint32_t *)0x4006440C))
#define GPIO_PORTN_IM_R         (*((volatile uint32_t *)0x40064410))
#define GPIO_PORTN_RIS_R        (*((volatile uint32_t *)0x40064414))
#define GPIO_PORTN_MIS_R        (*((volatile uint32_t *)0x40064418))
#define GPIO_PORTN_ICR_R        (*((volatile uint32_t *)0x4006441C))
#define GPIO_PORTN_AFSEL_R      (*((volatile uint32_t *)0x40064420))
#define GPIO_PORTN_DR2R_R       (*((volatile uint32_t *)0x40064500))
#define GPIO_PORTN_DR4R_R       (*((volatile uint32_t *)0x40064504))
#define GPIO_PORTN_DR8R_R       (*((volatile uint32_t *)0x40064508))
#define GPIO_PORTN_ODR_R        (*((volatile uint32_t *)0x4006450C))
#define GPIO_PORTN_PUR_R        (*((volatile uint32_t *)0x40064510))
#define GPIO_PORTN_PDR_R        (*((volatile uint32_t *)0x40064514))
#define GPIO_PORTN_SLR_R        (*((volatile uint32_t *)0x40064518))
#define GPIO_PORTN_DEN_R        (*((volatile uint32_t *)0x4006451C))
#define GPIO_PORTN_LOCK_R       (*((volatile uint32_t *)0x40064520))
#define GPIO_PORTN_CR_R         (*((volatile uint32_t *)0x40064524))
#define GPIO_PORTN_AMSEL_R      (*((volatile uint32_t *)0x40064528))
#define GPIO_PORTN_PCTL_R       (*((volatile uint32_t *)0x4006452C))
#define GPIO_PORTN_ADCCTL_R     (*((volatile uint32_t *)0x40064530))
#define GPIO_PORTN_DMACTL_R     (*((volatile uint32_t *)0x40064534))
#define GPIO_PORTN_SI_R         (*((volatile uint32_t *)0x40064538))
#define GPIO_PORTN_DR12R_R      (*((volatile uint32_t *)0x4006453C))
#define GPIO_PORTN_WAKEPEN_R    (*((volatile uint32_t *)0x40064540))
#define GPIO_PORTN_WAKELVL_R    (*((volatile uint32_t *)0x40064544))
#define GPIO_PORTN_WAKESTAT_R   (*((volatile uint32_t *)0x40064548))
#define GPIO_PORTN_PP_R         (*((volatile uint32_t *)0x40064FC0))
#define GPIO_PORTN_PC_R         (*((volatile uint32_t *)0x40064FC4))

//*****************************************************************************
//
// GPIO registers (PORTP)
//
//*****************************************************************************
#define GPIO_PORTP_DATA_BITS_R  ((volatile uint32_t *)0x40065000)
#define GPIO_PORTP_DATA_R       (*((volatile uint32_t *)0x400653FC))
#define GPIO_PORTP_DIR_R        (*((volatile uint32_t *)0x40065400))
#define GPIO_PORTP_IS_R         (*((volatile uint32_t *)0x40065404))
#define GPIO_PORTP_IBE_R        (*((volatile uint32_t *)0x40065408))
#define GPIO_PORTP_IEV_R        (*((volatile uint32_t *)0x4006540C))
#define GPIO_PORTP_IM_R         (*((volatile uint32_t *)0x40065410))
#define GPIO_PORTP_RIS_R        (*((volatile uint32_t *)0x40065414))
#define GPIO_PORTP_MIS_R        (*((volatile uint32_t *)0x40065418))
#define GPIO_PORTP_ICR_R        (*((volatile uint32_t *)0x4006541C))
#define GPIO_PORTP_AFSEL_R      (*((volatile uint32_t *)0x40065420))
#define GPIO_PORTP_DR2R_R       (*((volatile uint32_t *)0x40065500))
#define GPIO_PORTP_DR4R_R       (*((volatile uint32_t *)0x40065504))
#define GPIO_PORTP_DR8R_R       (*((volatile uint32_t *)0x40065508))
#define GPIO_PORTP_ODR_R        (*((volatile uint32_t *)0x4006550C))
#define GPIO_PORTP_PUR_R        (*((volatile uint32_t *)0x40065510))
#define GPIO_PORTP_PDR_R        (*((volatile uint32_t *)0x40065514))
#define GPIO_PORTP_SLR_R        (*((volatile uint32_t *)0x40065518))
#define GPIO_PORTP_DEN_R        (*((volatile uint32_t *)0x4006551C))
#define GPIO_PORTP_LOCK_R       (*((volatile uint32_t *)0x40065520))
#define GPIO_PORTP_CR_R         (*((volatile uint32_t *)0x40065524))
#define GPIO_PORTP_AMSEL_R      (*((volatile uint32_t *)0x40065528))
#define GPIO_PORTP_PCTL_R       (*((volatile uint32_t *)0x4006552C))
#define GPIO_PORTP_ADCCTL_R     (*((volatile uint32_t *)0x40065530))
#define GPIO_PORTP_DMACTL_R     (*((volatile uint32_t *)0x40065534))
#define GPIO_PORTP_SI_R         (*((volatile uint32_t *)0x40065538))
#define GPIO_PORTP_DR12R_R      (*((volatile uint32_t *)0x4006553C))
#define GPIO_PORTP_WAKEPEN_R    (*((volatile uint32_t *)0x40065540))
#define GPIO_PORTP_WAKELVL_R    (*((volatile uint32_t *)0x40065544))
#define GPIO_PORTP_WAKESTAT_R   (*((volatile uint32_t *)0x40065548))
#define GPIO_PORTP_PP_R         (*((volatile uint32_t *)0x40065FC0))
#define GPIO_PORTP_PC_R         (*((volatile uint32_t *)0x40065FC4))

//*****************************************************************************
//
// GPIO registers (PORTQ)
//
//*****************************************************************************
#define GPIO_PORTQ_DATA_BITS_R  ((volatile uint32_t *)0x40066000)
#define GPIO_PORTQ_DATA_R       (*((volatile uint32_t *)0x400663FC))
#define GPIO_PORTQ_DIR_R        (*((volatile uint32_t *)0x40066400))
#define GPIO_PORTQ_IS_R         (*((volatile uint32_t *)0x40066404))
#define GPIO_PORTQ_IBE_R        (*((volatile uint32_t *)0x40066408))
#define GPIO_PORTQ_IEV_R        (*((volatile uint32_t *)0x4006640C))
#define GPIO_PORTQ_IM_R         (*((volatile uint32_t *)0x40066410))
#define GPIO_PORTQ_RIS_R        (*((volatile uint32_t *)0x40066414))
#define GPIO_PORTQ_MIS_R        (*((volatile uint32_t *)0x40066418))
#define GPIO_PORTQ_ICR_R        (*((volatile uint32_t *)0x4006641C))
#define GPIO_PORTQ_AFSEL_R      (*((volatile uint32_t *)0x40066420))
#define GPIO_PORTQ_DR2R_R       (*((volatile uint32_t *)0x40066500))
#define GPIO_PORTQ_DR4R_R       (*((volatile uint32_t *)0x40066504))
#define GPIO_PORTQ_DR8R_R       (*((volatile uint32_t *)0x40066508))
#define GPIO_PORTQ_ODR_R        (*((volatile uint32_t *)0x4006650C))
#define GPIO_PORTQ_PUR_R        (*((volatile uint32_t *)0x40066510))
#define GPIO_PORTQ_PDR_R        (*((volatile uint32_t *)0x40066514))
#define GPIO_PORTQ_SLR_R        (*((volatile uint32_t *)0x40066518))
#define GPIO_PORTQ_DEN_R        (*((volatile uint32_t *)0x4006651C))
#define GPIO_PORTQ_LOCK_R       (*((volatile uint32_t *)0x40066520))
#define GPIO_PORTQ_CR_R         (*((volatile uint32_t *)0x40066524))
#define GPIO_PORTQ_AMSEL_R      (*((volatile uint32_t *)0x40066528))
#define GPIO_PORTQ_PCTL_R       (*((volatile uint32_t *)0x4006652C))
#define GPIO_PORTQ_ADCCTL_R     (*((volatile uint32_t *)0x40066530))
#define GPIO_PORTQ_DMACTL_R     (*((volatile uint32_t *)0x40066534))
#define GPIO_PORTQ_SI_R         (*((volatile uint32_t *)0x40066538))
#define GPIO_PORTQ_DR12R_R      (*((volatile uint32_t *)0x4006653C))
#define GPIO_PORTQ_WAKEPEN_R    (*((volatile uint32_t *)0x40066540))
#define GPIO_PORTQ_WAKELVL_R    (*((volatile uint32_t *)0x40066544))
#define GPIO_PORTQ_WAKESTAT_R   (*((volatile uint32_t *)0x40066548))
#define GPIO_PORTQ_PP_R         (*((volatile uint32_t *)0x40066FC0))
#define GPIO_PORTQ_PC_R         (*((volatile uint32_t *)0x40066FC4))

//*****************************************************************************
//
// EEPROM registers (EEPROM)
//
//*****************************************************************************
#define EEPROM_EESIZE_R         (*((volatile uint32_t *)0x400AF000))
#define EEPROM_EEBLOCK_R        (*((volatile uint32_t *)0x400AF004))
#define EEPROM_EEOFFSET_R       (*((volatile uint32_t *)0x400AF008))
#define EEPROM_EERDWR_R         (*((volatile uint32_t *)0x400AF010))
#define EEPROM_EERDWRINC_R      (*((volatile uint32_t *)0x400AF014))
#define EEPROM_EEDONE_R         (*((volatile uint32_t *)0x400AF018))
#define EEPROM_EESUPP_R         (*((volatile uint32_t *)0x400AF01C))
#define EEPROM_EEUNLOCK_R       (*((volatile uint32_t *)0x400AF020))
#define EEPROM_EEPROT_R         (*((volatile uint32_t *)0x400AF030))
#define EEPROM_EEPASS0_R        (*((volatile uint32_t *)0x400AF034))
#define EEPROM_EEPASS1_R        (*((volatile uint32_t *)0x400AF038))
#define EEPROM_EEPASS2_R        (*((volatile uint32_t *)0x400AF03C))
#define EEPROM_EEINT_R          (*((volatile uint32_t *)0x400AF040))
#define EEPROM_EEHIDE0_R        (*((volatile uint32_t *)0x400AF050))
#define EEPROM_EEHIDE1_R        (*((volatile uint32_t *)0x400AF054))
#define EEPROM_EEHIDE2_R        (*((volatile uint32_t *)0x400AF058))
#define EEPROM_EEDBGME_R        (*((volatile uint32_t *)0x400AF080))
#define EEPROM_PP_R             (*((volatile uint32_t *)0x400AFFC0))

//*****************************************************************************
//
// I2C registers (I2C8)
//
//*****************************************************************************
#define I2C8_MSA_R              (*((volatile uint32_t *)0x400B8000))
#define I2C8_MCS_R              (*((volatile uint32_t *)0x400B8004))
#define I2C8_MDR_R              (*((volatile uint32_t *)0x400B8008))
#define I2C8_MTPR_R             (*((volatile uint32_t *)0x400B800C))
#define I2C8_MIMR_R             (*((volatile uint32_t *)0x400B8010))
#define I2C8_MRIS_R             (*((volatile uint32_t *)0x400B8014))
#define I2C8_MMIS_R             (*((volatile uint32_t *)0x400B8018))
#define I2C8_MICR_R             (*((volatile uint32_t *)0x400B801C))
#define I2C8_MCR_R              (*((volatile uint32_t *)0x400B8020))
#define I2C8_MCLKOCNT_R         (*((volatile uint32_t *)0x400B8024))
#define I2C8_MBMON_R            (*((volatile uint32_t *)0x400B802C))
#define I2C8_MBLEN_R            (*((volatile uint32_t *)0x400B8030))
#define I2C8_MBCNT_R            (*((volatile uint32_t *)0x400B8034))
#define I2C8_SOAR_R             (*((volatile uint32_t *)0x400B8800))
#define I2C8_SCSR_R             (*((volatile uint32_t *)0x400B8804))
#define I2C8_SDR_R              (*((volatile uint32_t *)0x400B8808))
#define I2C8_SIMR_R             (*((volatile uint32_t *)0x400B880C))
#define I2C8_SRIS_R             (*((volatile uint32_t *)0x400B8810))
#define I2C8_SMIS_R             (*((volatile uint32_t *)0x400B8814))
#define I2C8_SICR_R             (*((volatile uint32_t *)0x400B8818))
#define I2C8_SOAR2_R            (*((volatile uint32_t *)0x400B881C))
#define I2C8_SACKCTL_R          (*((volatile uint32_t *)0x400B8820))
#define I2C8_FIFODATA_R         (*((volatile uint32_t *)0x400B8F00))
#define I2C8_FIFOCTL_R          (*((volatile uint32_t *)0x400B8F04))
#define I2C8_FIFOSTATUS_R       (*((volatile uint32_t *)0x400B8F08))
#define I2C8_PP_R               (*((volatile uint32_t *)0x400B8FC0))
#define I2C8_PC_R               (*((volatile uint32_t *)0x400B8FC4))

//*****************************************************************************
//
// I2C registers (I2C9)
//
//*****************************************************************************
#define I2C9_MSA_R              (*((volatile uint32_t *)0x400B9000))
#define I2C9_MCS_R              (*((volatile uint32_t *)0x400B9004))
#define I2C9_MDR_R              (*((volatile uint32_t *)0x400B9008))
#define I2C9_MTPR_R             (*((volatile uint32_t *)0x400B900C))
#define I2C9_MIMR_R             (*((volatile uint32_t *)0x400B9010))
#define I2C9_MRIS_R             (*((volatile uint32_t *)0x400B9014))
#define I2C9_MMIS_R             (*((volatile uint32_t *)0x400B9018))
#define I2C9_MICR_R             (*((volatile uint32_t *)0x400B901C))
#define I2C9_MCR_R              (*((volatile uint32_t *)0x400B9020))
#define I2C9_MCLKOCNT_R         (*((volatile uint32_t *)0x400B9024))
#define I2C9_MBMON_R            (*((volatile uint32_t *)0x400B902C))
#define I2C9_MBLEN_R            (*((volatile uint32_t *)0x400B9030))
#define I2C9_MBCNT_R            (*((volatile uint32_t *)0x400B9034))
#define I2C9_SOAR_R             (*((volatile uint32_t *)0x400B9800))
#define I2C9_SCSR_R             (*((volatile uint32_t *)0x400B9804))
#define I2C9_SDR_R              (*((volatile uint32_t *)0x400B9808))
#define I2C9_SIMR_R             (*((volatile uint32_t *)0x400B980C))
#define I2C9_SRIS_R             (*((volatile uint32_t *)0x400B9810))
#define I2C9_SMIS_R             (*((volatile uint32_t *)0x400B9814))
#define I2C9_SICR_R             (*((volatile uint32_t *)0x400B9818))
#define I2C9_SOAR2_R            (*((volatile uint32_t *)0x400B981C))
#define I2C9_SACKCTL_R          (*((volatile uint32_t *)0x400B9820))
#define I2C9_FIFODATA_R         (*((volatile uint32_t *)0x400B9F00))
#define I2C9_FIFOCTL_R          (*((volatile uint32_t *)0x400B9F04))
#define I2C9_FIFOSTATUS_R       (*((volatile uint32_t *)0x400B9F08))
#define I2C9_PP_R               (*((volatile uint32_t *)0x400B9FC0))
#define I2C9_PC_R               (*((volatile uint32_t *)0x400B9FC4))

//*****************************************************************************
//
// I2C registers (I2C4)
//
//*****************************************************************************
#define I2C4_MSA_R              (*((volatile uint32_t *)0x400C0000))
#define I2C4_MCS_R              (*((volatile uint32_t *)0x400C0004))
#define I2C4_MDR_R              (*((volatile uint32_t *)0x400C0008))
#define I2C4_MTPR_R             (*((volatile uint32_t *)0x400C000C))
#define I2C4_MIMR_R             (*((volatile uint32_t *)0x400C0010))
#define I2C4_MRIS_R             (*((volatile uint32_t *)0x400C0014))
#define I2C4_MMIS_R             (*((volatile uint32_t *)0x400C0018))
#define I2C4_MICR_R             (*((volatile uint32_t *)0x400C001C))
#define I2C4_MCR_R              (*((volatile uint32_t *)0x400C0020))
#define I2C4_MCLKOCNT_R         (*((volatile uint32_t *)0x400C0024))
#define I2C4_MBMON_R            (*((volatile uint32_t *)0x400C002C))
#define I2C4_MBLEN_R            (*((volatile uint32_t *)0x400C0030))
#define I2C4_MBCNT_R            (*((volatile uint32_t *)0x400C0034))
#define I2C4_SOAR_R             (*((volatile uint32_t *)0x400C0800))
#define I2C4_SCSR_R             (*((volatile uint32_t *)0x400C0804))
#define I2C4_SDR_R              (*((volatile uint32_t *)0x400C0808))
#define I2C4_SIMR_R             (*((volatile uint32_t *)0x400C080C))
#define I2C4_SRIS_R             (*((volatile uint32_t *)0x400C0810))
#define I2C4_SMIS_R             (*((volatile uint32_t *)0x400C0814))
#define I2C4_SICR_R             (*((volatile uint32_t *)0x400C0818))
#define I2C4_SOAR2_R            (*((volatile uint32_t *)0x400C081C))
#define I2C4_SACKCTL_R          (*((volatile uint32_t *)0x400C0820))
#define I2C4_FIFODATA_R         (*((volatile uint32_t *)0x400C0F00))
#define I2C4_FIFOCTL_R          (*((volatile uint32_t *)0x400C0F04))
#define I2C4_FIFOSTATUS_R       (*((volatile uint32_t *)0x400C0F08))
#define I2C4_PP_R               (*((volatile uint32_t *)0x400C0FC0))
#define I2C4_PC_R               (*((volatile uint32_t *)0x400C0FC4))

//*****************************************************************************
//
// I2C registers (I2C5)
//
//*****************************************************************************
#define I2C5_MSA_R              (*((volatile uint32_t *)0x400C1000))
#define I2C5_MCS_R              (*((volatile uint32_t *)0x400C1004))
#define I2C5_MDR_R              (*((volatile uint32_t *)0x400C1008))
#define I2C5_MTPR_R             (*((volatile uint32_t *)0x400C100C))
#define I2C5_MIMR_R             (*((volatile uint32_t *)0x400C1010))
#define I2C5_MRIS_R             (*((volatile uint32_t *)0x400C1014))
#define I2C5_MMIS_R             (*((volatile uint32_t *)0x400C1018))
#define I2C5_MICR_R             (*((volatile uint32_t *)0x400C101C))
#define I2C5_MCR_R              (*((volatile uint32_t *)0x400C1020))
#define I2C5_MCLKOCNT_R         (*((volatile uint32_t *)0x400C1024))
#define I2C5_MBMON_R            (*((volatile uint32_t *)0x400C102C))
#define I2C5_MBLEN_R            (*((volatile uint32_t *)0x400C1030))
#define I2C5_MBCNT_R            (*((volatile uint32_t *)0x400C1034))
#define I2C5_SOAR_R             (*((volatile uint32_t *)0x400C1800))
#define I2C5_SCSR_R             (*((volatile uint32_t *)0x400C1804))
#define I2C5_SDR_R              (*((volatile uint32_t *)0x400C1808))
#define I2C5_SIMR_R             (*((volatile uint32_t *)0x400C180C))
#define I2C5_SRIS_R             (*((volatile uint32_t *)0x400C1810))
#define I2C5_SMIS_R             (*((volatile uint32_t *)0x400C1814))
#define I2C5_SICR_R             (*((volatile uint32_t *)0x400C1818))
#define I2C5_SOAR2_R            (*((volatile uint32_t *)0x400C181C))
#define I2C5_SACKCTL_R          (*((volatile uint32_t *)0x400C1820))
#define I2C5_FIFODATA_R         (*((volatile uint32_t *)0x400C1F00))
#define I2C5_FIFOCTL_R          (*((volatile uint32_t *)0x400C1F04))
#define I2C5_FIFOSTATUS_R       (*((volatile uint32_t *)0x400C1F08))
#define I2C5_PP_R               (*((volatile uint32_t *)0x400C1FC0))
#define I2C5_PC_R               (*((volatile uint32_t *)0x400C1FC4))

//*****************************************************************************
//
// I2C registers (I2C6)
//
//*****************************************************************************
#define I2C6_MSA_R              (*((volatile uint32_t *)0x400C2000))
#define I2C6_MCS_R              (*((volatile uint32_t *)0x400C2004))
#define I2C6_MDR_R              (*((volatile uint32_t *)0x400C2008))
#define I2C6_MTPR_R             (*((volatile uint32_t *)0x400C200C))
#define I2C6_MIMR_R             (*((volatile uint32_t *)0x400C2010))
#define I2C6_MRIS_R             (*((volatile uint32_t *)0x400C2014))
#define I2C6_MMIS_R             (*((volatile uint32_t *)0x400C2018))
#define I2C6_MICR_R             (*((volatile uint32_t *)0x400C201C))
#define I2C6_MCR_R              (*((volatile uint32_t *)0x400C2020))
#define I2C6_MCLKOCNT_R         (*((volatile uint32_t *)0x400C2024))
#define I2C6_MBMON_R            (*((volatile uint32_t *)0x400C202C))
#define I2C6_MBLEN_R            (*((volatile uint32_t *)0x400C2030))
#define I2C6_MBCNT_R            (*((volatile uint32_t *)0x400C2034))
#define I2C6_SOAR_R             (*((volatile uint32_t *)0x400C2800))
#define I2C6_SCSR_R             (*((volatile uint32_t *)0x400C2804))
#define I2C6_SDR_R              (*((volatile uint32_t *)0x400C2808))
#define I2C6_SIMR_R             (*((volatile uint32_t *)0x400C280C))
#define I2C6_SRIS_R             (*((volatile uint32_t *)0x400C2810))
#define I2C6_SMIS_R             (*((volatile uint32_t *)0x400C2814))
#define I2C6_SICR_R             (*((volatile uint32_t *)0x400C2818))
#define I2C6_SOAR2_R            (*((volatile uint32_t *)0x400C281C))
#define I2C6_SACKCTL_R          (*((volatile uint32_t *)0x400C2820))
#define I2C6_FIFODATA_R         (*((volatile uint32_t *)0x400C2F00))
#define I2C6_FIFOCTL_R          (*((volatile uint32_t *)0x400C2F04))
#define I2C6_FIFOSTATUS_R       (*((volatile uint32_t *)0x400C2F08))
#define I2C6_PP_R               (*((volatile uint32_t *)0x400C2FC0))
#define I2C6_PC_R               (*((volatile uint32_t *)0x400C2FC4))

//*****************************************************************************
//
// I2C registers (I2C7)
//
//*****************************************************************************
#define I2C7_MSA_R              (*((volatile uint32_t *)0x400C3000))
#define I2C7_MCS_R              (*((volatile uint32_t *)0x400C3004))
#define I2C7_MDR_R              (*((volatile uint32_t *)0x400C3008))
#define I2C7_MTPR_R             (*((volatile uint32_t *)0x400C300C))
#define I2C7_MIMR_R             (*((volatile uint32_t *)0x400C3010))
#define I2C7_MRIS_R             (*((volatile uint32_t *)0x400C3014))
#define I2C7_MMIS_R             (*((volatile uint32_t *)0x400C3018))
#define I2C7_MICR_R             (*((volatile uint32_t *)0x400C301C))
#define I2C7_MCR_R              (*((volatile uint32_t *)0x400C3020))
#define I2C7_MCLKOCNT_R         (*((volatile uint32_t *)0x400C3024))
#define I2C7_MBMON_R            (*((volatile uint32_t *)0x400C302C))
#define I2C7_MBLEN_R            (*((volatile uint32_t *)0x400C3030))
#define I2C7_MBCNT_R            (*((volatile uint32_t *)0x400C3034))
#define I2C7_SOAR_R             (*((volatile uint32_t *)0x400C3800))
#define I2C7_SCSR_R             (*((volatile uint32_t *)0x400C3804))
#define I2C7_SDR_R              (*((volatile uint32_t *)0x400C3808))
#define I2C7_SIMR_R             (*((volatile uint32_t *)0x400C380C))
#define I2C7_SRIS_R             (*((volatile uint32_t *)0x400C3810))
#define I2C7_SMIS_R             (*((volatile uint32_t *)0x400C3814))
#define I2C7_SICR_R             (*((volatile uint32_t *)0x400C3818))
#define I2C7_SOAR2_R            (*((volatile uint32_t *)0x400C381C))
#define I2C7_SACKCTL_R          (*((volatile uint32_t *)0x400C3820))
#define I2C7_FIFODATA_R         (*((volatile uint32_t *)0x400C3F00))
#define I2C7_FIFOCTL_R          (*((volatile uint32_t *)0x400C3F04))
#define I2C7_FIFOSTATUS_R       (*((volatile uint32_t *)0x400C3F08))
#define I2C7_PP_R               (*((volatile uint32_t *)0x400C3FC0))
#define I2C7_PC_R               (*((volatile uint32_t *)0x400C3FC4))

//*****************************************************************************
//
// External Peripheral Interface registers (EPI0)
//
//*****************************************************************************
#define EPI0_CFG_R              (*((volatile uint32_t *)0x400D0000))
#define EPI0_BAUD_R             (*((volatile uint32_t *)0x400D0004))
#define EPI0_BAUD2_R            (*((volatile uint32_t *)0x400D0008))
#define EPI0_HB16CFG_R          (*((volatile uint32_t *)0x400D0010))
#define EPI0_GPCFG_R            (*((volatile uint32_t *)0x400D0010))
#define EPI0_SDRAMCFG_R         (*((volatile uint32_t *)0x400D0010))
#define EPI0_HB8CFG_R           (*((volatile uint32_t *)0x400D0010))
#define EPI0_HB8CFG2_R          (*((volatile uint32_t *)0x400D0014))
#define EPI0_HB16CFG2_R         (*((volatile uint32_t *)0x400D0014))
#define EPI0_ADDRMAP_R          (*((volatile uint32_t *)0x400D001C))
#define EPI0_RSIZE0_R           (*((volatile uint32_t *)0x400D0020))
#define EPI0_RADDR0_R           (*((volatile uint32_t *)0x400D0024))
#define EPI0_RPSTD0_R           (*((volatile uint32_t *)0x400D0028))
#define EPI0_RSIZE1_R           (*((volatile uint32_t *)0x400D0030))
#define EPI0_RADDR1_R           (*((volatile uint32_t *)0x400D0034))
#define EPI0_RPSTD1_R           (*((volatile uint32_t *)0x400D0038))
#define EPI0_STAT_R             (*((volatile uint32_t *)0x400D0060))
#define EPI0_RFIFOCNT_R         (*((volatile uint32_t *)0x400D006C))
#define EPI0_READFIFO0_R        (*((volatile uint32_t *)0x400D0070))
#define EPI0_READFIFO1_R        (*((volatile uint32_t *)0x400D0074))
#define EPI0_READFIFO2_R        (*((volatile uint32_t *)0x400D0078))
#define EPI0_READFIFO3_R        (*((volatile uint32_t *)0x400D007C))
#define EPI0_READFIFO4_R        (*((volatile uint32_t *)0x400D0080))
#define EPI0_READFIFO5_R        (*((volatile uint32_t *)0x400D0084))
#define EPI0_READFIFO6_R        (*((volatile uint32_t *)0x400D0088))
#define EPI0_READFIFO7_R        (*((volatile uint32_t *)0x400D008C))
#define EPI0_FIFOLVL_R          (*((volatile uint32_t *)0x400D0200))
#define EPI0_WFIFOCNT_R         (*((volatile uint32_t *)0x400D0204))
#define EPI0_DMATXCNT_R         (*((volatile uint32_t *)0x400D0208))
#define EPI0_IM_R               (*((volatile uint32_t *)0x400D0210))
#define EPI0_RIS_R              (*((volatile uint32_t *)0x400D0214))
#define EPI0_MIS_R              (*((volatile uint32_t *)0x400D0218))
#define EPI0_EISC_R             (*((volatile uint32_t *)0x400D021C))
#define EPI0_HB8CFG3_R          (*((volatile uint32_t *)0x400D0308))
#define EPI0_HB16CFG3_R         (*((volatile uint32_t *)0x400D0308))
#define EPI0_HB16CFG4_R         (*((volatile uint32_t *)0x400D030C))
#define EPI0_HB8CFG4_R          (*((volatile uint32_t *)0x400D030C))
#define EPI0_HB8TIME_R          (*((volatile uint32_t *)0x400D0310))
#define EPI0_HB16TIME_R         (*((volatile uint32_t *)0x400D0310))
#define EPI0_HB8TIME2_R         (*((volatile uint32_t *)0x400D0314))
#define EPI0_HB16TIME2_R        (*((volatile uint32_t *)0x400D0314))
#define EPI0_HB16TIME3_R        (*((volatile uint32_t *)0x400D0318))
#define EPI0_HB8TIME3_R         (*((volatile uint32_t *)0x400D0318))
#define EPI0_HB8TIME4_R         (*((volatile uint32_t *)0x400D031C))
#define EPI0_HB16TIME4_R        (*((volatile uint32_t *)0x400D031C))
#define EPI0_HBPSRAM_R          (*((volatile uint32_t *)0x400D0360))

//*****************************************************************************
//
// Timer registers (TIMER6)
//
//*****************************************************************************
#define TIMER6_CFG_R            (*((volatile uint32_t *)0x400E0000))
#define TIMER6_TAMR_R           (*((volatile uint32_t *)0x400E0004))
#define TIMER6_TBMR_R           (*((volatile uint32_t *)0x400E0008))
#define TIMER6_CTL_R            (*((volatile uint32_t *)0x400E000C))
#define TIMER6_SYNC_R           (*((volatile uint32_t *)0x400E0010))
#define TIMER6_IMR_R            (*((volatile uint32_t *)0x400E0018))
#define TIMER6_RIS_R            (*((volatile uint32_t *)0x400E001C))
#define TIMER6_MIS_R            (*((volatile uint32_t *)0x400E0020))
#define TIMER6_ICR_R            (*((volatile uint32_t *)0x400E0024))
#define TIMER6_TAILR_R          (*((volatile uint32_t *)0x400E0028))
#define TIMER6_TBILR_R          (*((volatile uint32_t *)0x400E002C))
#define TIMER6_TAMATCHR_R       (*((volatile uint32_t *)0x400E0030))
#define TIMER6_TBMATCHR_R       (*((volatile uint32_t *)0x400E0034))
#define TIMER6_TAPR_R           (*((volatile uint32_t *)0x400E0038))
#define TIMER6_TBPR_R           (*((volatile uint32_t *)0x400E003C))
#define TIMER6_TAPMR_R          (*((volatile uint32_t *)0x400E0040))
#define TIMER6_TBPMR_R          (*((volatile uint32_t *)0x400E0044))
#define TIMER6_TAR_R            (*((volatile uint32_t *)0x400E0048))
#define TIMER6_TBR_R            (*((volatile uint32_t *)0x400E004C))
#define TIMER6_TAV_R            (*((volatile uint32_t *)0x400E0050))
#define TIMER6_TBV_R            (*((volatile uint32_t *)0x400E0054))
#define TIMER6_RTCPD_R          (*((volatile uint32_t *)0x400E0058))
#define TIMER6_TAPS_R           (*((volatile uint32_t *)0x400E005C))
#define TIMER6_TBPS_R           (*((volatile uint32_t *)0x400E0060))
#define TIMER6_DMAEV_R          (*((volatile uint32_t *)0x400E006C))
#define TIMER6_ADCEV_R          (*((volatile uint32_t *)0x400E0070))
#define TIMER6_PP_R             (*((volatile uint32_t *)0x400E0FC0))
#define TIMER6_CC_R             (*((volatile uint32_t *)0x400E0FC8))

//*****************************************************************************
//
// Timer registers (TIMER7)
//
//*****************************************************************************
#define TIMER7_CFG_R            (*((volatile uint32_t *)0x400E1000))
#define TIMER7_TAMR_R           (*((volatile uint32_t *)0x400E1004))
#define TIMER7_TBMR_R           (*((volatile uint32_t *)0x400E1008))
#define TIMER7_CTL_R            (*((volatile uint32_t *)0x400E100C))
#define TIMER7_SYNC_R           (*((volatile uint32_t *)0x400E1010))
#define TIMER7_IMR_R            (*((volatile uint32_t *)0x400E1018))
#define TIMER7_RIS_R            (*((volatile uint32_t *)0x400E101C))
#define TIMER7_MIS_R            (*((volatile uint32_t *)0x400E1020))
#define TIMER7_ICR_R            (*((volatile uint32_t *)0x400E1024))
#define TIMER7_TAILR_R          (*((volatile uint32_t *)0x400E1028))
#define TIMER7_TBILR_R          (*((volatile uint32_t *)0x400E102C))
#define TIMER7_TAMATCHR_R       (*((volatile uint32_t *)0x400E1030))
#define TIMER7_TBMATCHR_R       (*((volatile uint32_t *)0x400E1034))
#define TIMER7_TAPR_R           (*((volatile uint32_t *)0x400E1038))
#define TIMER7_TBPR_R           (*((volatile uint32_t *)0x400E103C))
#define TIMER7_TAPMR_R          (*((volatile uint32_t *)0x400E1040))
#define TIMER7_TBPMR_R          (*((volatile uint32_t *)0x400E1044))
#define TIMER7_TAR_R            (*((volatile uint32_t *)0x400E1048))
#define TIMER7_TBR_R            (*((volatile uint32_t *)0x400E104C))
#define TIMER7_TAV_R            (*((volatile uint32_t *)0x400E1050))
#define TIMER7_TBV_R            (*((volatile uint32_t *)0x400E1054))
#define TIMER7_RTCPD_R          (*((volatile uint32_t *)0x400E1058))
#define TIMER7_TAPS_R           (*((volatile uint32_t *)0x400E105C))
#define TIMER7_TBPS_R           (*((volatile uint32_t *)0x400E1060))
#define TIMER7_DMAEV_R          (*((volatile uint32_t *)0x400E106C))
#define TIMER7_ADCEV_R          (*((volatile uint32_t *)0x400E1070))
#define TIMER7_PP_R             (*((volatile uint32_t *)0x400E1FC0))
#define TIMER7_CC_R             (*((volatile uint32_t *)0x400E1FC8))

//*****************************************************************************
//
// EMAC registers (EMAC0)
//
//*****************************************************************************
#define EMAC0_CFG_R             (*((volatile uint32_t *)0x400EC000))
#define EMAC0_FRAMEFLTR_R       (*((volatile uint32_t *)0x400EC004))
#define EMAC0_HASHTBLH_R        (*((volatile uint32_t *)0x400EC008))
#define EMAC0_HASHTBLL_R        (*((volatile uint32_t *)0x400EC00C))
#define EMAC0_MIIADDR_R         (*((volatile uint32_t *)0x400EC010))
#define EMAC0_MIIDATA_R         (*((volatile uint32_t *)0x400EC014))
#define EMAC0_FLOWCTL_R         (*((volatile uint32_t *)0x400EC018))
#define EMAC0_VLANTG_R          (*((volatile uint32_t *)0x400EC01C))
#define EMAC0_STATUS_R          (*((volatile uint32_t *)0x400EC024))
#define EMAC0_RWUFF_R           (*((volatile uint32_t *)0x400EC028))
#define EMAC0_PMTCTLSTAT_R      (*((volatile uint32_t *)0x400EC02C))
#define EMAC0_RIS_R             (*((volatile uint32_t *)0x400EC038))
#define EMAC0_IM_R              (*((volatile uint32_t *)0x400EC03C))
#define EMAC0_ADDR0H_R          (*((volatile uint32_t *)0x400EC040))
#define EMAC0_ADDR0L_R          (*((volatile uint32_t *)0x400EC044))
#define EMAC0_ADDR1H_R          (*((volatile uint32_t *)0x400EC048))
#define EMAC0_ADDR1L_R          (*((volatile uint32_t *)0x400EC04C))
#define EMAC0_ADDR2H_R          (*((volatile uint32_t *)0x400EC050))
#define EMAC0_ADDR2L_R          (*((volatile uint32_t *)0x400EC054))
#define EMAC0_ADDR3H_R          (*((volatile uint32_t *)0x400EC058))
#define EMAC0_ADDR3L_R          (*((volatile uint32_t *)0x400EC05C))
#define EMAC0_WDOGTO_R          (*((volatile uint32_t *)0x400EC0DC))
#define EMAC0_MMCCTRL_R         (*((volatile uint32_t *)0x400EC100))
#define EMAC0_MMCRXRIS_R        (*((volatile uint32_t *)0x400EC104))
#define EMAC0_MMCTXRIS_R        (*((volatile uint32_t *)0x400EC108))
#define EMAC0_MMCRXIM_R         (*((volatile uint32_t *)0x400EC10C))
#define EMAC0_MMCTXIM_R         (*((volatile uint32_t *)0x400EC110))
#define EMAC0_TXCNTGB_R         (*((volatile uint32_t *)0x400EC118))
#define EMAC0_TXCNTSCOL_R       (*((volatile uint32_t *)0x400EC14C))
#define EMAC0_TXCNTMCOL_R       (*((volatile uint32_t *)0x400EC150))
#define EMAC0_TXOCTCNTG_R       (*((volatile uint32_t *)0x400EC164))
#define EMAC0_RXCNTGB_R         (*((volatile uint32_t *)0x400EC180))
#define EMAC0_RXCNTCRCERR_R     (*((volatile uint32_t *)0x400EC194))
#define EMAC0_RXCNTALGNERR_R    (*((volatile uint32_t *)0x400EC198))
#define EMAC0_RXCNTGUNI_R       (*((volatile uint32_t *)0x400EC1C4))
#define EMAC0_VLNINCREP_R       (*((volatile uint32_t *)0x400EC584))
#define EMAC0_VLANHASH_R        (*((volatile uint32_t *)0x400EC588))
#define EMAC0_TIMSTCTRL_R       (*((volatile uint32_t *)0x400EC700))
#define EMAC0_SUBSECINC_R       (*((volatile uint32_t *)0x400EC704))
#define EMAC0_TIMSEC_R          (*((volatile uint32_t *)0x400EC708))
#define EMAC0_TIMNANO_R         (*((volatile uint32_t *)0x400EC70C))
#define EMAC0_TIMSECU_R         (*((volatile uint32_t *)0x400EC710))
#define EMAC0_TIMNANOU_R        (*((volatile uint32_t *)0x400EC714))
#define EMAC0_TIMADD_R          (*((volatile uint32_t *)0x400EC718))
#define EMAC0_TARGSEC_R         (*((volatile uint32_t *)0x400EC71C))
#define EMAC0_TARGNANO_R        (*((volatile uint32_t *)0x400EC720))
#define EMAC0_HWORDSEC_R        (*((volatile uint32_t *)0x400EC724))
#define EMAC0_TIMSTAT_R         (*((volatile uint32_t *)0x400EC728))
#define EMAC0_PPSCTRL_R         (*((volatile uint32_t *)0x400EC72C))
#define EMAC0_PPS0INTVL_R       (*((volatile uint32_t *)0x400EC760))
#define EMAC0_PPS0WIDTH_R       (*((volatile uint32_t *)0x400EC764))
#define EMAC0_DMABUSMOD_R       (*((volatile uint32_t *)0x400ECC00))
#define EMAC0_TXPOLLD_R         (*((volatile uint32_t *)0x400ECC04))
#define EMAC0_RXPOLLD_R         (*((volatile uint32_t *)0x400ECC08))
#define EMAC0_RXDLADDR_R        (*((volatile uint32_t *)0x400ECC0C))
#define EMAC0_TXDLADDR_R        (*((volatile uint32_t *)0x400ECC10))
#define EMAC0_DMARIS_R          (*((volatile uint32_t *)0x400ECC14))
#define EMAC0_DMAOPMODE_R       (*((volatile uint32_t *)0x400ECC18))
#define EMAC0_DMAIM_R           (*((volatile uint32_t *)0x400ECC1C))
#define EMAC0_MFBOC_R           (*((volatile uint32_t *)0x400ECC20))
#define EMAC0_RXINTWDT_R        (*((volatile uint32_t *)0x400ECC24))
#define EMAC0_HOSTXDESC_R       (*((volatile uint32_t *)0x400ECC48))
#define EMAC0_HOSRXDESC_R       (*((volatile uint32_t *)0x400ECC4C))
#define EMAC0_HOSTXBA_R         (*((volatile uint32_t *)0x400ECC50))
#define EMAC0_HOSRXBA_R         (*((volatile uint32_t *)0x400ECC54))
#define EMAC0_PP_R              (*((volatile uint32_t *)0x400ECFC0))
#define EMAC0_PC_R              (*((volatile uint32_t *)0x400ECFC4))
#define EMAC0_CC_R              (*((volatile uint32_t *)0x400ECFC8))
#define EMAC0_EPHYRIS_R         (*((volatile uint32_t *)0x400ECFD0))
#define EMAC0_EPHYIM_R          (*((volatile uint32_t *)0x400ECFD4))
#define EMAC0_EPHYMISC_R        (*((volatile uint32_t *)0x400ECFD8))

//*****************************************************************************
//
// EPHY registers (EMAC0)
//
//*****************************************************************************
#define EPHY_BMCR               0x00000000  // Ethernet PHY Basic Mode Control
#define EPHY_BMSR               0x00000001  // Ethernet PHY Basic Mode Status
#define EPHY_ID1                0x00000002  // Ethernet PHY Identifier Register
                                            // 1
#define EPHY_ID2                0x00000003  // Ethernet PHY Identifier Register
                                            // 2
#define EPHY_ANA                0x00000004  // Ethernet PHY Auto-Negotiation
                                            // Advertisement
#define EPHY_ANLPA              0x00000005  // Ethernet PHY Auto-Negotiation
                                            // Link Partner Ability
#define EPHY_ANER               0x00000006  // Ethernet PHY Auto-Negotiation
                                            // Expansion
#define EPHY_ANNPTR             0x00000007  // Ethernet PHY Auto-Negotiation
                                            // Next Page TX
#define EPHY_ANLNPTR            0x00000008  // Ethernet PHY Auto-Negotiation
                                            // Link Partner Ability Next Page
#define EPHY_CFG1               0x00000009  // Ethernet PHY Configuration 1
#define EPHY_CFG2               0x0000000A  // Ethernet PHY Configuration 2
#define EPHY_CFG3               0x0000000B  // Ethernet PHY Configuration 3
#define EPHY_REGCTL             0x0000000D  // Ethernet PHY Register Control
#define EPHY_ADDAR              0x0000000E  // Ethernet PHY Address or Data
#define EPHY_STS                0x00000010  // Ethernet PHY Status
#define EPHY_SCR                0x00000011  // Ethernet PHY Specific Control
#define EPHY_MISR1              0x00000012  // Ethernet PHY MII Interrupt
                                            // Status 1
#define EPHY_MISR2              0x00000013  // Ethernet PHY MII Interrupt
                                            // Status 2
#define EPHY_FCSCR              0x00000014  // Ethernet PHY False Carrier Sense
                                            // Counter
#define EPHY_RXERCNT            0x00000015  // Ethernet PHY Receive Error Count
#define EPHY_BISTCR             0x00000016  // Ethernet PHY BIST Control
#define EPHY_LEDCR              0x00000018  // Ethernet PHY LED Control
#define EPHY_CTL                0x00000019  // Ethernet PHY Control
#define EPHY_10BTSC             0x0000001A  // Ethernet PHY 10Base-T
                                            // Status/Control - MR26
#define EPHY_BICSR1             0x0000001B  // Ethernet PHY BIST Control and
                                            // Status 1
#define EPHY_BICSR2             0x0000001C  // Ethernet PHY BIST Control and
                                            // Status 2
#define EPHY_CDCR               0x0000001E  // Ethernet PHY Cable Diagnostic
                                            // Control
#define EPHY_RCR                0x0000001F  // Ethernet PHY Reset Control
#define EPHY_LEDCFG             0x00000025  // Ethernet PHY LED Configuration

//*****************************************************************************
//
// System Exception Module registers (SYSEXC)
//
//*****************************************************************************
#define SYSEXC_RIS_R            (*((volatile uint32_t *)0x400F9000))
#define SYSEXC_IM_R             (*((volatile uint32_t *)0x400F9004))
#define SYSEXC_MIS_R            (*((volatile uint32_t *)0x400F9008))
#define SYSEXC_IC_R             (*((volatile uint32_t *)0x400F900C))

//*****************************************************************************
//
// Hibernation module registers (HIB)
//
//*****************************************************************************
#define HIB_RTCC_R              (*((volatile uint32_t *)0x400FC000))
#define HIB_RTCM0_R             (*((volatile uint32_t *)0x400FC004))
#define HIB_RTCLD_R             (*((volatile uint32_t *)0x400FC00C))
#define HIB_CTL_R               (*((volatile uint32_t *)0x400FC010))
#define HIB_IM_R                (*((volatile uint32_t *)0x400FC014))
#define HIB_RIS_R               (*((volatile uint32_t *)0x400FC018))
#define HIB_MIS_R               (*((volatile uint32_t *)0x400FC01C))
#define HIB_IC_R                (*((volatile uint32_t *)0x400FC020))
#define HIB_RTCT_R              (*((volatile uint32_t *)0x400FC024))
#define HIB_RTCSS_R             (*((volatile uint32_t *)0x400FC028))
#define HIB_IO_R                (*((volatile uint32_t *)0x400FC02C))
#define HIB_DATA_R              (*((volatile uint32_t *)0x400FC030))
#define HIB_CALCTL_R            (*((volatile uint32_t *)0x400FC300))
#define HIB_CAL0_R              (*((volatile uint32_t *)0x400FC310))
#define HIB_CAL1_R              (*((volatile uint32_t *)0x400FC314))
#define HIB_CALLD0_R            (*((volatile uint32_t *)0x400FC320))
#define HIB_CALLD1_R            (*((volatile uint32_t *)0x400FC324))
#define HIB_CALM0_R             (*((volatile uint32_t *)0x400FC330))
#define HIB_CALM1_R             (*((volatile uint32_t *)0x400FC334))
#define HIB_LOCK_R              (*((volatile uint32_t *)0x400FC360))
#define HIB_TPCTL_R             (*((volatile uint32_t *)0x400FC400))
#define HIB_TPSTAT_R            (*((volatile uint32_t *)0x400FC404))
#define HIB_TPIO_R              (*((volatile uint32_t *)0x400FC410))
#define HIB_TPLOG0_R            (*((volatile uint32_t *)0x400FC4E0))
#define HIB_TPLOG1_R            (*((volatile uint32_t *)0x400FC4E4))
#define HIB_TPLOG2_R            (*((volatile uint32_t *)0x400FC4E8))
#define HIB_TPLOG3_R            (*((volatile uint32_t *)0x400FC4EC))
#define HIB_TPLOG4_R            (*((volatile uint32_t *)0x400FC4F0))
#define HIB_TPLOG5_R            (*((volatile uint32_t *)0x400FC4F4))
#define HIB_TPLOG6_R            (*((volatile uint32_t *)0x400FC4F8))
#define HIB_TPLOG7_R            (*((volatile uint32_t *)0x400FC4FC))
#define HIB_PP_R                (*((volatile uint32_t *)0x400FCFC0))
#define HIB_CC_R                (*((volatile uint32_t *)0x400FCFC8))

//*****************************************************************************
//
// FLASH registers (FLASH CTRL)
//
//*****************************************************************************
#define FLASH_FMA_R             (*((volatile uint32_t *)0x400FD000))
#define FLASH_FMD_R             (*((volatile uint32_t *)0x400FD004))
#define FLASH_FMC_R             (*((volatile uint32_t *)0x400FD008))
#define FLASH_FCRIS_R           (*((volatile uint32_t *)0x400FD00C))
#define FLASH_FCIM_R            (*((volatile uint32_t *)0x400FD010))
#define FLASH_FCMISC_R          (*((volatile uint32_t *)0x400FD014))
#define FLASH_FMC2_R            (*((volatile uint32_t *)0x400FD020))
#define FLASH_FWBVAL_R          (*((volatile uint32_t *)0x400FD030))
#define FLASH_FLPEKEY_R         (*((volatile uint32_t *)0x400FD03C))
#define FLASH_FWBN_R            (*((volatile uint32_t *)0x400FD100))
#define FLASH_PP_R              (*((volatile uint32_t *)0x400FDFC0))
#define FLASH_SSIZE_R           (*((volatile uint32_t *)0x400FDFC4))
#define FLASH_CONF_R            (*((volatile uint32_t *)0x400FDFC8))
#define FLASH_ROMSWMAP_R        (*((volatile uint32_t *)0x400FDFCC))
#define FLASH_DMASZ_R           (*((volatile uint32_t *)0x400FDFD0))
#define FLASH_DMAST_R           (*((volatile uint32_t *)0x400FDFD4))
#define FLASH_RVP_R             (*((volatile uint32_t *)0x400FE0D4))
#define FLASH_BOOTCFG_R         (*((volatile uint32_t *)0x400FE1D0))
#define FLASH_USERREG0_R        (*((volatile uint32_t *)0x400FE1E0))
#define FLASH_USERREG1_R        (*((volatile uint32_t *)0x400FE1E4))
#define FLASH_USERREG2_R        (*((volatile uint32_t *)0x400FE1E8))
#define FLASH_USERREG3_R        (*((volatile uint32_t *)0x400FE1EC))
#define FLASH_FMPRE0_R          (*((volatile uint32_t *)0x400FE200))
#define FLASH_FMPRE1_R          (*((volatile uint32_t *)0x400FE204))
#define FLASH_FMPRE2_R          (*((volatile uint32_t *)0x400FE208))
#define FLASH_FMPRE3_R          (*((volatile uint32_t *)0x400FE20C))
#define FLASH_FMPRE4_R          (*((volatile uint32_t *)0x400FE210))
#define FLASH_FMPRE5_R          (*((volatile uint32_t *)0x400FE214))
#define FLASH_FMPRE6_R          (*((volatile uint32_t *)0x400FE218))
#define FLASH_FMPRE7_R          (*((volatile uint32_t *)0x400FE21C))
#define FLASH_FMPRE8_R          (*((volatile uint32_t *)0x400FE220))
#define FLASH_FMPRE9_R          (*((volatile uint32_t *)0x400FE224))
#define FLASH_FMPRE10_R         (*((volatile uint32_t *)0x400FE228))
#define FLASH_FMPRE11_R         (*((volatile uint32_t *)0x400FE22C))
#define FLASH_FMPRE12_R         (*((volatile uint32_t *)0x400FE230))
#define FLASH_FMPRE13_R         (*((volatile uint32_t *)0x400FE234))
#define FLASH_FMPRE14_R         (*((volatile uint32_t *)0x400FE238))
#define FLASH_FMPRE15_R         (*((volatile uint32_t *)0x400FE23C))
#define FLASH_FMPPE0_R          (*((volatile uint32_t *)0x400FE400))
#define FLASH_FMPPE1_R          (*((volatile uint32_t *)0x400FE404))
#define FLASH_FMPPE2_R          (*((volatile uint32_t *)0x400FE408))
#define FLASH_FMPPE3_R          (*((volatile uint32_t *)0x400FE40C))
#define FLASH_FMPPE4_R          (*((volatile uint32_t *)0x400FE410))
#define FLASH_FMPPE5_R          (*((volatile uint32_t *)0x400FE414))
#define FLASH_FMPPE6_R          (*((volatile uint32_t *)0x400FE418))
#define FLASH_FMPPE7_R          (*((volatile uint32_t *)0x400FE41C))
#define FLASH_FMPPE8_R          (*((volatile uint32_t *)0x400FE420))
#define FLASH_FMPPE9_R          (*((volatile uint32_t *)0x400FE424))
#define FLASH_FMPPE10_R         (*((volatile uint32_t *)0x400FE428))
#define FLASH_FMPPE11_R         (*((volatile uint32_t *)0x400FE42C))
#define FLASH_FMPPE12_R         (*((volatile uint32_t *)0x400FE430))
#define FLASH_FMPPE13_R         (*((volatile uint32_t *)0x400FE434))
#define FLASH_FMPPE14_R         (*((volatile uint32_t *)0x400FE438))
#define FLASH_FMPPE15_R         (*((volatile uint32_t *)0x400FE43C))

//*****************************************************************************
//
// System Control registers (SYSCTL)
//
//*****************************************************************************
#define SYSCTL_DID0_R           (*((volatile uint32_t *)0x400FE000))
#define SYSCTL_DID1_R           (*((volatile uint32_t *)0x400FE004))
#define SYSCTL_PTBOCTL_R        (*((volatile uint32_t *)0x400FE038))
#define SYSCTL_RIS_R            (*((volatile uint32_t *)0x400FE050))
#define SYSCTL_IMC_R            (*((volatile uint32_t *)0x400FE054))
#define SYSCTL_MISC_R           (*((volatile uint32_t *)0x400FE058))
#define SYSCTL_RESC_R           (*((volatile uint32_t *)0x400FE05C))
#define SYSCTL_PWRTC_R          (*((volatile uint32_t *)0x400FE060))
#define SYSCTL_NMIC_R           (*((volatile uint32_t *)0x400FE064))
#define SYSCTL_MOSCCTL_R        (*((volatile uint32_t *)0x400FE07C))
#define SYSCTL_RSCLKCFG_R       (*((volatile uint32_t *)0x400FE0B0))
#define SYSCTL_MEMTIM0_R        (*((volatile uint32_t *)0x400FE0C0))
#define SYSCTL_ALTCLKCFG_R      (*((volatile uint32_t *)0x400FE138))
#define SYSCTL_DSCLKCFG_R       (*((volatile uint32_t *)0x400FE144))
#define SYSCTL_DIVSCLK_R        (*((volatile uint32_t *)0x400FE148))
#define SYSCTL_SYSPROP_R        (*((volatile uint32_t *)0x400FE14C))
#define SYSCTL_PIOSCCAL_R       (*((volatile uint32_t *)0x400FE150))
#define SYSCTL_PIOSCSTAT_R      (*((volatile uint32_t *)0x400FE154))
#define SYSCTL_PLLFREQ0_R       (*((volatile uint32_t *)0x400FE160))
#define SYSCTL_PLLFREQ1_R       (*((volatile uint32_t *)0x400FE164))
#define SYSCTL_PLLSTAT_R        (*((volatile uint32_t *)0x400FE168))
#define SYSCTL_SLPPWRCFG_R      (*((volatile uint32_t *)0x400FE188))
#define SYSCTL_DSLPPWRCFG_R     (*((volatile uint32_t *)0x400FE18C))
#define SYSCTL_NVMSTAT_R        (*((volatile uint32_t *)0x400FE1A0))
#define SYSCTL_LDOSPCTL_R       (*((volatile uint32_t *)0x400FE1B4))
#define SYSCTL_LDODPCTL_R       (*((volatile uint32_t *)0x400FE1BC))
#define SYSCTL_RESBEHAVCTL_R    (*((volatile uint32_t *)0x400FE1D8))
#define SYSCTL_HSSR_R           (*((volatile uint32_t *)0x400FE1F4))
#define SYSCTL_USBPDS_R         (*((volatile uint32_t *)0x400FE280))
#define SYSCTL_USBMPC_R         (*((volatile uint32_t *)0x400FE284))
#define SYSCTL_EMACPDS_R        (*((volatile uint32_t *)0x400FE288))
#define SYSCTL_EMACMPC_R        (*((volatile uint32_t *)0x400FE28C))
#define SYSCTL_PPWD_R           (*((volatile uint32_t *)0x400FE300))
#define SYSCTL_PPTIMER_R        (*((volatile uint32_t *)0x400FE304))
#define SYSCTL_PPGPIO_R         (*((volatile uint32_t *)0x400FE308))
#define SYSCTL_PPDMA_R          (*((volatile uint32_t *)0x400FE30C))
#define SYSCTL_PPEPI_R          (*((volatile uint32_t *)0x400FE310))
#define SYSCTL_PPHIB_R          (*((volatile uint32_t *)0x400FE314))
#define SYSCTL_PPUART_R         (*((volatile uint32_t *)0x400FE318))
#define SYSCTL_PPSSI_R          (*((volatile uint32_t *)0x400FE31C))
#define SYSCTL_PPI2C_R          (*((volatile uint32_t *)0x400FE320))
#define SYSCTL_PPUSB_R          (*((volatile uint32_t *)0x400FE328))
#define SYSCTL_PPEPHY_R         (*((volatile uint32_t *)0x400FE330))
#define SYSCTL_PPCAN_R          (*((volatile uint32_t *)0x400FE334))
#define SYSCTL_PPADC_R          (*((volatile uint32_t *)0x400FE338))
#define SYSCTL_PPACMP_R         (*((volatile uint32_t *)0x400FE33C))
#define SYSCTL_PPPWM_R          (*((volatile uint32_t *)0x400FE340))
#define SYSCTL_PPQEI_R          (*((volatile uint32_t *)0x400FE344))
#define SYSCTL_PPLPC_R          (*((volatile uint32_t *)0x400FE348))
#define SYSCTL_PPPECI_R         (*((volatile uint32_t *)0x400FE350))
#define SYSCTL_PPFAN_R          (*((volatile uint32_t *)0x400FE354))
#define SYSCTL_PPEEPROM_R       (*((volatile uint32_t *)0x400FE358))
#define SYSCTL_PPWTIMER_R       (*((volatile uint32_t *)0x400FE35C))
#define SYSCTL_PPRTS_R          (*((volatile uint32_t *)0x400FE370))
#define SYSCTL_PPCCM_R          (*((volatile uint32_t *)0x400FE374))
#define SYSCTL_PPLCD_R          (*((volatile uint32_t *)0x400FE390))
#define SYSCTL_PPOWIRE_R        (*((volatile uint32_t *)0x400FE398))
#define SYSCTL_PPEMAC_R         (*((volatile uint32_t *)0x400FE39C))
#define SYSCTL_PPHIM_R          (*((volatile uint32_t *)0x400FE3A4))
#define SYSCTL_SRWD_R           (*((volatile uint32_t *)0x400FE500))
#define SYSCTL_SRTIMER_R        (*((volatile uint32_t *)0x400FE504))
#define SYSCTL_SRGPIO_R         (*((volatile uint32_t *)0x400FE508))
#define SYSCTL_SRDMA_R          (*((volatile uint32_t *)0x400FE50C))
#define SYSCTL_SREPI_R          (*((volatile uint32_t *)0x400FE510))
#define SYSCTL_SRHIB_R          (*((volatile uint32_t *)0x400FE514))
#define SYSCTL_SRUART_R         (*((volatile uint32_t *)0x400FE518))
#define SYSCTL_SRSSI_R          (*((volatile uint32_t *)0x400FE51C))
#define SYSCTL_SRI2C_R          (*((volatile uint32_t *)0x400FE520))
#define SYSCTL_SRUSB_R          (*((volatile uint32_t *)0x400FE528))
#define SYSCTL_SREPHY_R         (*((volatile uint32_t *)0x400FE530))
#define SYSCTL_SRCAN_R          (*((volatile uint32_t *)0x400FE534))
#define SYSCTL_SRADC_R          (*((volatile uint32_t *)0x400FE538))
#define SYSCTL_SRACMP_R         (*((volatile uint32_t *)0x400FE53C))
#define SYSCTL_SRPWM_R          (*((volatile uint32_t *)0x400FE540))
#define SYSCTL_SRQEI_R          (*((volatile uint32_t *)0x400FE544))
#define SYSCTL_SREEPROM_R       (*((volatile uint32_t *)0x400FE558))
#define SYSCTL_SRCCM_R          (*((volatile uint32_t *)0x400FE574))
#define SYSCTL_SREMAC_R         (*((volatile uint32_t *)0x400FE59C))
#define SYSCTL_RCGCWD_R         (*((volatile uint32_t *)0x400FE600))
#define SYSCTL_RCGCTIMER_R      (*((volatile uint32_t *)0x400FE604))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCDMA_R        (*((volatile uint32_t *)0x400FE60C))
#define SYSCTL_RCGCEPI_R        (*((volatile uint32_t *)0x400FE610))
#define SYSCTL_RCGCHIB_R        (*((volatile uint32_t *)0x400FE614))
#define SYSCTL_RCGCUART_R       (*((volatile uint32_t *)0x400FE618))
#define SYSCTL_RCGCSSI_R        (*((volatile uint32_t *)0x400FE61C))
#define SYSCTL_RCGCI2C_R        (*((volatile uint32_t *)0x400FE620))
#define SYSCTL_RCGCUSB_R        (*((volatile uint32_t *)0x400FE628))
#define SYSCTL_RCGCEPHY_R       (*((volatile uint32_t *)0x400FE630))
#define SYSCTL_RCGCCAN_R        (*((volatile uint32_t *)0x400FE634))
#define SYSCTL_RCGCADC_R        (*((volatile uint32_t *)0x400FE638))
#define SYSCTL_RCGCACMP_R       (*((volatile uint32_t *)0x400FE63C))
#define SYSCTL_RCGCPWM_R        (*((volatile uint32_t *)0x400FE640))
#define SYSCTL_RCGCQEI_R        (*((volatile uint32_t *)0x400FE644))
#define SYSCTL_RCGCEEPROM_R     (*((volatile uint32_t *)0x400FE658))
#define SYSCTL_RCGCCCM_R        (*((volatile uint32_t *)0x400FE674))
#define SYSCTL_RCGCEMAC_R       (*((volatile uint32_t *)0x400FE69C))
#define SYSCTL_SCGCWD_R         (*((volatile uint32_t *)0x400FE700))
#define SYSCTL_SCGCTIMER_R      (*((volatile uint32_t *)0x400FE704))
#define SYSCTL_SCGCGPIO_R       (*((volatile uint32_t *)0x400FE708))
#define SYSCTL_SCGCDMA_R        (*((volatile uint32_t *)0x400FE70C))
#define SYSCTL_SCGCEPI_R        (*((volatile uint32_t *)0x400FE710))
#define SYSCTL_SCGCHIB_R        (*((volatile uint32_t *)0x400FE714))
#define SYSCTL_SCGCUART_R       (*((volatile uint32_t *)0x400FE718))
#define SYSCTL_SCGCSSI_R        (*((volatile uint32_t *)0x400FE71C))
#define SYSCTL_SCGCI2C_R        (*((volatile uint32_t *)0x400FE720))
#define SYSCTL_SCGCUSB_R        (*((volatile uint32_t *)0x400FE728))
#define SYSCTL_SCGCEPHY_R       (*((volatile uint32_t *)0x400FE730))
#define SYSCTL_SCGCCAN_R        (*((volatile uint32_t *)0x400FE734))
#define SYSCTL_SCGCADC_R        (*((volatile uint32_t *)0x400FE738))
#define SYSCTL_SCGCACMP_R       (*((volatile uint32_t *)0x400FE73C))
#define SYSCTL_SCGCPWM_R        (*((volatile uint32_t *)0x400FE740))
#define SYSCTL_SCGCQEI_R        (*((volatile uint32_t *)0x400FE744))
#define SYSCTL_SCGCEEPROM_R     (*((volatile uint32_t *)0x400FE758))
#define SYSCTL_SCGCCCM_R        (*((volatile uint32_t *)0x400FE774))
#define SYSCTL_SCGCEMAC_R       (*((volatile uint32_t *)0x400FE79C))
#define SYSCTL_DCGCWD_R         (*((volatile uint32_t *)0x400FE800))
#define SYSCTL_DCGCTIMER_R      (*((volatile uint32_t *)0x400FE804))
#define SYSCTL_DCGCGPIO_R       (*((volatile uint32_t *)0x400FE808))
#define SYSCTL_DCGCDMA_R        (*((volatile uint32_t *)0x400FE80C))
#define SYSCTL_DCGCEPI_R        (*((volatile uint32_t *)0x400FE810))
#define SYSCTL_DCGCHIB_R        (*((volatile uint32_t *)0x400FE814))
#define SYSCTL_DCGCUART_R       (*((volatile uint32_t *)0x400FE818))
#define SYSCTL_DCGCSSI_R        (*((volatile uint32_t *)0x400FE81C))
#define SYSCTL_DCGCI2C_R        (*((volatile uint32_t *)0x400FE820))
#define SYSCTL_DCGCUSB_R        (*((volatile uint32_t *)0x400FE828))
#define SYSCTL_DCGCEPHY_R       (*((volatile uint32_t *)0x400FE830))
#define SYSCTL_DCGCCAN_R        (*((volatile uint32_t *)0x400FE834))
#define SYSCTL_DCGCADC_R        (*((volatile uint32_t *)0x400FE838))
#define SYSCTL_DCGCACMP_R       (*((volatile uint32_t *)0x400FE83C))
#define SYSCTL_DCGCPWM_R        (*((volatile uint32_t *)0x400FE840))
#define SYSCTL_DCGCQEI_R        (*((volatile uint32_t *)0x400FE844))
#define SYSCTL_DCGCEEPROM_R     (*((volatile uint32_t *)0x400FE858))
#define SYSCTL_DCGCCCM_R        (*((volatile uint32_t *)0x400FE874))
#define SYSCTL_DCGCEMAC_R       (*((volatile uint32_t *)0x400FE89C))
#define SYSCTL_PCWD_R           (*((volatile uint32_t *)0x400FE900))
#define SYSCTL_PCTIMER_R        (*((volatile uint32_t *)0x400FE904))
#define SYSCTL_PCGPIO_R         (*((volatile uint32_t *)0x400FE908))
#define SYSCTL_PCDMA_R          (*((volatile uint32_t *)0x400FE90C))
#define SYSCTL_PCEPI_R          (*((volatile uint32_t *)0x400FE910))
#define SYSCTL_PCHIB_R          (*((volatile uint32_t *)0x400FE914))
#define SYSCTL_PCUART_R         (*((volatile uint32_t *)0x400FE918))
#define SYSCTL_PCSSI_R          (*((volatile uint32_t *)0x400FE91C))
#define SYSCTL_PCI2C_R          (*((volatile uint32_t *)0x400FE920))
#define SYSCTL_PCUSB_R          (*((volatile uint32_t *)0x400FE928))
#define SYSCTL_PCEPHY_R         (*((volatile uint32_t *)0x400FE930))
#define SYSCTL_PCCAN_R          (*((volatile uint32_t *)0x400FE934))
#define SYSCTL_PCADC_R          (*((volatile uint32_t *)0x400FE938))
#define SYSCTL_PCACMP_R         (*((volatile uint32_t *)0x400FE93C))
#define SYSCTL_PCPWM_R          (*((volatile uint32_t *)0x400FE940))
#define SYSCTL_PCQEI_R          (*((volatile uint32_t *)0x400FE944))
#define SYSCTL_PCEEPROM_R       (*((volatile uint32_t *)0x400FE958))
#define SYSCTL_PCCCM_R          (*((volatile uint32_t *)0x400FE974))
#define SYSCTL_PCEMAC_R         (*((volatile uint32_t *)0x400FE99C))
#define SYSCTL_PRWD_R           (*((volatile uint32_t *)0x400FEA00))
#define SYSCTL_PRTIMER_R        (*((volatile uint32_t *)0x400FEA04))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRDMA_R          (*((volatile uint32_t *)0x400FEA0C))
#define SYSCTL_PREPI_R          (*((volatile uint32_t *)0x400FEA10))
#define SYSCTL_PRHIB_R          (*((volatile uint32_t *)0x400FEA14))
#define SYSCTL_PRUART_R         (*((volatile uint32_t *)0x400FEA18))
#define SYSCTL_PRSSI_R          (*((volatile uint32_t *)0x400FEA1C))
#define SYSCTL_PRI2C_R          (*((volatile uint32_t *)0x400FEA20))
#define SYSCTL_PRUSB_R          (*((volatile uint32_t *)0x400FEA28))
#define SYSCTL_PREPHY_R         (*((volatile uint32_t *)0x400FEA30))
#define SYSCTL_PRCAN_R          (*((volatile uint32_t *)0x400FEA34))
#define SYSCTL_PRADC_R          (*((volatile uint32_t *)0x400FEA38))
#define SYSCTL_PRACMP_R         (*((volatile uint32_t *)0x400FEA3C))
#define SYSCTL_PRPWM_R          (*((volatile uint32_t *)0x400FEA40))
#define SYSCTL_PRQEI_R          (*((volatile uint32_t *)0x400FEA44))
#define SYSCTL_PREEPROM_R       (*((volatile uint32_t *)0x400FEA58))
#define SYSCTL_PRCCM_R          (*((volatile uint32_t *)0x400FEA74))
#define SYSCTL_PREMAC_R         (*((volatile uint32_t *)0x400FEA9C))

//*****************************************************************************
//
// Micro Direct Memory Access registers (UDMA)
//
//*****************************************************************************
#define UDMA_STAT_R             (*((volatile uint32_t *)0x400FF000))
#define UDMA_CFG_R              (*((volatile uint32_t *)0x400FF004))
#define UDMA_CTLBASE_R          (*((volatile uint32_t *)0x400FF008))
#define UDMA_ALTBASE_R          (*((volatile uint32_t *)0x400FF00C))
#define UDMA_WAITSTAT_R         (*((volatile uint32_t *)0x400FF010))
#define UDMA_SWREQ_R            (*((volatile uint32_t *)0x400FF014))
#define UDMA_USEBURSTSET_R      (*((volatile uint32_t *)0x400FF018))
#define UDMA_USEBURSTCLR_R      (*((volatile uint32_t *)0x400FF01C))
#define UDMA_REQMASKSET_R       (*((volatile uint32_t *)0x400FF020))
#define UDMA_REQMASKCLR_R       (*((volatile uint32_t *)0x400FF024))
#define UDMA_ENASET_R           (*((volatile uint32_t *)0x400FF028))
#define UDMA_ENACLR_R           (*((volatile uint32_t *)0x400FF02C))
#define UDMA_ALTSET_R           (*((volatile uint32_t *)0x400FF030))
#define UDMA_ALTCLR_R           (*((volatile uint32_t *)0x400FF034))
#define UDMA_PRIOSET_R          (*((volatile uint32_t *)0x400FF038))
#define UDMA_PRIOCLR_R          (*((volatile uint32_t *)0x400FF03C))
#define UDMA_ERRCLR_R           (*((volatile uint32_t *)0x400FF04C))
#define UDMA_CHASGN_R           (*((volatile uint32_t *)0x400FF500))
#define UDMA_CHMAP0_R           (*((volatile uint32_t *)0x400FF510))
#define UDMA_CHMAP1_R           (*((volatile uint32_t *)0x400FF514))
#define UDMA_CHMAP2_R           (*((volatile uint32_t *)0x400FF518))
#define UDMA_CHMAP3_R           (*((volatile uint32_t *)0x400FF51C))

//*****************************************************************************
//
// Micro Direct Memory Access (uDMA) offsets (UDMA)
//
//*****************************************************************************
#define UDMA_SRCENDP            0x00000000  // DMA Channel Source Address End
                                            // Pointer
#define UDMA_DSTENDP            0x00000004  // DMA Channel Destination Address
                                            // End Pointer
#define UDMA_CHCTL              0x00000008  // DMA Channel Control Word

//*****************************************************************************
//
// EC registers (CCM0)
//
//*****************************************************************************
#define CCM0_CRCCTRL_R          (*((volatile uint32_t *)0x44030400))
#define CCM0_CRCSEED_R          (*((volatile uint32_t *)0x44030410))
#define CCM0_CRCDIN_R           (*((volatile uint32_t *)0x44030414))
#define CCM0_CRCRSLTPP_R        (*((volatile uint32_t *)0x44030418))

//*****************************************************************************
//
// NVIC registers (NVIC)
//
//*****************************************************************************
#define NVIC_ACTLR_R            (*((volatile uint32_t *)0xE000E008))
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_EN0_R              (*((volatile uint32_t *)0xE000E100))
#define NVIC_EN1_R              (*((volatile uint32_t *)0xE000E104))
#define NVIC_EN2_R              (*((volatile uint32_t *)0xE000E108))
#define NVIC_EN3_R              (*((volatile uint32_t *)0xE000E10C))
#define NVIC_DIS0_R             (*((volatile uint32_t *)0xE000E180))
#define NVIC_DIS1_R             (*((volatile uint32_t *)0xE000E184))
#define NVIC_DIS2_R             (*((volatile uint32_t *)0xE000E188))
#define NVIC_DIS3_R             (*((volatile uint32_t *)0xE000E18C))
#define NVIC_PEND0_R            (*((volatile uint32_t *)0xE000E200))
#define NVIC_PEND1_R            (*((volatile uint32_t *)0xE000E204))
#define NVIC_PEND2_R            (*((volatile uint32_t *)0xE000E208))
#define NVIC_PEND3_R            (*((volatile uint32_t *)0xE000E20C))
#define NVIC_UNPEND0_R          (*((volatile uint32_t *)0xE000E280))
#define NVIC_UNPEND1_R          (*((volatile uint32_t *)0xE000E284))
#define NVIC_UNPEND2_R          (*((volatile uint32_t *)0xE000E288))
#define NVIC_UNPEND3_R          (*((volatile uint32_t *)0xE000E28C))
#define NVIC_ACTIVE0_R          (*((volatile uint32_t *)0xE000E300))
#define NVIC_ACTIVE1_R          (*((volatile uint32_t *)0xE000E304))
#define NVIC_ACTIVE2_R          (*((volatile uint32_t *)0xE000E308))
#define NVIC_ACTIVE3_R          (*((volatile uint32_t *)0xE000E30C))
#define NVIC_PRI0_R             (*((volatile uint32_t *)0xE000E400))
#define NVIC_PRI1_R             (*((volatile uint32_t *)0xE000E404))
#define NVIC_PRI2_R             (*((volatile uint32_t *)0xE000E408))
#define NVIC_PRI3_R             (*((volatile uint32_t *)0xE000E40C))
#define NVIC_PRI4_R             (*((volatile uint32_t *)0xE000E410))
#define NVIC_PRI5_R             (*((volatile uint32_t *)0xE000E414))
#define NVIC_PRI6_R             (*((volatile uint32_t *)0xE000E418))
#define NVIC_PRI7_R             (*((volatile uint32_t *)0xE000E41C))
#define NVIC_PRI8_R             (*((volatile uint32_t *)0xE000E420))
#define NVIC_PRI9_R             (*((volatile uint32_t *)0xE000E424))
#define NVIC_PRI10_R            (*((volatile uint32_t *)0xE000E428))
#define NVIC_PRI11_R            (*((volatile uint32_t *)0xE000E42C))
#define NVIC_PRI12_R            (*((volatile uint32_t *)0xE000E430))
#define NVIC_PRI13_R            (*((volatile uint32_t *)0xE000E434))
#define NVIC_PRI14_R            (*((volatile uint32_t *)0xE000E438))
#define NVIC_PRI15_R            (*((volatile uint32_t *)0xE000E43C))
#define NVIC_PRI16_R            (*((volatile uint32_t *)0xE000E440))
#define NVIC_PRI17_R            (*((volatile uint32_t *)0xE000E444))
#define NVIC_PRI18_R            (*((volatile uint32_t *)0xE000E448))
#define NVIC_PRI19_R            (*((volatile uint32_t *)0xE000E44C))
#define NVIC_PRI20_R            (*((volatile uint32_t *)0xE000E450))
#define NVIC_PRI21_R            (*((volatile uint32_t *)0xE000E454))
#define NVIC_PRI22_R            (*((volatile uint32_t *)0xE000E458))
#define NVIC_PRI23_R            (*((volatile uint32_t *)0xE000E45C))
#define NVIC_PRI24_R            (*((volatile uint32_t *)0xE000E460))
#define NVIC_PRI25_R            (*((volatile uint32_t *)0xE000E464))
#define NVIC_PRI26_R            (*((volatile uint32_t *)0xE000E468))
#define NVIC_PRI27_R            (*((volatile uint32_t *)0xE000E46C))
#define NVIC_PRI28_R            (*((volatile uint32_t *)0xE000E470))
#define NVIC_CPUID_R            (*((volatile uint32_t *)0xE000ED00))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_VTABLE_R           (*((volatile uint32_t *)0xE000ED08))
#define NVIC_APINT_R            (*((volatile uint32_t *)0xE000ED0C))
#define NVIC_SYS_CTRL_R         (*((volatile uint32_t *)0xE000ED10))
#define NVIC_CFG_CTRL_R         (*((volatile uint32_t *)0xE000ED14))
#define NVIC_SYS_PRI1_R         (*((volatile uint32_t *)0xE000ED18))
#define NVIC_SYS_PRI2_R         (*((volatile uint32_t *)0xE000ED1C))
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))
#define NVIC_SYS_HND_CTRL_R     (*((volatile uint32_t *)0xE000ED24))
#define NVIC_FAULT_STAT_R       (*((volatile uint32_t *)0xE000ED28))
#define NVIC_HFAULT_STAT_R      (*((volatile uint32_t *)0xE000ED2C))
#define NVIC_DEBUG_STAT_R       (*((volatile uint32_t *)0xE000ED30))
#define NVIC_MM_ADDR_R          (*((volatile uint32_t *)0xE000ED34))
#define NVIC_FAULT_ADDR_R       (*((volatile uint32_t *)0xE000ED38))
#define NVIC_CPAC_R             (*((volatile uint32_t *)0xE000ED88))
#define NVIC_MPU_TYPE_R         (*((volatile uint32_t *)0xE000ED90))
#define NVIC_MPU_CTRL_R         (*((volatile uint32_t *)0xE000ED94))
#define NVIC_MPU_NUMBER_R       (*((volatile uint32_t *)0xE000ED98))
#define NVIC_MPU_BASE_R         (*((volatile uint32_t *)0xE000ED9C))
#define NVIC_MPU_ATTR_R         (*((volatile uint32_t *)0xE000EDA0))
#define NVIC_MPU_BASE1_R        (*((volatile uint32_t *)0xE000EDA4))
#define NVIC_MPU_ATTR1_R        (*((volatile uint32_t *)0xE000EDA8))
#define NVIC_MPU_BASE2_R        (*((volatile uint32_t *)0xE000EDAC))
#define NVIC_MPU_ATTR2_R        (*((volatile uint32_t *)0xE000EDB0))
#define NVIC_MPU_BASE3_R        (*((volatile uint32_t *)0xE000EDB4))
#define NVIC_MPU_ATTR3_R        (*((volatile uint32_t *)0xE000EDB8))
#define NVIC_DBG_CTRL_R         (*((volatile uint32_t *)0xE000EDF0))
#define NVIC_DBG_XFER_R         (*((volatile uint32_t *)0xE000EDF4))
#define NVIC_DBG_DATA_R         (*((volatile uint32_t *)0xE000EDF8))
#define NVIC_DBG_INT_R          (*((volatile uint32_t *)0xE000EDFC))
#define NVIC_SW_TRIG_R          (*((volatile uint32_t *)0xE000EF00))
#define NVIC_FPCC_R             (*((volatile uint32_t *)0xE000EF34))
#define NVIC_FPCA_R             (*((volatile uint32_t *)0xE000EF38))
#define NVIC_FPDSC_R            (*((volatile uint32_t *)0xE000EF3C))

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
//*****************************************************************************
#define WDT_LOAD_M              0xFFFFFFFF  // Watchdog Load Value
#define WDT_LOAD_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
//*****************************************************************************
#define WDT_VALUE_M             0xFFFFFFFF  // Watchdog Value
#define WDT_VALUE_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_CTL register.
//
//*****************************************************************************
#define WDT_CTL_WRC             0x80000000  // Write Complete
#define WDT_CTL_INTTYPE         0x00000004  // Watchdog Interrupt Type
#define WDT_CTL_RESEN           0x00000002  // Watchdog Reset Enable
#define WDT_CTL_INTEN           0x00000001  // Watchdog Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
//*****************************************************************************
#define WDT_ICR_M               0xFFFFFFFF  // Watchdog Interrupt Clear
#define WDT_ICR_S               0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
//*****************************************************************************
#define WDT_RIS_WDTRIS          0x00000001  // Watchdog Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
//*****************************************************************************
#define WDT_MIS_WDTMIS          0x00000001  // Watchdog Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_TEST register.
//
//*****************************************************************************
#define WDT_TEST_STALL          0x00000100  // Watchdog Stall Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
//*****************************************************************************
#define WDT_LOCK_M              0xFFFFFFFF  // Watchdog Lock
#define WDT_LOCK_UNLOCKED       0x00000000  // Unlocked
#define WDT_LOCK_LOCKED         0x00000001  // Locked
#define WDT_LOCK_UNLOCK         0x1ACCE551  // Unlocks the watchdog timer

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_FRF_TI          0x00000010  // Synchronous Serial Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_4           0x00000003  // 4-bit data
#define SSI_CR0_DSS_5           0x00000004  // 5-bit data
#define SSI_CR0_DSS_6           0x00000005  // 6-bit data
#define SSI_CR0_DSS_7           0x00000006  // 7-bit data
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR0_DSS_9           0x00000008  // 9-bit data
#define SSI_CR0_DSS_10          0x00000009  // 10-bit data
#define SSI_CR0_DSS_11          0x0000000A  // 11-bit data
#define SSI_CR0_DSS_12          0x0000000B  // 12-bit data
#define SSI_CR0_DSS_13          0x0000000C  // 13-bit data
#define SSI_CR0_DSS_14          0x0000000D  // 14-bit data
#define SSI_CR0_DSS_15          0x0000000E  // 15-bit data
#define SSI_CR0_DSS_16          0x0000000F  // 16-bit data
#define SSI_CR0_SCR_S           8

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
#define SSI_CR1_EOM             0x00000800  // Stop Frame (End of Message)
#define SSI_CR1_FSSHLDFRM       0x00000400  // FSS Hold Frame
#define SSI_CR1_HSCLKEN         0x00000200  // High Speed Clock Enable
#define SSI_CR1_DIR             0x00000100  // SSI Direction of Operation
#define SSI_CR1_MODE_M          0x000000C0  // SSI Mode
#define SSI_CR1_MODE_LEGACY     0x00000000  // Legacy SSI mode
#define SSI_CR1_MODE_BI         0x00000040  // Bi-SSI mode
#define SSI_CR1_MODE_QUAD       0x00000080  // Quad-SSI Mode
#define SSI_CR1_MODE_ADVANCED   0x000000C0  // Advanced SSI Mode with 8-bit
                                            // packet size
#define SSI_CR1_EOT             0x00000010  // End of Transmission
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable
#define SSI_CR1_LBM             0x00000001  // SSI Loopback Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
#define SSI_DR_DATA_M           0x0000FFFF  // SSI Receive/Transmit Data
#define SSI_DR_DATA_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_RFF              0x00000008  // SSI Receive FIFO Full
#define SSI_SR_RNE              0x00000004  // SSI Receive FIFO Not Empty
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_SR_TFE              0x00000001  // SSI Transmit FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CPSR_CPSDVSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
#define SSI_IM_EOTIM            0x00000040  // End of Transmit Interrupt Mask
#define SSI_IM_DMATXIM          0x00000020  // SSI Transmit DMA Interrupt Mask
#define SSI_IM_DMARXIM          0x00000010  // SSI Receive DMA Interrupt Mask
#define SSI_IM_TXIM             0x00000008  // SSI Transmit FIFO Interrupt Mask
#define SSI_IM_RXIM             0x00000004  // SSI Receive FIFO Interrupt Mask
#define SSI_IM_RTIM             0x00000002  // SSI Receive Time-Out Interrupt
                                            // Mask
#define SSI_IM_RORIM            0x00000001  // SSI Receive Overrun Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
#define SSI_RIS_EOTRIS          0x00000040  // End of Transmit Raw Interrupt
                                            // Status
#define SSI_RIS_DMATXRIS        0x00000020  // SSI Transmit DMA Raw Interrupt
                                            // Status
#define SSI_RIS_DMARXRIS        0x00000010  // SSI Receive DMA Raw Interrupt
                                            // Status
#define SSI_RIS_TXRIS           0x00000008  // SSI Transmit FIFO Raw Interrupt
                                            // Status
#define SSI_RIS_RXRIS           0x00000004  // SSI Receive FIFO Raw Interrupt
                                            // Status
#define SSI_RIS_RTRIS           0x00000002  // SSI Receive Time-Out Raw
                                            // Interrupt Status
#define SSI_RIS_RORRIS          0x00000001  // SSI Receive Overrun Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
#define SSI_MIS_EOTMIS          0x00000040  // End of Transmit Masked Interrupt
                                            // Status
#define SSI_MIS_DMATXMIS        0x00000020  // SSI Transmit DMA Masked
                                            // Interrupt Status
#define SSI_MIS_DMARXMIS        0x00000010  // SSI Receive DMA Masked Interrupt
                                            // Status
#define SSI_MIS_TXMIS           0x00000008  // SSI Transmit FIFO Masked
                                            // Interrupt Status
#define SSI_MIS_RXMIS           0x00000004  // SSI Receive FIFO Masked
                                            // Interrupt Status
#define SSI_MIS_RTMIS           0x00000002  // SSI Receive Time-Out Masked
                                            // Interrupt Status
#define SSI_MIS_RORMIS          0x00000001  // SSI Receive Overrun Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
#define SSI_ICR_EOTIC           0x00000040  // End of Transmit Interrupt Clear
#define SSI_ICR_DMATXIC         0x00000020  // SSI Transmit DMA Interrupt Clear
#define SSI_ICR_DMARXIC         0x00000010  // SSI Receive DMA Interrupt Clear
#define SSI_ICR_RTIC            0x00000002  // SSI Receive Time-Out Interrupt
                                            // Clear
#define SSI_ICR_RORIC           0x00000001  // SSI Receive Overrun Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
//*****************************************************************************
#define SSI_DMACTL_TXDMAE       0x00000002  // Transmit DMA Enable
#define SSI_DMACTL_RXDMAE       0x00000001  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_PP register.
//
//*****************************************************************************
#define SSI_PP_FSSHLDFRM        0x00000008  // FSS Hold Frame Capability
#define SSI_PP_MODE_M           0x00000006  // Mode of Operation
#define SSI_PP_MODE_LEGACY      0x00000000  // Legacy SSI mode
#define SSI_PP_MODE_ADVBI       0x00000002  // Legacy mode, Advanced SSI mode
                                            // and Bi-SSI mode enabled
#define SSI_PP_MODE_ADVBIQUAD   0x00000004  // Legacy mode, Advanced mode,
                                            // Bi-SSI and Quad-SSI mode enabled
#define SSI_PP_HSCLK            0x00000001  // High Speed Capability

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
//*****************************************************************************
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // System clock (based on clock
                                            // source and divisor factor)
#define SSI_CC_CS_PIOSC         0x00000005  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
//*****************************************************************************
#define UART_DR_OE              0x00000800  // UART Overrun Error
#define UART_DR_BE              0x00000400  // UART Break Error
#define UART_DR_PE              0x00000200  // UART Parity Error
#define UART_DR_FE              0x00000100  // UART Framing Error
#define UART_DR_DATA_M          0x000000FF  // Data Transmitted or Received
#define UART_DR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
//*****************************************************************************
#define UART_RSR_OE             0x00000008  // UART Overrun Error
#define UART_RSR_BE             0x00000004  // UART Break Error
#define UART_RSR_PE             0x00000002  // UART Parity Error
#define UART_RSR_FE             0x00000001  // UART Framing Error

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
#define UART_ECR_DATA_M         0x000000FF  // Error Clear
#define UART_ECR_DATA_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
//*****************************************************************************
#define UART_FR_RI              0x00000100  // Ring Indicator
#define UART_FR_TXFE            0x00000080  // UART Transmit FIFO Empty
#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_FR_BUSY            0x00000008  // UART Busy
#define UART_FR_DCD             0x00000004  // Data Carrier Detect
#define UART_FR_DSR             0x00000002  // Data Set Ready
#define UART_FR_CTS             0x00000001  // Clear To Send

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
#define UART_ILPR_ILPDVSR_M     0x000000FF  // IrDA Low-Power Divisor
#define UART_ILPR_ILPDVSR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
//*****************************************************************************
#define UART_IBRD_DIVINT_M      0x0000FFFF  // Integer Baud-Rate Divisor
#define UART_IBRD_DIVINT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
//*****************************************************************************
#define UART_FBRD_DIVFRAC_M     0x0000003F  // Fractional Baud-Rate Divisor
#define UART_FBRD_DIVFRAC_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
#define UART_LCRH_SPS           0x00000080  // UART Stick Parity Select
#define UART_LCRH_WLEN_M        0x00000060  // UART Word Length
#define UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define UART_LCRH_WLEN_8        0x00000060  // 8 bits
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select
#define UART_LCRH_EPS           0x00000004  // UART Even Parity Select
#define UART_LCRH_PEN           0x00000002  // UART Parity Enable
#define UART_LCRH_BRK           0x00000001  // UART Send Break

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
//*****************************************************************************
#define UART_CTL_CTSEN          0x00008000  // Enable Clear To Send
#define UART_CTL_RTSEN          0x00004000  // Enable Request to Send
#define UART_CTL_RTS            0x00000800  // Request to Send
#define UART_CTL_DTR            0x00000400  // Data Terminal Ready
#define UART_CTL_RXE            0x00000200  // UART Receive Enable
#define UART_CTL_TXE            0x00000100  // UART Transmit Enable
#define UART_CTL_LBE            0x00000080  // UART Loop Back Enable
#define UART_CTL_HSE            0x00000020  // High-Speed Enable
#define UART_CTL_EOT            0x00000010  // End of Transmission
#define UART_CTL_SMART          0x00000008  // ISO 7816 Smart Card Support
#define UART_CTL_SIRLP          0x00000004  // UART SIR Low-Power Mode
#define UART_CTL_SIREN          0x00000002  // UART SIR Enable
#define UART_CTL_UARTEN         0x00000001  // UART Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
//*****************************************************************************
#define UART_IFLS_RX_M          0x00000038  // UART Receive Interrupt FIFO
                                            // Level Select
#define UART_IFLS_RX1_8         0x00000000  // RX FIFO >= 1/8 full
#define UART_IFLS_RX2_8         0x00000008  // RX FIFO >= 1/4 full
#define UART_IFLS_RX4_8         0x00000010  // RX FIFO >= 1/2 full (default)
#define UART_IFLS_RX6_8         0x00000018  // RX FIFO >= 3/4 full
#define UART_IFLS_RX7_8         0x00000020  // RX FIFO >= 7/8 full
#define UART_IFLS_TX_M          0x00000007  // UART Transmit Interrupt FIFO
                                            // Level Select
#define UART_IFLS_TX1_8         0x00000000  // TX FIFO <= 1/8 full
#define UART_IFLS_TX2_8         0x00000001  // TX FIFO <= 1/4 full
#define UART_IFLS_TX4_8         0x00000002  // TX FIFO <= 1/2 full (default)
#define UART_IFLS_TX6_8         0x00000003  // TX FIFO <= 3/4 full
#define UART_IFLS_TX7_8         0x00000004  // TX FIFO <= 7/8 full

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
//*****************************************************************************
#define UART_IM_DMATXIM         0x00020000  // Transmit DMA Interrupt Mask
#define UART_IM_DMARXIM         0x00010000  // Receive DMA Interrupt Mask
#define UART_IM_9BITIM          0x00001000  // 9-Bit Mode Interrupt Mask
#define UART_IM_EOTIM           0x00000800  // End of Transmission Interrupt
                                            // Mask
#define UART_IM_OEIM            0x00000400  // UART Overrun Error Interrupt
                                            // Mask
#define UART_IM_BEIM            0x00000200  // UART Break Error Interrupt Mask
#define UART_IM_PEIM            0x00000100  // UART Parity Error Interrupt Mask
#define UART_IM_FEIM            0x00000080  // UART Framing Error Interrupt
                                            // Mask
#define UART_IM_RTIM            0x00000040  // UART Receive Time-Out Interrupt
                                            // Mask
#define UART_IM_TXIM            0x00000020  // UART Transmit Interrupt Mask
#define UART_IM_RXIM            0x00000010  // UART Receive Interrupt Mask
#define UART_IM_DSRMIM          0x00000008  // UART Data Set Ready Modem
                                            // Interrupt Mask
#define UART_IM_DCDMIM          0x00000004  // UART Data Carrier Detect Modem
                                            // Interrupt Mask
#define UART_IM_CTSMIM          0x00000002  // UART Clear to Send Modem
                                            // Interrupt Mask
#define UART_IM_RIMIM           0x00000001  // UART Ring Indicator Modem
                                            // Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
#define UART_RIS_DMATXRIS       0x00020000  // Transmit DMA Raw Interrupt
                                            // Status
#define UART_RIS_DMARXRIS       0x00010000  // Receive DMA Raw Interrupt Status
#define UART_RIS_9BITRIS        0x00001000  // 9-Bit Mode Raw Interrupt Status
#define UART_RIS_EOTRIS         0x00000800  // End of Transmission Raw
                                            // Interrupt Status
#define UART_RIS_OERIS          0x00000400  // UART Overrun Error Raw Interrupt
                                            // Status
#define UART_RIS_BERIS          0x00000200  // UART Break Error Raw Interrupt
                                            // Status
#define UART_RIS_PERIS          0x00000100  // UART Parity Error Raw Interrupt
                                            // Status
#define UART_RIS_FERIS          0x00000080  // UART Framing Error Raw Interrupt
                                            // Status
#define UART_RIS_RTRIS          0x00000040  // UART Receive Time-Out Raw
                                            // Interrupt Status
#define UART_RIS_TXRIS          0x00000020  // UART Transmit Raw Interrupt
                                            // Status
#define UART_RIS_RXRIS          0x00000010  // UART Receive Raw Interrupt
                                            // Status
#define UART_RIS_DSRRIS         0x00000008  // UART Data Set Ready Modem Raw
                                            // Interrupt Status
#define UART_RIS_DCDRIS         0x00000004  // UART Data Carrier Detect Modem
                                            // Raw Interrupt Status
#define UART_RIS_CTSRIS         0x00000002  // UART Clear to Send Modem Raw
                                            // Interrupt Status
#define UART_RIS_RIRIS          0x00000001  // UART Ring Indicator Modem Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
#define UART_MIS_DMATXMIS       0x00020000  // Transmit DMA Masked Interrupt
                                            // Status
#define UART_MIS_DMARXMIS       0x00010000  // Receive DMA Masked Interrupt
                                            // Status
#define UART_MIS_9BITMIS        0x00001000  // 9-Bit Mode Masked Interrupt
                                            // Status
#define UART_MIS_EOTMIS         0x00000800  // End of Transmission Masked
                                            // Interrupt Status
#define UART_MIS_OEMIS          0x00000400  // UART Overrun Error Masked
                                            // Interrupt Status
#define UART_MIS_BEMIS          0x00000200  // UART Break Error Masked
                                            // Interrupt Status
#define UART_MIS_PEMIS          0x00000100  // UART Parity Error Masked
                                            // Interrupt Status
#define UART_MIS_FEMIS          0x00000080  // UART Framing Error Masked
                                            // Interrupt Status
#define UART_MIS_RTMIS          0x00000040  // UART Receive Time-Out Masked
                                            // Interrupt Status
#define UART_MIS_TXMIS          0x00000020  // UART Transmit Masked Interrupt
                                            // Status
#define UART_MIS_RXMIS          0x00000010  // UART Receive Masked Interrupt
                                            // Status
#define UART_MIS_DSRMIS         0x00000008  // UART Data Set Ready Modem Masked
                                            // Interrupt Status
#define UART_MIS_DCDMIS         0x00000004  // UART Data Carrier Detect Modem
                                            // Masked Interrupt Status
#define UART_MIS_CTSMIS         0x00000002  // UART Clear to Send Modem Masked
                                            // Interrupt Status
#define UART_MIS_RIMIS          0x00000001  // UART Ring Indicator Modem Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
#define UART_ICR_DMATXIC        0x00020000  // Transmit DMA Interrupt Clear
#define UART_ICR_DMARXIC        0x00010000  // Receive DMA Interrupt Clear
#define UART_ICR_9BITIC         0x00001000  // 9-Bit Mode Interrupt Clear
#define UART_ICR_EOTIC          0x00000800  // End of Transmission Interrupt
                                            // Clear
#define UART_ICR_OEIC           0x00000400  // Overrun Error Interrupt Clear
#define UART_ICR_BEIC           0x00000200  // Break Error Interrupt Clear
#define UART_ICR_PEIC           0x00000100  // Parity Error Interrupt Clear
#define UART_ICR_FEIC           0x00000080  // Framing Error Interrupt Clear
#define UART_ICR_RTIC           0x00000040  // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear
#define UART_ICR_DSRMIC         0x00000008  // UART Data Set Ready Modem
                                            // Interrupt Clear
#define UART_ICR_DCDMIC         0x00000004  // UART Data Carrier Detect Modem
                                            // Interrupt Clear
#define UART_ICR_CTSMIC         0x00000002  // UART Clear to Send Modem
                                            // Interrupt Clear
#define UART_ICR_RIMIC          0x00000001  // UART Ring Indicator Modem
                                            // Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
#define UART_DMACTL_DMAERR      0x00000004  // DMA on Error
#define UART_DMACTL_TXDMAE      0x00000002  // Transmit DMA Enable
#define UART_DMACTL_RXDMAE      0x00000001  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
//*****************************************************************************
#define UART_9BITADDR_9BITEN    0x00008000  // Enable 9-Bit Mode
#define UART_9BITADDR_ADDR_M    0x000000FF  // Self Address for 9-Bit Mode
#define UART_9BITADDR_ADDR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
//*****************************************************************************
#define UART_9BITAMASK_MASK_M   0x000000FF  // Self Address Mask for 9-Bit Mode
#define UART_9BITAMASK_MASK_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
//*****************************************************************************
#define UART_PP_MSE             0x00000008  // Modem Support Extended
#define UART_PP_MS              0x00000004  // Modem Support
#define UART_PP_NB              0x00000002  // 9-Bit Support
#define UART_PP_SC              0x00000001  // Smart Card Support

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
//*****************************************************************************
#define UART_CC_CS_M            0x0000000F  // UART Baud Clock Source
#define UART_CC_CS_SYSCLK       0x00000000  // System clock (based on clock
                                            // source and divisor factor)
#define UART_CC_CS_PIOSC        0x00000005  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
//*****************************************************************************
#define I2C_MSA_SA_M            0x000000FE  // I2C Slave Address
#define I2C_MSA_RS              0x00000001  // Receive not send
#define I2C_MSA_SA_S            1

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
//*****************************************************************************
#define I2C_MCS_ACTDMARX        0x80000000  // DMA RX Active Status
#define I2C_MCS_ACTDMATX        0x40000000  // DMA TX Active Status
#define I2C_MCS_CLKTO           0x00000080  // Clock Timeout Error
#define I2C_MCS_BURST           0x00000040  // Burst Enable
#define I2C_MCS_BUSBSY          0x00000040  // Bus Busy
#define I2C_MCS_IDLE            0x00000020  // I2C Idle
#define I2C_MCS_QCMD            0x00000020  // Quick Command
#define I2C_MCS_ARBLST          0x00000010  // Arbitration Lost
#define I2C_MCS_HS              0x00000010  // High-Speed Enable
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
//*****************************************************************************
#define I2C_MDR_DATA_M          0x000000FF  // This byte contains the data
                                            // transferred during a transaction
#define I2C_MDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
//*****************************************************************************
#define I2C_MTPR_PULSEL_M       0x00070000  // Glitch Suppression Pulse Width
#define I2C_MTPR_PULSEL_BYPASS  0x00000000  // Bypass
#define I2C_MTPR_PULSEL_1       0x00010000  // 1 clock
#define I2C_MTPR_PULSEL_2       0x00020000  // 2 clocks
#define I2C_MTPR_PULSEL_3       0x00030000  // 3 clocks
#define I2C_MTPR_PULSEL_4       0x00040000  // 4 clocks
#define I2C_MTPR_PULSEL_8       0x00050000  // 8 clocks
#define I2C_MTPR_PULSEL_16      0x00060000  // 16 clocks
#define I2C_MTPR_PULSEL_31      0x00070000  // 31 clocks
#define I2C_MTPR_HS             0x00000080  // High-Speed Enable
#define I2C_MTPR_TPR_M          0x0000007F  // Timer Period
#define I2C_MTPR_TPR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
#define I2C_MIMR_RXFFIM         0x00000800  // Receive FIFO Full Interrupt Mask
#define I2C_MIMR_TXFEIM         0x00000400  // Transmit FIFO Empty Interrupt
                                            // Mask
#define I2C_MIMR_RXIM           0x00000200  // Receive FIFO Request Interrupt
                                            // Mask
#define I2C_MIMR_TXIM           0x00000100  // Transmit FIFO Request Interrupt
                                            // Mask
#define I2C_MIMR_ARBLOSTIM      0x00000080  // Arbitration Lost Interrupt Mask
#define I2C_MIMR_STOPIM         0x00000040  // STOP Detection Interrupt Mask
#define I2C_MIMR_STARTIM        0x00000020  // START Detection Interrupt Mask
#define I2C_MIMR_NACKIM         0x00000010  // Address/Data NACK Interrupt Mask
#define I2C_MIMR_DMATXIM        0x00000008  // Transmit DMA Interrupt Mask
#define I2C_MIMR_DMARXIM        0x00000004  // Receive DMA Interrupt Mask
#define I2C_MIMR_CLKIM          0x00000002  // Clock Timeout Interrupt Mask
#define I2C_MIMR_IM             0x00000001  // Master Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
#define I2C_MRIS_RXFFRIS        0x00000800  // Receive FIFO Full Raw Interrupt
                                            // Status
#define I2C_MRIS_TXFERIS        0x00000400  // Transmit FIFO Empty Raw
                                            // Interrupt Status
#define I2C_MRIS_RXRIS          0x00000200  // Receive FIFO Request Raw
                                            // Interrupt Status
#define I2C_MRIS_TXRIS          0x00000100  // Transmit Request Raw Interrupt
                                            // Status
#define I2C_MRIS_ARBLOSTRIS     0x00000080  // Arbitration Lost Raw Interrupt
                                            // Status
#define I2C_MRIS_STOPRIS        0x00000040  // STOP Detection Raw Interrupt
                                            // Status
#define I2C_MRIS_STARTRIS       0x00000020  // START Detection Raw Interrupt
                                            // Status
#define I2C_MRIS_NACKRIS        0x00000010  // Address/Data NACK Raw Interrupt
                                            // Status
#define I2C_MRIS_DMATXRIS       0x00000008  // Transmit DMA Raw Interrupt
                                            // Status
#define I2C_MRIS_DMARXRIS       0x00000004  // Receive DMA Raw Interrupt Status
#define I2C_MRIS_CLKRIS         0x00000002  // Clock Timeout Raw Interrupt
                                            // Status
#define I2C_MRIS_RIS            0x00000001  // Master Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
#define I2C_MMIS_RXFFMIS        0x00000800  // Receive FIFO Full Interrupt Mask
#define I2C_MMIS_TXFEMIS        0x00000400  // Transmit FIFO Empty Interrupt
                                            // Mask
#define I2C_MMIS_RXMIS          0x00000200  // Receive FIFO Request Interrupt
                                            // Mask
#define I2C_MMIS_TXMIS          0x00000100  // Transmit Request Interrupt Mask
#define I2C_MMIS_ARBLOSTMIS     0x00000080  // Arbitration Lost Interrupt Mask
#define I2C_MMIS_STOPMIS        0x00000040  // STOP Detection Interrupt Mask
#define I2C_MMIS_STARTMIS       0x00000020  // START Detection Interrupt Mask
#define I2C_MMIS_NACKMIS        0x00000010  // Address/Data NACK Interrupt Mask
#define I2C_MMIS_DMATXMIS       0x00000008  // Transmit DMA Interrupt Status
#define I2C_MMIS_DMARXMIS       0x00000004  // Receive DMA Interrupt Status
#define I2C_MMIS_CLKMIS         0x00000002  // Clock Timeout Masked Interrupt
                                            // Status
#define I2C_MMIS_MIS            0x00000001  // Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
#define I2C_MICR_RXFFIC         0x00000800  // Receive FIFO Full Interrupt
                                            // Clear
#define I2C_MICR_TXFEIC         0x00000400  // Transmit FIFO Empty Interrupt
                                            // Clear
#define I2C_MICR_RXIC           0x00000200  // Receive FIFO Request Interrupt
                                            // Clear
#define I2C_MICR_TXIC           0x00000100  // Transmit FIFO Request Interrupt
                                            // Clear
#define I2C_MICR_ARBLOSTIC      0x00000080  // Arbitration Lost Interrupt Clear
#define I2C_MICR_STOPIC         0x00000040  // STOP Detection Interrupt Clear
#define I2C_MICR_STARTIC        0x00000020  // START Detection Interrupt Clear
#define I2C_MICR_NACKIC         0x00000010  // Address/Data NACK Interrupt
                                            // Clear
#define I2C_MICR_DMATXIC        0x00000008  // Transmit DMA Interrupt Clear
#define I2C_MICR_DMARXIC        0x00000004  // Receive DMA Interrupt Clear
#define I2C_MICR_CLKIC          0x00000002  // Clock Timeout Interrupt Clear
#define I2C_MICR_IC             0x00000001  // Master Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
#define I2C_MCR_SFE             0x00000020  // I2C Slave Function Enable
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define I2C_MCR_LPBK            0x00000001  // I2C Loopback

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCLKOCNT register.
//
//*****************************************************************************
#define I2C_MCLKOCNT_CNTL_M     0x000000FF  // I2C Master Count
#define I2C_MCLKOCNT_CNTL_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBMON register.
//
//*****************************************************************************
#define I2C_MBMON_SDA           0x00000002  // I2C SDA Status
#define I2C_MBMON_SCL           0x00000001  // I2C SCL Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBLEN register.
//
//*****************************************************************************
#define I2C_MBLEN_CNTL_M        0x000000FF  // I2C Burst Length
#define I2C_MBLEN_CNTL_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBCNT register.
//
//*****************************************************************************
#define I2C_MBCNT_CNTL_M        0x000000FF  // I2C Master Burst Count
#define I2C_MBCNT_CNTL_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
//*****************************************************************************
#define I2C_SOAR_OAR_M          0x0000007F  // I2C Slave Own Address
#define I2C_SOAR_OAR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
//*****************************************************************************
#define I2C_SCSR_ACTDMARX       0x80000000  // DMA RX Active Status
#define I2C_SCSR_ACTDMATX       0x40000000  // DMA TX Active Status
#define I2C_SCSR_QCMDRW         0x00000020  // Quick Command Read / Write
#define I2C_SCSR_QCMDST         0x00000010  // Quick Command Status
#define I2C_SCSR_OAR2SEL        0x00000008  // OAR2 Address Matched
#define I2C_SCSR_FBR            0x00000004  // First Byte Received
#define I2C_SCSR_RXFIFO         0x00000004  // RX FIFO Enable
#define I2C_SCSR_TXFIFO         0x00000002  // TX FIFO Enable
#define I2C_SCSR_TREQ           0x00000002  // Transmit Request
#define I2C_SCSR_DA             0x00000001  // Device Active
#define I2C_SCSR_RREQ           0x00000001  // Receive Request

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
//*****************************************************************************
#define I2C_SDR_DATA_M          0x000000FF  // Data for Transfer
#define I2C_SDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
//*****************************************************************************
#define I2C_SIMR_RXFFIM         0x00000100  // Receive FIFO Full Interrupt Mask
#define I2C_SIMR_TXFEIM         0x00000080  // Transmit FIFO Empty Interrupt
                                            // Mask
#define I2C_SIMR_RXIM           0x00000040  // Receive FIFO Request Interrupt
                                            // Mask
#define I2C_SIMR_TXIM           0x00000020  // Transmit FIFO Request Interrupt
                                            // Mask
#define I2C_SIMR_DMATXIM        0x00000010  // Transmit DMA Interrupt Mask
#define I2C_SIMR_DMARXIM        0x00000008  // Receive DMA Interrupt Mask
#define I2C_SIMR_STOPIM         0x00000004  // Stop Condition Interrupt Mask
#define I2C_SIMR_STARTIM        0x00000002  // Start Condition Interrupt Mask
#define I2C_SIMR_DATAIM         0x00000001  // Data Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
#define I2C_SRIS_RXFFRIS        0x00000100  // Receive FIFO Full Raw Interrupt
                                            // Status
#define I2C_SRIS_TXFERIS        0x00000080  // Transmit FIFO Empty Raw
                                            // Interrupt Status
#define I2C_SRIS_RXRIS          0x00000040  // Receive FIFO Request Raw
                                            // Interrupt Status
#define I2C_SRIS_TXRIS          0x00000020  // Transmit Request Raw Interrupt
                                            // Status
#define I2C_SRIS_DMATXRIS       0x00000010  // Transmit DMA Raw Interrupt
                                            // Status
#define I2C_SRIS_DMARXRIS       0x00000008  // Receive DMA Raw Interrupt Status
#define I2C_SRIS_STOPRIS        0x00000004  // Stop Condition Raw Interrupt
                                            // Status
#define I2C_SRIS_STARTRIS       0x00000002  // Start Condition Raw Interrupt
                                            // Status
#define I2C_SRIS_DATARIS        0x00000001  // Data Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
//*****************************************************************************
#define I2C_SMIS_RXFFMIS        0x00000100  // Receive FIFO Full Interrupt Mask
#define I2C_SMIS_TXFEMIS        0x00000080  // Transmit FIFO Empty Interrupt
                                            // Mask
#define I2C_SMIS_RXMIS          0x00000040  // Receive FIFO Request Interrupt
                                            // Mask
#define I2C_SMIS_TXMIS          0x00000020  // Transmit FIFO Request Interrupt
                                            // Mask
#define I2C_SMIS_DMATXMIS       0x00000010  // Transmit DMA Masked Interrupt
                                            // Status
#define I2C_SMIS_DMARXMIS       0x00000008  // Receive DMA Masked Interrupt
                                            // Status
#define I2C_SMIS_STOPMIS        0x00000004  // Stop Condition Masked Interrupt
                                            // Status
#define I2C_SMIS_STARTMIS       0x00000002  // Start Condition Masked Interrupt
                                            // Status
#define I2C_SMIS_DATAMIS        0x00000001  // Data Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
//*****************************************************************************
#define I2C_SICR_RXFFIC         0x00000100  // Receive FIFO Full Interrupt Mask
#define I2C_SICR_TXFEIC         0x00000080  // Transmit FIFO Empty Interrupt
                                            // Mask
#define I2C_SICR_RXIC           0x00000040  // Receive Request Interrupt Mask
#define I2C_SICR_TXIC           0x00000020  // Transmit Request Interrupt Mask
#define I2C_SICR_DMATXIC        0x00000010  // Transmit DMA Interrupt Clear
#define I2C_SICR_DMARXIC        0x00000008  // Receive DMA Interrupt Clear
#define I2C_SICR_STOPIC         0x00000004  // Stop Condition Interrupt Clear
#define I2C_SICR_STARTIC        0x00000002  // Start Condition Interrupt Clear
#define I2C_SICR_DATAIC         0x00000001  // Data Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR2 register.
//
//*****************************************************************************
#define I2C_SOAR2_OAR2EN        0x00000080  // I2C Slave Own Address 2 Enable
#define I2C_SOAR2_OAR2_M        0x0000007F  // I2C Slave Own Address 2
#define I2C_SOAR2_OAR2_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SACKCTL register.
//
//*****************************************************************************
#define I2C_SACKCTL_ACKOVAL     0x00000002  // I2C Slave ACK Override Value
#define I2C_SACKCTL_ACKOEN      0x00000001  // I2C Slave ACK Override Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFODATA register.
//
//*****************************************************************************
#define I2C_FIFODATA_DATA_M     0x000000FF  // I2C TX FIFO Write Data Byte
#define I2C_FIFODATA_DATA_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFOCTL register.
//
//*****************************************************************************
#define I2C_FIFOCTL_RXASGNMT    0x80000000  // RX Control Assignment
#define I2C_FIFOCTL_RXFLUSH     0x40000000  // RX FIFO Flush
#define I2C_FIFOCTL_DMARXENA    0x20000000  // DMA RX Channel Enable
#define I2C_FIFOCTL_RXTRIG_M    0x00070000  // RX FIFO Trigger
#define I2C_FIFOCTL_TXASGNMT    0x00008000  // TX Control Assignment
#define I2C_FIFOCTL_TXFLUSH     0x00004000  // TX FIFO Flush
#define I2C_FIFOCTL_DMATXENA    0x00002000  // DMA TX Channel Enable
#define I2C_FIFOCTL_TXTRIG_M    0x00000007  // TX FIFO Trigger
#define I2C_FIFOCTL_RXTRIG_S    16
#define I2C_FIFOCTL_TXTRIG_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_FIFOSTATUS
// register.
//
//*****************************************************************************
#define I2C_FIFOSTATUS_RXABVTRIG                                              \
                                0x00040000  // RX FIFO Above Trigger Level
#define I2C_FIFOSTATUS_RXFF     0x00020000  // RX FIFO Full
#define I2C_FIFOSTATUS_RXFE     0x00010000  // RX FIFO Empty
#define I2C_FIFOSTATUS_TXBLWTRIG                                              \
                                0x00000004  // TX FIFO Below Trigger Level
#define I2C_FIFOSTATUS_TXFF     0x00000002  // TX FIFO Full
#define I2C_FIFOSTATUS_TXFE     0x00000001  // TX FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PP register.
//
//*****************************************************************************
#define I2C_PP_HS               0x00000001  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PC register.
//
//*****************************************************************************
#define I2C_PC_HS               0x00000001  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CTL register.
//
//*****************************************************************************
#define PWM_CTL_GLOBALSYNC3     0x00000008  // Update PWM Generator 3
#define PWM_CTL_GLOBALSYNC2     0x00000004  // Update PWM Generator 2
#define PWM_CTL_GLOBALSYNC1     0x00000002  // Update PWM Generator 1
#define PWM_CTL_GLOBALSYNC0     0x00000001  // Update PWM Generator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_SYNC register.
//
//*****************************************************************************
#define PWM_SYNC_SYNC3          0x00000008  // Reset Generator 3 Counter
#define PWM_SYNC_SYNC2          0x00000004  // Reset Generator 2 Counter
#define PWM_SYNC_SYNC1          0x00000002  // Reset Generator 1 Counter
#define PWM_SYNC_SYNC0          0x00000001  // Reset Generator 0 Counter

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENABLE register.
//
//*****************************************************************************
#define PWM_ENABLE_PWM7EN       0x00000080  // MnPWM7 Output Enable
#define PWM_ENABLE_PWM6EN       0x00000040  // MnPWM6 Output Enable
#define PWM_ENABLE_PWM5EN       0x00000020  // MnPWM5 Output Enable
#define PWM_ENABLE_PWM4EN       0x00000010  // MnPWM4 Output Enable
#define PWM_ENABLE_PWM3EN       0x00000008  // MnPWM3 Output Enable
#define PWM_ENABLE_PWM2EN       0x00000004  // MnPWM2 Output Enable
#define PWM_ENABLE_PWM1EN       0x00000002  // MnPWM1 Output Enable
#define PWM_ENABLE_PWM0EN       0x00000001  // MnPWM0 Output Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INVERT register.
//
//*****************************************************************************
#define PWM_INVERT_PWM7INV      0x00000080  // Invert MnPWM7 Signal
#define PWM_INVERT_PWM6INV      0x00000040  // Invert MnPWM6 Signal
#define PWM_INVERT_PWM5INV      0x00000020  // Invert MnPWM5 Signal
#define PWM_INVERT_PWM4INV      0x00000010  // Invert MnPWM4 Signal
#define PWM_INVERT_PWM3INV      0x00000008  // Invert MnPWM3 Signal
#define PWM_INVERT_PWM2INV      0x00000004  // Invert MnPWM2 Signal
#define PWM_INVERT_PWM1INV      0x00000002  // Invert MnPWM1 Signal
#define PWM_INVERT_PWM0INV      0x00000001  // Invert MnPWM0 Signal

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULT register.
//
//*****************************************************************************
#define PWM_FAULT_FAULT7        0x00000080  // MnPWM7 Fault
#define PWM_FAULT_FAULT6        0x00000040  // MnPWM6 Fault
#define PWM_FAULT_FAULT5        0x00000020  // MnPWM5 Fault
#define PWM_FAULT_FAULT4        0x00000010  // MnPWM4 Fault
#define PWM_FAULT_FAULT3        0x00000008  // MnPWM3 Fault
#define PWM_FAULT_FAULT2        0x00000004  // MnPWM2 Fault
#define PWM_FAULT_FAULT1        0x00000002  // MnPWM1 Fault
#define PWM_FAULT_FAULT0        0x00000001  // MnPWM0 Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INTEN register.
//
//*****************************************************************************
#define PWM_INTEN_INTFAULT3     0x00080000  // Interrupt Fault 3
#define PWM_INTEN_INTFAULT2     0x00040000  // Interrupt Fault 2
#define PWM_INTEN_INTFAULT1     0x00020000  // Interrupt Fault 1
#define PWM_INTEN_INTFAULT0     0x00010000  // Interrupt Fault 0
#define PWM_INTEN_INTPWM3       0x00000008  // PWM3 Interrupt Enable
#define PWM_INTEN_INTPWM2       0x00000004  // PWM2 Interrupt Enable
#define PWM_INTEN_INTPWM1       0x00000002  // PWM1 Interrupt Enable
#define PWM_INTEN_INTPWM0       0x00000001  // PWM0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_RIS register.
//
//*****************************************************************************
#define PWM_RIS_INTFAULT3       0x00080000  // Interrupt Fault PWM 3
#define PWM_RIS_INTFAULT2       0x00040000  // Interrupt Fault PWM 2
#define PWM_RIS_INTFAULT1       0x00020000  // Interrupt Fault PWM 1
#define PWM_RIS_INTFAULT0       0x00010000  // Interrupt Fault PWM 0
#define PWM_RIS_INTPWM3         0x00000008  // PWM3 Interrupt Asserted
#define PWM_RIS_INTPWM2         0x00000004  // PWM2 Interrupt Asserted
#define PWM_RIS_INTPWM1         0x00000002  // PWM1 Interrupt Asserted
#define PWM_RIS_INTPWM0         0x00000001  // PWM0 Interrupt Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ISC register.
//
//*****************************************************************************
#define PWM_ISC_INTFAULT3       0x00080000  // FAULT3 Interrupt Asserted
#define PWM_ISC_INTFAULT2       0x00040000  // FAULT2 Interrupt Asserted
#define PWM_ISC_INTFAULT1       0x00020000  // FAULT1 Interrupt Asserted
#define PWM_ISC_INTFAULT0       0x00010000  // FAULT0 Interrupt Asserted
#define PWM_ISC_INTPWM3         0x00000008  // PWM3 Interrupt Status
#define PWM_ISC_INTPWM2         0x00000004  // PWM2 Interrupt Status
#define PWM_ISC_INTPWM1         0x00000002  // PWM1 Interrupt Status
#define PWM_ISC_INTPWM0         0x00000001  // PWM0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_STATUS register.
//
//*****************************************************************************
#define PWM_STATUS_FAULT3       0x00000008  // Generator 3 Fault Status
#define PWM_STATUS_FAULT2       0x00000004  // Generator 2 Fault Status
#define PWM_STATUS_FAULT1       0x00000002  // Generator 1 Fault Status
#define PWM_STATUS_FAULT0       0x00000001  // Generator 0 Fault Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULTVAL register.
//
//*****************************************************************************
#define PWM_FAULTVAL_PWM7       0x00000080  // MnPWM7 Fault Value
#define PWM_FAULTVAL_PWM6       0x00000040  // MnPWM6 Fault Value
#define PWM_FAULTVAL_PWM5       0x00000020  // MnPWM5 Fault Value
#define PWM_FAULTVAL_PWM4       0x00000010  // MnPWM4 Fault Value
#define PWM_FAULTVAL_PWM3       0x00000008  // MnPWM3 Fault Value
#define PWM_FAULTVAL_PWM2       0x00000004  // MnPWM2 Fault Value
#define PWM_FAULTVAL_PWM1       0x00000002  // MnPWM1 Fault Value
#define PWM_FAULTVAL_PWM0       0x00000001  // MnPWM0 Fault Value

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENUPD register.
//
//*****************************************************************************
#define PWM_ENUPD_ENUPD7_M      0x0000C000  // MnPWM7 Enable Update Mode
#define PWM_ENUPD_ENUPD7_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD7_LSYNC  0x00008000  // Locally Synchronized
#define PWM_ENUPD_ENUPD7_GSYNC  0x0000C000  // Globally Synchronized
#define PWM_ENUPD_ENUPD6_M      0x00003000  // MnPWM6 Enable Update Mode
#define PWM_ENUPD_ENUPD6_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD6_LSYNC  0x00002000  // Locally Synchronized
#define PWM_ENUPD_ENUPD6_GSYNC  0x00003000  // Globally Synchronized
#define PWM_ENUPD_ENUPD5_M      0x00000C00  // MnPWM5 Enable Update Mode
#define PWM_ENUPD_ENUPD5_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD5_LSYNC  0x00000800  // Locally Synchronized
#define PWM_ENUPD_ENUPD5_GSYNC  0x00000C00  // Globally Synchronized
#define PWM_ENUPD_ENUPD4_M      0x00000300  // MnPWM4 Enable Update Mode
#define PWM_ENUPD_ENUPD4_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD4_LSYNC  0x00000200  // Locally Synchronized
#define PWM_ENUPD_ENUPD4_GSYNC  0x00000300  // Globally Synchronized
#define PWM_ENUPD_ENUPD3_M      0x000000C0  // MnPWM3 Enable Update Mode
#define PWM_ENUPD_ENUPD3_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD3_LSYNC  0x00000080  // Locally Synchronized
#define PWM_ENUPD_ENUPD3_GSYNC  0x000000C0  // Globally Synchronized
#define PWM_ENUPD_ENUPD2_M      0x00000030  // MnPWM2 Enable Update Mode
#define PWM_ENUPD_ENUPD2_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD2_LSYNC  0x00000020  // Locally Synchronized
#define PWM_ENUPD_ENUPD2_GSYNC  0x00000030  // Globally Synchronized
#define PWM_ENUPD_ENUPD1_M      0x0000000C  // MnPWM1 Enable Update Mode
#define PWM_ENUPD_ENUPD1_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD1_LSYNC  0x00000008  // Locally Synchronized
#define PWM_ENUPD_ENUPD1_GSYNC  0x0000000C  // Globally Synchronized
#define PWM_ENUPD_ENUPD0_M      0x00000003  // MnPWM0 Enable Update Mode
#define PWM_ENUPD_ENUPD0_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD0_LSYNC  0x00000002  // Locally Synchronized
#define PWM_ENUPD_ENUPD0_GSYNC  0x00000003  // Globally Synchronized

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CTL register.
//
//*****************************************************************************
#define PWM_0_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_0_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_0_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_0_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_0_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_0_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_0_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_0_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_0_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_0_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_0_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_0_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_0_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_0_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_0_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_0_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_0_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_0_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_0_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_0_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_0_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_0_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_0_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_0_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_0_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_0_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_0_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_0_CTL_MODE          0x00000002  // Counter Mode
#define PWM_0_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_INTEN register.
//
//*****************************************************************************
#define PWM_0_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
                                            // Down
#define PWM_0_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_0_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
                                            // Down
#define PWM_0_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_0_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_0_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_0_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
                                            // Down
#define PWM_0_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
                                            // Up
#define PWM_0_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
                                            // Down
#define PWM_0_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
                                            // Up
#define PWM_0_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_0_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_RIS register.
//
//*****************************************************************************
#define PWM_0_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
                                            // Status
#define PWM_0_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_0_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
                                            // Status
#define PWM_0_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_0_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_0_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_ISC register.
//
//*****************************************************************************
#define PWM_0_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_0_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_0_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_0_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_0_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_0_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_LOAD register.
//
//*****************************************************************************
#define PWM_0_LOAD_M            0x0000FFFF  // Counter Load Value
#define PWM_0_LOAD_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_COUNT register.
//
//*****************************************************************************
#define PWM_0_COUNT_M           0x0000FFFF  // Counter Value
#define PWM_0_COUNT_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPA register.
//
//*****************************************************************************
#define PWM_0_CMPA_M            0x0000FFFF  // Comparator A Value
#define PWM_0_CMPA_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPB register.
//
//*****************************************************************************
#define PWM_0_CMPB_M            0x0000FFFF  // Comparator B Value
#define PWM_0_CMPB_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENA register.
//
//*****************************************************************************
#define PWM_0_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_0_GENA_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_0_GENA_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_0_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_0_GENA_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_0_GENA_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_0_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_0_GENA_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_0_GENA_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_0_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_0_GENA_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_0_GENA_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_0_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_0_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_0_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_0_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_0_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_0_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_0_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_0_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_0_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENB register.
//
//*****************************************************************************
#define PWM_0_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_0_GENB_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_0_GENB_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_0_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_0_GENB_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_0_GENB_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_0_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_0_GENB_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_0_GENB_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_0_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_0_GENB_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_0_GENB_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_0_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_0_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_0_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_0_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_0_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_0_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_0_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_0_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_0_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBCTL register.
//
//*****************************************************************************
#define PWM_0_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBRISE register.
//
//*****************************************************************************
#define PWM_0_DBRISE_DELAY_M    0x00000FFF  // Dead-Band Rise Delay
#define PWM_0_DBRISE_DELAY_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBFALL register.
//
//*****************************************************************************
#define PWM_0_DBFALL_DELAY_M    0x00000FFF  // Dead-Band Fall Delay
#define PWM_0_DBFALL_DELAY_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_0_FLTSRC0_FAULT3    0x00000008  // Fault3 Input
#define PWM_0_FLTSRC0_FAULT2    0x00000004  // Fault2 Input
#define PWM_0_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_0_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_0_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_0_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_0_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_0_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_0_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_0_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_0_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_0_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_0_MINFLTPER_M       0x0000FFFF  // Minimum Fault Period
#define PWM_0_MINFLTPER_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CTL register.
//
//*****************************************************************************
#define PWM_1_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_1_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_1_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_1_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_1_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_1_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_1_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_1_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_1_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_1_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_1_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_1_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_1_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_1_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_1_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_1_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_1_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_1_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_1_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_1_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_1_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_1_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_1_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_1_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_1_CTL_CMPAUPD       0x00000010
tm4c1294ncpdt.h
Exibindo tm4c1294ncpdt.h…