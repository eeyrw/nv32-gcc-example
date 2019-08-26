/**
  ******************************************************************************
  * @file      startup_stm32f0xx.s
  * @author    MCD Application Team
  * @version   V1.5.0
  * @date      05-December-2014
  * @brief     STM32F030 Devices vector table for RIDE7 toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Configure the system clock 
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M0 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

  .syntax unified
  .arch armv6-m
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss




  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  movs r1, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4

LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1
  cmp r2, r3
  bcc CopyDataInit
  ldr r2, =_sbss
  b LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
  movs r3, #0
  str  r3, [r2]
  adds r2, r2, #4


LoopFillZerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss

/* Call the clock system intitialization function.*/
    bl  SystemInit
    
/* Call the application's entry point.*/
  bl main
  
LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
    .section .isr_vector,"a",%progbits
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    _estack            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler                     /* Hard Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long   Reserved16_IRQHandler   /* Reserved interrupt 16 */
    .long   Reserved17_IRQHandler   /* Reserved interrupt 17 */
    .long   Reserved18_IRQHandler   /* Reserved interrupt 18 */
    .long   Reserved19_IRQHandler   /* Reserved interrupt 19 */
    .long   Reserved20_IRQHandler   /* Reserved interrupt 20 */
    .long   ETMRH_IRQHandler        /* ETMRH command complete/read collision interrupt */
    .long   LVD_LVW_IRQHandler      /* Low Voltage Detect, Low Voltage Warning */
    .long   IRQ_IRQHandler          /* External interrupt */
    .long   I2C0_IRQHandler         /* I2C0 interrupt */
    .long   Reserved25_IRQHandler   /* Reserved interrupt 25 */
    .long   SPI0_IRQHandler         /* SPI0 interrupt */
    .long   SPI1_IRQHandler         /* SPI1 interrupt */
    .long   UART0_IRQHandler        /* UART0 status/error interrupt */
    .long   UART1_IRQHandler        /* UART1 status/error interrupt */
    .long   UART2_IRQHandler        /* UART2 status/error interrupt */
    .long   ADC0_IRQHandler         /* ADC0 interrupt */
    .long   ACMP0_IRQHandler        /* CMP0 interrupt */
    .long   ETM0_IRQHandler         /* ETM0 Single interrupt vector for all sources */
    .long   ETM1_IRQHandler         /* ETM1 Single interrupt vector for all sources */
    .long   ETM2_IRQHandler         /* ETM2 Single interrupt vector for all sources */
    .long   RTC_IRQHandler          /* RTC overflow */
    .long   ACMP1_IRQHandler        /* ACMP1 interrupt */
    .long   PIT_CH0_IRQHandler      /* PIT CH0 overflow */
    .long   PIT_CH1_IRQHandler      /* PIT CH1 overflow */
    .long   KBI0_IRQHandler         /* Keyboard interrupt 0 */
    .long   KBI1_IRQHandler         /* Keyboard interrupt 1 */
    .long   Reserved42_IRQHandler   /* Reserved interrupt 42 */
    .long   ICS_IRQHandler          /* MCG interrupt */
    .long   Watchdog_IRQHandler     /* WDOG Interrupt */
    .long   Reserved45_IRQHandler   /* Reserved interrupt 45 */
    .long   Reserved46_IRQHandler   /* Reserved interrupt 46 */
    .long   Reserved47_IRQHandler   /* Reserved interrupt 47 */

    .size    __isr_vector, . - __isr_vector
 
    .text
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler     NMI_Handler
    def_default_handler     HardFault_Handler
    def_default_handler     SVC_Handler
    def_default_handler     PendSV_Handler
    def_default_handler     SysTick_Handler
    def_default_handler     Reserved16_IRQHandler
    def_default_handler     Reserved17_IRQHandler
    def_default_handler     Reserved18_IRQHandler
    def_default_handler     Reserved19_IRQHandler
    def_default_handler     Reserved20_IRQHandler
    def_default_handler     ETMRH_IRQHandler
    def_default_handler     LVD_LVW_IRQHandler
    def_default_handler     IRQ_IRQHandler
    def_default_handler     I2C0_IRQHandler
    def_default_handler     Reserved25_IRQHandler
    def_default_handler     SPI0_IRQHandler
    def_default_handler     SPI1_IRQHandler
    def_default_handler     UART0_IRQHandler
    def_default_handler     UART1_IRQHandler
    def_default_handler     UART2_IRQHandler
    def_default_handler     ADC0_IRQHandler
    def_default_handler     ACMP0_IRQHandler
    def_default_handler     ETM0_IRQHandler
    def_default_handler     ETM1_IRQHandler
    def_default_handler     ETM2_IRQHandler
    def_default_handler     RTC_IRQHandler
    def_default_handler     ACMP1_IRQHandler
    def_default_handler     PIT_CH0_IRQHandler
    def_default_handler     PIT_CH1_IRQHandler
    def_default_handler     KBI0_IRQHandler
    def_default_handler     KBI1_IRQHandler
    def_default_handler     Reserved42_IRQHandler
    def_default_handler     ICS_IRQHandler
    def_default_handler     Watchdog_IRQHandler
    def_default_handler     Reserved45_IRQHandler
    def_default_handler     Reserved46_IRQHandler
    def_default_handler     Reserved47_IRQHandler

    .weak   DEF_IRQHandler
    .set    DEF_IRQHandler, Default_Handler

/* Flash protection region, placed at 0x400 */
    .text
    .thumb
    .align 2
    .section .kinetis_flash_config_field,"a",%progbits
kinetis_flash_config:
    .long 0xffffffff
    .long 0xffffffff
    .long 0xffffffff
    .long 0xfffeffff

    .end

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

