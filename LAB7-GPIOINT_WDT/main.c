/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 14/12/08 11:49a $
 * @brief    Show the usage of GPIO interrupt function.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#include "Scankey.h"
#include "wdh.h"
#include "Scankey.h" 


#define PLL_CLOCK   50000000
volatile uint8_t g_u8IsWDTTimeoutINT = 0;
int state = 0;

/**
 * @brief       GPIO PA/PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA/PB default IRQ, declared in startup_NUC100Series.s.
 */
void GPAB_IRQHandler(void)
{
    /* To check if PB.3 interrupt occurred */

    if(GPIO_GET_INT_FLAG(PA, BIT2))
    {
        GPIO_CLR_INT_FLAG(PA, BIT2);
        //printf("Change!!!\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PA, PB interrupts */
        PA->ISRC = PA->ISRC;
        //PB->ISRC = PB->ISRC;
        printf("Un-expected interrupts.\n");
    }
}
void WDT_IRQHandler(void)
{
    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsWDTTimeoutINT = 1;
        //WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_1026CLK, TRUE, FALSE);
        printf("WDT time-out interrupt occurred.\n");
        if(state==0)
        {
            printf("No problem!!!\n");
            WDT_RESET_COUNTER();
        }else if(state ==1)
        {

            printf("Alarm!!!~~~Reset!!!\n");
        }
    }
}



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC((CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk));

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady((CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);    
        
    /* Select WDT module clock source */
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, NULL);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    //printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    //printf("+------------------------------------------------+\n");
    //printf("|    GPIO PB.3 and PE.5 Interrupt Sample Code    |\n");
    //printf("+------------------------------------------------+\n\n");
    if(WDT_GET_RESET_FLAG() == 1)
    {
        /* Use PA.0 to check time-out period time */
        GPIO_SetMode(PA, 0, GPIO_PMD_OUTPUT);
        PA0 = 1;
        WDT_CLEAR_RESET_FLAG();        
        //while(1);
    }
    printf("Start LAB7\n");
    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
  
    /* Configure PB.3 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT3, GPIO_PMD_INPUT);
    GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
    GPIO_EnableInt(PA, 2, GPIO_INT_FALLING);
    //GPIO_EnableInt(PA, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPAB_IRQn);

    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT2);
    
    SYS_UnlockReg();
    g_u8IsWDTTimeoutINT = 0;
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_1026CLK, TRUE, FALSE);
    WDT_EnableInt();
    NVIC_EnableIRQ(WDT_IRQn);
    
    while(1){
        
        if(ScanKey()==1)
        {
            if(state == 0) state += 1;
            else if( state == 1) state = 0;
        }
        if(state == 0)
        {
            printf("Safe!\n");
            CLK_SysTickDelay(2000000);
        }else if(state == 1)
        {
            printf("Alarm!!!\n");
            CLK_SysTickDelay(2000000);
        }
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
