/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/04/20 2:56p $
 * @brief
 *           Implement SPI Master loop back transfer.
 *           This sample code needs to connect SPI2_MISO0 pin and SPI2_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#define PLL_CLOCK           50000000


//uint32_t g_au32SourceData[10] = {0, 0xB2, 0xB4, 0xB6, 0xB3, 0xB5, 0xB7, 0x9E, 0x9F, 0xA0};
//int32_t g_au32ReusultData[10];
//int32_t tmp;


/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
void ADLX345_setup(void);
void registerWrite(char address,char value);
uint16_t RawDataX(void);
uint16_t RawDataY(void);
uint16_t RawDataZ(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    float x1,y1,z1,x,y,z;
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();
	  
	  /* Init ADLX345 */
	  ADLX345_setup(); 

 
    printf("\nStart");
    
		while(1){					 
				x1 = RawDataX();
			  y1 = RawDataY();
				z1 = RawDataZ();
			
				x = x1/65536;
				y = y1/65536;
				z = z1/65536;
				
		
			printf("\nx: %.2f, y: %.2f, z: %.2f", x,y,z);
		
		
				CLK_SysTickDelay(80000000); 
		}	

		//while(1);			
}

void SYS_Init(void)
{
    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Enable external 12 MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select HCLK as the clock source of SPI2 */
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL1_SPI2_S_HCLK, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable SPI2 peripheral clock */
    CLK_EnableModuleClock(SPI2_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Setup SPI2 multi-function pins */
    SYS->GPD_MFP = SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;
    SYS->ALT_MFP = SYS_ALT_MFP_PD0_SPI2_SS0 | SYS_ALT_MFP_PD1_SPI2_CLK | SYS_ALT_MFP_PD2_SPI2_MISO0 | SYS_ALT_MFP_PD3_SPI2_MOSI0;

    /* UPDate System Core Clock */
    /* User can use SystemCoreClockUPDate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2 MHz */
    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_3, 8, 2000000);
	  SPI_SetBusClock(SPI2, 2000000);
		SPI_DisableAutoSS(SPI2);
		SPI_SET_SS0_LOW(SPI2);
    /* Enable the automatic hardware slave select function. Select the SPI2_SS0 pin and configure as low-active. */
    /*SPI_EnableAutoSS(SPI2, SPI_SS0, SPI_SS_ACTIVE_LOW);*/
}
void ADLX345_setup(void){
	         
		/*POWER_CTL(0x2D): 0x08*/
		registerWrite(0x2D,0x08);
	
	  /*DATA_FORMAT(0x31): 0x0B*/
	  registerWrite(0x31,0x0B);
	
		/*FIFO_CTL(0x38): 0x80*/
	  registerWrite(0x38,0x80);
					 
		/*FIFO_CTL(0x38): 0x80*/
	  registerWrite(0x38,0x80);
					 
	     

           printf("ADXL  Init\n");
}
void registerWrite(char address,char value){
		SPI_WRITE_TX0(SPI2,address);
		SPI_TRIGGER(SPI2);
		while(SPI_IS_BUSY(SPI2));
		SPI_WRITE_TX0(SPI2,value);
		SPI_TRIGGER(SPI2);
		while(SPI_IS_BUSY(SPI2));
}

uint16_t RawDataX(void){
	
		uint8_t LoByte,HiByte;
	
		SPI_WRITE_TX0(SPI2, 0xB2);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    LoByte = SPI_READ_RX0(SPI2);
	
		SPI_WRITE_TX0(SPI2, 0xB3);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    HiByte = SPI_READ_RX0(SPI2);
		
		
		return((HiByte<<8) | LoByte);
}

uint16_t RawDataY(void){
	
		uint8_t LoByte,HiByte;
	
		SPI_WRITE_TX0(SPI2, 0xB4);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    LoByte = SPI_READ_RX0(SPI2);
	
		SPI_WRITE_TX0(SPI2, 0xB5);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    HiByte = SPI_READ_RX0(SPI2);
		
		
		return((HiByte<<8) | LoByte);
}

uint16_t RawDataZ(void){
	
		uint8_t LoByte,HiByte;
	
		SPI_WRITE_TX0(SPI2, 0xB6);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    LoByte = SPI_READ_RX0(SPI2);
	
		SPI_WRITE_TX0(SPI2, 0xB7);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    HiByte = SPI_READ_RX0(SPI2);
		
		
		return((HiByte<<8) | LoByte);
}
/*
uint16_t offsetX(void){
	
		uint8_t Byte,HiByte;
	
		SPI_WRITE_TX0(SPI2, 0x1E);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2));
    Byte = SPI_READ_RX0(SPI2);
	
		
		return Byte;
}*/

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
