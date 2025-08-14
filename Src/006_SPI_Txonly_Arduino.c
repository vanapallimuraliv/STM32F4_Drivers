/*
 * 006_SPI_Txonly_Arduino.c
 *
 *  Created on: Aug 11, 2025
 *      Author: vanap
 */


#include "stm32f407xx.h"
#include <string.h>
void GPIO_Intitialization(void);
void SPI2_Initialization(void);

void delay()
{
	for(uint32_t i =0; i<500000/2; i++);
}

/* PB14--> SPI2_MISO
 * PB15--> SPI2_MOSI
 * PB13--> SPI2_SCLK
 * PB12--> SPI2_NSS
 * ALT --5
 * */

void GPIO_Initialization(void)
{
	GPIO_Handle_t SPIPin2;
		SPIPin2.pGPIOx = GPIOB;
		SPIPin2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		SPIPin2.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		SPIPin2.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PUPL;
		SPIPin2.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
		SPIPin2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;



		// MOSI
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPin2);


		// SCLK
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPIPin2);
		// MISO
		/*SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		GPIO_Init(&SPIPin2); */

		// NSS
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPIPin2);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t Btn;
	Btn.pGPIOx = GPIOA;
	Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Btn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GPIO_Init(&Btn);
}
void SPI2_Initialization(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_config.SPI_DeviceMode = SPI_MASTER;
	SPI2Handle.SPI_config.SPI_BusConfig = SPI_BUS_CFG_FULLD;
	SPI2Handle.SPI_config.SPI_Speed = SPI_BAUD_RATE8; // speed 1Mbps;
	SPI2Handle.SPI_config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_config.SPI_CPHA = SPI_CPHA0;
	SPI2Handle.SPI_config.SPI_CPOL = SPI_CPOL0;
	SPI2Handle.SPI_config.SPI_SSM = SPI_SSM_HW;

	SPI_Init(&SPI2Handle);
}

int main()
{

	char user_data[12] = "Hello world";
	GPIO_Initialization();
	GPIO_ButtonInit();
	SPI2_Initialization();

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
		{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();
		SPI_PeripheralCtl(SPI2, ENABLE);

		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen, 1);

		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralCtl(SPI2, DISABLE);
		}
	return 0;
}
