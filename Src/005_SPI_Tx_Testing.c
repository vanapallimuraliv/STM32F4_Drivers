/*
 * 005_SPI_Tx_Testing.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Murali
 */


#include "stm32f407xx.h"
#include <string.h>
void GPIO_Intitialization(void);
void SPI2_Initialization(void);

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
		GPIO_Init(&SPIPin2);

		// NSS
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPIPin2); */
}

void SPI2_Initialization(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_config.SPI_DeviceMode = SPI_MASTER;
	SPI2Handle.SPI_config.SPI_BusConfig = SPI_BUS_CFG_FULLD;
	SPI2Handle.SPI_config.SPI_Speed = SPI_BAUD_RATE4; // speed 8Mbps;
	SPI2Handle.SPI_config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_config.SPI_CPHA = SPI_CPHA0;
	SPI2Handle.SPI_config.SPI_CPOL = SPI_CPOL0;
	SPI2Handle.SPI_config.SPI_SSM = SPI_SSM_SW;

	SPI_Init(&SPI2Handle);
}

int main()
{
	char user_data[] = "Hello world";
	GPIO_Initialization();
	SPI2_Initialization();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralCtl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralCtl(SPI2, DISABLE);

	while(1);
	return 0;
}
