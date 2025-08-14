/*
 * 002_Switch_Led_toggle.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Murali V
 *
 */


#include "Stm32f407xx.h"

void delay()
{
	for(uint8_t i=0; i<50000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	GPIO_Handle_t Switch;
	Switch.pGPIOx = GPIOD;
	Switch.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	Switch.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Switch.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PUPL;
	Switch.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	Switch.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	GPIO_PeriClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&Switch);

	while(1)
	{
	if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_10) == 1)
	{
		delay();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	}
	}
}
