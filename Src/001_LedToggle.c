/*
 * 001_LedToggle.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Murali V
 *
 *
 */

#include "../Drivers/Inc/stm32F407xx.h"

void delay()
{
	for(uint32_t i =0; i<=500000/2; i++);
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

	//GPIO_PeriClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
