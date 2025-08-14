/*
 * 003_Exter_Interrupt.c
 *
 *  Created on: Aug 5, 2025
 *      Author: vanap
 */


#include "Stm32f407xx.h"
void delay(void)
{
	for(uint32_t i =0; i<50000/2; i++);
}

int main()
{
	// configure Button
	GPIO_Handle_t Button,LED;
	memset(&Button,0,sizeof(Button));
	memset(&LED,0,sizeof(LED));
	Button.pGPIOx = GPIOD;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PUPL;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	Button.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PU_UP;
	GPIO_Init(&Button);

	// configure LED

	LED.pGPIOx = GPIOD;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PUPL;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	LED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_PeriClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&LED);

	// IRQ CONFGURATION
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI5_9,ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9,IRQ_PR_NO_15);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{

	GPIO_IRQHandler(GPIO_PIN_NO_5); // clear the interrupt
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
