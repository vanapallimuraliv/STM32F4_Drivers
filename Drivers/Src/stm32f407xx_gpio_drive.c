/*
 * stm32f407xx_gpio_drive.c
 *
 *  Created on: Aug 4, 2025
 *      Author: vanap
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

/******************************************************************************
 *@function 				- PeriClkCtrl
 *@Brief:					- used to Enable the GPIO clock for respective PORT
 *							- i.e. GPIOA,B,C,D,E,G,H,I
 *
 *@Parameter[in]:			-*pGPIOx = Pointer and we are accessing the Base address
 *@Parameter[in]:			- EnorDis = used to Enable / Disable
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis)
{

	if(EnorDis == ENABLE)
	{
		if(pGPIOx == GPIOA)  // GPIO_RegDef_t* pGPIO = (GPIO_RegDef_t*)GPIOA_BASEADDR
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)  // We are doing this GPIO_RegDef_t* pGPIO = (GPIO_RegDef_t*)GPIOA_BASEADDR
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/******************************************************************************
 *@function 				-GPIO_Init
 *
 *@Brief:					-used to enable all types of settings like Pin Mode,
 *							 output Speed, Output Type,PinPuPdCtrl
 *
 *@Parameter[in]:			-*pGPIO_Handle
 *@return:					-None
 *
 *@note:
 *
 * ****************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	//enable the peripheral clock
	GPIO_PeriClkCtrl(pGPIO_Handle->pGPIOx, ENABLE);

	// temp register
	uint32_t temp=0;

	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANLOG)
	{
		pGPIO_Handle->pGPIOx->MODAR &= ~(3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
		 // 2 << 2 * 3
		temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		pGPIO_Handle->pGPIOx->MODAR |= temp; // setting
		temp =0;
	}else
	{
			// 1.For Interrupt
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure EXIT Falling EDGE
			EXTI->FTSR  |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			// clearing the EXTI Raising EDGE
			EXTI->RSTR  &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure EXIT Raising EDGE
			EXTI->RSTR  |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			// clearing the EXTI FALLING EDGE
			EXTI->FTSR  &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure EXIT Raising and Falling EDGE
			EXTI->RSTR  |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			// clearing the EXTI FALLING EDGE
			EXTI->FTSR  |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2. configure the GPIO Port selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) /4 ; // 16 /4 and
		// 2 % 4
		uint8_t temp2 = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIO_Handle->pGPIOx);
		SYSCFG->EXTICR[temp1] |= portcode << (4 * temp2);

		//3.Enable the EXIT interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;
	temp =0;

	pGPIO_Handle->pGPIOx->OTYPER &= ~(1 << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinOutType << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER |= temp;
	temp =0;

	pGPIO_Handle->pGPIOx->PUPDR &= ~(3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->PUPDR |= temp;
	temp =0;

	 if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	 {
		 uint8_t temp1 = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) / 8;  // 12/4 =
		 uint8_t temp2 = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) % 8; // 12 % 8 = 3 % 2
		 pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );
		 pGPIO_Handle->pGPIOx->AFR[temp1] |= pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2);
	 }

}

/******************************************************************************
 *@function 				- GPIO_DeInit
 *@Brief:					- Resting the clock to stop GPIO
 *
 *@Parameter[in]:			-*pGPIOx
 *
 *@return:					- None
 *
 *@note: By set(1) and resting(0) the GPIOx AHB1RSTR
 *
 * ****************************************************************************/


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_PCLK_RSTR();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_PCLK_RSTR();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_PCLK_RSTR();
	}else if(pGPIOx == GPIOD)
	{
		GPIOC_PCLK_RSTR();
	}else if(pGPIOx == GPIOE)
	{
		GPIOD_PCLK_RSTR();
	}else if(pGPIOx == GPIOF)
	{
		GPIOE_PCLK_RSTR();
	}else if(pGPIOx == GPIOG)
	{
		GPIOF_PCLK_RSTR();
	}else if(pGPIOx == GPIOH)
	{
		GPIOG_PCLK_RSTR();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_PCLK_RSTR();
	}
}

/******************************************************************************
 *@function 				-GPIO_ReadFromInputPin
 *@Brief:					- Bit Extracting using right shifting respective
 *							  pinNumber and applying AND gate.
 *@Parameter[in]:			-*pGPIOx
 *@Parameter[in]:			-PinNumber
 *
 *@return:					-value
 *@note:
 *
 * ****************************************************************************/


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value =0;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber ) & 0x00000001);
	return value;
}

/******************************************************************************
 *@function 				-GPIO_ReadFromInputPin
 *@Brief:					- Read all port i.e. 16 pins
 *@Parameter[in]:			-*pGPIOx
 *
 *@return:					-value
 *
 *@note:
 *
 * ****************************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value =0;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


/******************************************************************************
 *@function 				-GPIO_WriteToOutputPin
 *@Brief:					- if value is set then Output drain is 1 of the
 *							  respective PinNumber
 *
 *@Parameter[in]:			-*pGPIOx
 *@Parameter[in]:			-PinNumber
 *@Parameter[in]:			-Value
 *@return:					- None
 *
 *@note:
 *
 * ****************************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/******************************************************************************
 *@function 				- GPIO_WriteToOutputPort
 *@Brief:					-
 *@Parameter[in]:			-*pGPIOx
 *@Parameter[in]:			- Value
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note: no of pins can be set up as output
 *
 * ****************************************************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/******************************************************************************
 *@function 				- GPIO_ToggleOutputPin
 *@Brief:					- Applying the XOR to respective Pin number
 *
 *@Parameter[in]:			-*pGPIOx
 *@Parameter[in]:			- PinNumber
 *
 *@return:					-
 *
 *@note: XOR{^} :I/P | O/P
 *			   	   1 | 0
 *				   0 | 1
 * ****************************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= 1 << PinNumber; // doing XOR operation
}


/******************************************************************************
 *@function 				- GPIO_IRQInterruptConfig
 *@Brief:					-this function is used to set up the interrupt
 *							 [used Peripheral End Implementation]
 *
 *@Parameter[in]:			-IRQNumber
 *@Parameter[in]:			-EnorDi
 *@Parameter[in]:			-
 *@return:					- None
 *
 *@note:Interrupt setting and clearing it.(M4 Processor end Implementation).
 *
 * ****************************************************************************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <= 31)
			{
				*NVIC_ICER0 |= (1 << IRQNumber);
			}else if(IRQNumber >= 32 && IRQNumber < 64)
			{
				*NVIC_ICER1 |= (1 << IRQNumber % 32);
			}else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				*NVIC_ICER2 |= (1 << IRQNumber % 64);
			}
	}
}


/******************************************************************************
 *@function 				- GPIO_IRQPriorityConfig
 *@Brief:					- this function is used to set up the interrupt priority
 *@Brief:
 *@Parameter[in]:			- IRQNumber
 *@Parameter[in]:			- IRQPriority
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note: M4 processor end Implementation
 *
 * ****************************************************************************/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber /4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t Shift_Amount = 8 * iprx_section + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << Shift_Amount);
	// giving the priority to Interrupt
}


/******************************************************************************
 *@function 				-GPIO_IRQHandler
 *@Brief:					-if Interrupt has be occurs PR is set then clear the PR
 *
 *@Parameter[in]:			-IRQNumber
 *
 *@return:					- None
 *
 *@note: Peripheral side implementation
 *
 * ****************************************************************************/

void GPIO_IRQHandler(uint8_t IRQNumber)
{
	// clear the PR register for corresponding pin number
	// if PR is set then clear the PR
	if(EXTI->PR & (1<< IRQNumber) )
	{
		// clear the PR
		EXTI->PR |= (1<< IRQNumber);
	}
}







