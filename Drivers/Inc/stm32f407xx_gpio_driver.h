/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Aug 4, 2025
 *      Author: Murali V
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "Stm32f407xx.h"

// this is the configuration setting for GPIO pins
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtrl;
	uint8_t GPIO_PinOutType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;



typedef struct
{
	// pointer to hold base address of the GPIO Peripheral
	GPIO_RegDef_t *pGPIOx; // Base address of gpio pin to enable GPIOA,B,C,D..
	GPIO_PinConfig_t  GPIO_PinConfig;  // it has a holding a respective pin configuration settings

}GPIO_Handle_t;

// GPIO possible Mode
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANLOG			3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

// GPIO SPEED Possibles
#define GPIO_LOW_SPEED			0
#define GPIO_MID_SPEED			1
#define GPIO_HIGH_SPEED			2
#define GPIO_VHIGH_SPEED		3

// GPIO OUT TYPE Possibles
#define GPIO_OUT_PUPL			0
#define GPIO_OUT_OP_DRN			1

// GPIO PULL UP AND PULL DOWN Possibles
#define GPIO_NO_PUPD			0
#define GPIO_PU_UP				1
#define GPIO_PU_DN				2


#define GPIO_PIN_NO_0 			0
#define GPIO_PIN_NO_1 			1
#define GPIO_PIN_NO_2 			2
#define GPIO_PIN_NO_3 			3
#define GPIO_PIN_NO_4 			4
#define GPIO_PIN_NO_5 			5
#define GPIO_PIN_NO_6 			6
#define GPIO_PIN_NO_7 			7
#define GPIO_PIN_NO_8 			8
#define GPIO_PIN_NO_9 			9
#define GPIO_PIN_NO_10 			10
#define GPIO_PIN_NO_11 			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13 			13
#define GPIO_PIN_NO_14 			14
#define GPIO_PIN_NO_15 			15

// External interrupt IRQ positioning
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI5_9				23
#define IRQ_NO_EXTI10_15			40

// IRQ PRIORITY
#define IRQ_PR_NO_1					1
#define IRQ_PR_NO_2					2
#define IRQ_PR_NO_3					3
#define IRQ_PR_NO_4					4
#define IRQ_PR_NO_5					5
#define IRQ_PR_NO_6					6
#define IRQ_PR_NO_7					7
#define IRQ_PR_NO_8					8
#define IRQ_PR_NO_9					9
#define IRQ_PR_NO_10				10
#define IRQ_PR_NO_11				11
#define IRQ_PR_NO_12				12
#define IRQ_PR_NO_13				13
#define IRQ_PR_NO_14				14
#define IRQ_PR_NO_15				15

/**********************************************************************************
 * API for GPIO supported driver
 *
 ***********************************************************************************/
// clock enable

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis);


//initial and De initial
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// input and output configuration
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//interrupt handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandler(uint8_t IRQNumber);






#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
