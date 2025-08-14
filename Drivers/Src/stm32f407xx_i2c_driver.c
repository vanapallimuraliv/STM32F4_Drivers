/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Aug 12, 2025
 *      Author: vanap
 */

#include "stm32f407xx.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);

uint16_t AHBPrescal[9] = {2,4,8,16,32,64,128,256,512};
uint8_t APBLPrescal[4]= {2,4,8,16};

uint32_t PLL_selectedsystem()
{
	return 0;
}
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
uint32_t RCC_GetPclkValue(void)
{
	uint32_t Pclk, systemClk, ahbp, PPRE1;
	uint8_t clkSrc;
	clkSrc = ((RCC->CFGR >> 2) & 0x3);
		if(clkSrc == 0)
		{
			systemClk = 16000000;
		}else if(clkSrc == 1)
		{
			systemClk = 8000000;
		}else if(clkSrc == 2)
		{
			systemClk = PLL_selectedsystem();
		}

	uint8_t temp;
	temp = ((RCC->CFGR >> 4 ) & 0xf);
	if(temp > 8 )
	{
		ahbp =1;
	}else
	{
		ahbp = AHBPrescal[temp-8];
	}

	 uint8_t temp1;
	 temp1 = ((RCC->CFGR >> 10) & 0x7);
	 if(temp1 > 4)
	 {
		 PPRE1 =1;
	 }else
	 {
		 PPRE1 =  APBLPrescal[temp1-4];
	 }

		Pclk = (systemClk / ahbp) / PPRE1;
	 return Pclk;
}
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1);
	pI2Cx->DR = slaveAddr;
}


/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummy_read = pI2Cx->SR1;
			dummy_read = pI2Cx->SR2;
	(void)dummy_read;
}

/******************************************************************************
 *@function 				- I2C_PeriClkCtrl
 *@Brief:					- used to Enable the SPI clock for respective
 *
 *
 *@Parameter[in]:			-*pI2Cx = Pointer and we are accessing the Base address
 *@Parameter[in]:			- EnorDis = used to Enable / Disable
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void I2C_peripheralCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else if(EnorDi == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}

// I2C inti and Deinti

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
void I2C_Init(I2C_Handle_t *pI2C_Handle)
{
	// Enable the peripheral clock control
	I2C_peripheralCtrl(pI2C_Handle->pI2Cx, ENABLE);
	// initialize the temp variable
	uint32_t tempReg =0;

	//
	tempReg |= (pI2C_Handle->I2C_config.I2C_AckCtrl << I2C_CR1_ACK);
	pI2C_Handle->pI2Cx->CR1 = tempReg;

	// config CR2 frequency
	tempReg =0;
	tempReg |= RCC_GetPclkValue() / 100000;
	pI2C_Handle->pI2Cx->CR2 = (tempReg & 0x3F);

	// Program the own Address
	tempReg |= (pI2C_Handle->I2C_config.I2C_DeviceAddr << I2C_OAR1_ADD0);
	tempReg |= (1<< 14);
	pI2C_Handle->pI2Cx->OAR1 = tempReg;

	uint16_t CCR_Value =0;
	tempReg =0;
	if(pI2C_Handle->I2C_config.I2C_SclSpeed == I2C_SPEED_SM)
	{
		// Set standard mode
		pI2C_Handle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS);
		CCR_Value = RCC_GetPclkValue() /(2 * pI2C_Handle->I2C_config.I2C_SclSpeed );
		tempReg |= (CCR_Value & 0xFFF);

	}else
	{
		tempReg |= (1 << I2C_CCR_FS);
		tempReg |=pI2C_Handle->I2C_config.I2C_FmDutyCycle << I2C_CCR_DUTY;

		if(pI2C_Handle->I2C_config.I2C_FmDutyCycle == I2C_FM_DUTY_2)
		{
			CCR_Value = (RCC_GetPclkValue() / (3 * pI2C_Handle->I2C_config.I2C_SclSpeed ));
		}else
		{
			CCR_Value = (RCC_GetPclkValue() / (25 * pI2C_Handle->I2C_config.I2C_SclSpeed));
		}
		tempReg |= CCR_Value;
	}
	pI2C_Handle->pI2Cx->CCR = tempReg;

}


/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_PCLK_RSTR();
	}else if(pI2Cx == I2C2)
	{
		I2C2_PCLK_RSTR();
	}else if(pI2Cx == I2C3)
	{
		I2C3_PCLK_RSTR();
	}
}


/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					-
 *
 *@note:
 *
 * ****************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// Generate a start bit
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);


	// confirm the start generation completed by checking SB FLAG in SR1
	while( ! (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB)));
	// note: until SB Clear SCL will be stretched (pulled to low)

	// send the address of the slave with R/W (0)
	I2C_ExecuteAddrPhase(pI2C_Handle->pI2Cx, SlaveAddr);

	// Confirm that address phase is completed by checking ADDR flag in SR1
	while( ! I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR));

	// Clear the ADDR Flag according to its software sequence
	I2C_ClearAddrFlag(pI2C_Handle->pI2Cx);

	// note: until ADDR Clear SCL will be stretched (pulled to low)

	// send a data until length be comes 0
	if(Len >0)
	{
		// Wait for TxE flag set
		while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE));
		pI2C_Handle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// when Lenght become 0 wait for TxE =1 and BTF =1 Before generating the STOP condition
	// Note: TxE =1 and BTF =1 mean That SR and DR are empty and next transmission should begin
	// when BTF =1; SCL Should be stretched (pulled to low).
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF));
	// Generate a STOP bit
	I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagStatus)
{
	if(pI2Cx->SR1 & FlagStatus)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}









