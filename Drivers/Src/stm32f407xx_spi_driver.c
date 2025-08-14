/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Murali V
 */

#include "stm32f407xx.h"

static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPI_Handle);
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPI_Handle);
static void SPI_OverError_IT_Handle(SPI_Handle_t *pSPI_Handle);

/******************************************************************************
 *@function 				- SPI_PeriClkCtrl
 *@Brief:					- used to Enable the SPI clock for respective
 *
 *
 *@Parameter[in]:			-*pSPIx = Pointer and we are accessing the Base address
 *@Parameter[in]:			- EnorDis = used to Enable / Disable
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}else if(pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}else if(pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
	}else if(EnorDis == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}else if(pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}else if(pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
	}
}


/******************************************************************************
 *@function 				-SPI_Init
 *@Brief:					-
 *
 *@Parameter[in]:			-*pSPI_Handle
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
//initial and De initial
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	// PClk enable
	SPI_PeriClkCtrl(pSPI_Handle->pSPIx, ENABLE);

	uint32_t tempReg =0;
	tempReg = pSPI_Handle->SPI_config.SPI_DeviceMode << SPI_CR1_MSTR;

	if(pSPI_Handle->SPI_config.SPI_BusConfig == SPI_BUS_CFG_FULLD)
	{
		//bidirectional data mode set
		tempReg &= ~(1<< SPI_CR1_BIDI);

	}else if(pSPI_Handle->SPI_config.SPI_BusConfig == SPI_BUS_CFG_HALFD)
	{
		//bidirectional data mode reset
		tempReg |= (1<< SPI_CR1_BIDI);

	}else if(pSPI_Handle->SPI_config.SPI_BusConfig == SPI_BUS_CFG_S_RXONLY)
	{
		//bidirectional data mode set
		tempReg &= ~(1<< SPI_CR1_BIDI);
		//receive-only mode)
		tempReg |= (1<< SPI_CR1_RXONLY);
	}
	tempReg |= (pSPI_Handle->SPI_config.SPI_DFF << SPI_CR1_DFF);

	tempReg |= (pSPI_Handle->SPI_config.SPI_CPOL << SPI_CR1_CPOL);

	tempReg |= (pSPI_Handle->SPI_config.SPI_CPHA << SPI_CR1_CPHA);

	tempReg |= (pSPI_Handle->SPI_config.SPI_Speed << SPI_CR1_CLK_SPEED);

	tempReg |= (pSPI_Handle->SPI_config.SPI_SSM << SPI_CR1_SSM);

	pSPI_Handle->pSPIx->CR1 = tempReg;
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_RSTR();
	}else if(pSPIx == SPI2)
	{
		SPI2_PCLK_RSTR();
	}else if(pSPIx == SPI3)
	{
		SPI3_PCLK_RSTR();
	}else if(pSPIx == SPI4)
	{
		SPI4_PCLK_RSTR();
	}else if(pSPIx == SPI5)
	{
		SPI5_PCLK_RSTR();
	}else if(pSPIx == SPI6)
	{
		SPI6_PCLK_RSTR();
	}
}


/******************************************************************************
 *@function 				- SPI_SendData
 *@Brief:					- This is used to send a data from the master to slave
 *
 *@Parameter[in]:			-pSPIx
 *@Parameter[in]:			-pTxBuffer
 *@Parameter[in]:			-Len
 *@return:					- none
 *
 *@note: This is blocking call
 *
 * ****************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint16_t Len)
{
	while(Len > 0)
	{
		// wait for TX buffer empty
		if(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == FLAG_RESET )
		{
			// check the DFF =0 (8 BITS) DFF == 1 (16 Bits)
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				pSPIx->DR = *((uint16_t*)pTxBuffer);
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			//DFF =0 (8 BITS)
			}else
			{
				pSPIx->DR = *pTxBuffer;
				Len--;
				pTxBuffer++;
			}
		}
	}
}

/******************************************************************************
 *@function 				-SPI_ReceiveData
 *@Brief:					-
 *
 *@Parameter[in]:			-pSPIx
 *@Parameter[in]:			-pRxBuffer
 *@Parameter[in]:			-Len
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint16_t Len)
{
	while(Len > 0)
	{
		if(SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE) == FLAG_RESET)
		{
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				// 16 Bit data
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				pRxBuffer++;
			}else
			{
				//8 Bit Data
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}
		}
	}
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint16_t Len)
{
	uint8_t State = pSPI_Handle->TxState;
	if( State !=  SPI_BUSY_IN_TX)
	{
	//1. save the Tx buffer Address and len info in the globle variable
	pSPI_Handle->pTxBuffer = pTxBuffer;
	pSPI_Handle->TxLen = Len;


	//2. Mark the SPI State as Busy in transmission so that
	//No OTHER code can take over until transmission is over
	pSPI_Handle->TxState = SPI_BUSY_IN_TX;

	pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return State;
	//4.Data transmission will be handled by the ISR Code
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint16_t Len)
{
	uint8_t State = pSPI_Handle->RxState;
		if( State !=  SPI_BUSY_IN_RX)
		{
		//1. save the Tx buffer Address and len info in the globle variable
		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = Len;


		//2. Mark the SPI State as Busy in transmission so that
		//No OTHER code can take over until transmission is over
		pSPI_Handle->RxState = SPI_BUSY_IN_RX;

		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		}

		return State;
}

// Interrupt configuration
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 32 && IRQNumber <= 64 )
		{
			*NVIC_ISER1 |= (1 << IRQNumber);
		}else if(IRQNumber > 64 && IRQNumber <= 96 )
		{
			*NVIC_ISER2 |= (1 << IRQNumber);
		}
	}else if(EnorDi == 	DISABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 32 && IRQNumber <= 64 )
		{
			*NVIC_ISER1 |= (1 << IRQNumber);
		}else if(IRQNumber > 64 && IRQNumber <= 96 )
		{
			*NVIC_ISER2 |= (1 << IRQNumber);
		}
	}
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t Shift_Amount = 8 * iprx_section + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << Shift_Amount);
	// giving the priority to Interrupt
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_IRQHandler(SPI_Handle_t *pSPI_Handle)
{
	uint8_t temp1, temp2;
	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_TXE); // status register status
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE); // Interrupt status

	//
	if(temp1 && temp2)
	{
		SPI_TXE_IT_Handle(pSPI_Handle);
	}
	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_RXNE); // status register status
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE); // Interrupt status

	//
	if(temp1 && temp2)
	{
		SPI_RXNE_IT_Handle(pSPI_Handle);
	}
	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_OVR); // status register status
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE); // Interrupt status

	if(temp1 && temp2)
	{
		SPI_OverError_IT_Handle(pSPI_Handle);
	}

}

//SPI Other API's
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagStatus)
{
	if(pSPIx->SR & FlagStatus)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPI_Handle)
{
	if(pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		pSPI_Handle->pSPIx->DR = *((uint16_t*)pSPI_Handle->pTxBuffer);
		pSPI_Handle->TxLen--;
		pSPI_Handle->TxLen--;
	}else
	{
		pSPI_Handle->pSPIx->DR = *(pSPI_Handle->pTxBuffer);
		pSPI_Handle->TxLen--;
	}

	if( !pSPI_Handle->TxLen)
	{
		//This prevents interrupt from setting up of the txe flag
		// After completion of data transfer  turn off the communication
		SPI_CloseTransmission(pSPI_Handle);
	}
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPI_Handle)
{
	if(pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		 *((uint16_t*)pSPI_Handle->pRxBuffer) = pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
		pSPI_Handle->RxLen--;
	}else
	{
		*(pSPI_Handle->pTxBuffer) = pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
	}

	if( !pSPI_Handle->RxLen)
	{
		//This prevents interrupt from setting up of the txe flag
		// After completion of receive data turn off the communication
		SPI_CloseReception(pSPI_Handle);
	}
}
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
static void SPI_OverError_IT_Handle(SPI_Handle_t *pSPI_Handle)
{
	uint8_t temp =0;
	// clear the over run flag
	if(pSPI_Handle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPI_Handle->pSPIx->DR;
		temp = pSPI_Handle->pSPIx->SR;
	}
	(void)temp;
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_ERRIE);
	//call the application
	SPI_ApplicationEventCallBack(pSPI_Handle, SPI_EVENT_OVR_COMPLT);
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/

void SPI_ClearOverFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/

void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR2 &= ~(SPI_CR2_TXEIE);
	pSPI_Handle->pTxBuffer = NULL;
	pSPI_Handle->TxLen =0;
	pSPI_Handle->TxState = SPI_READY;
	SPI_ApplicationEventCallBack(pSPI_Handle, SPI_EVENT_TX_COMPLT);
}
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/

void SPI_CloseReception(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE);
	pSPI_Handle->pRxBuffer = NULL;
	pSPI_Handle->RxLen =0;
	pSPI_Handle->RxState = SPI_READY;
	SPI_ApplicationEventCallBack(pSPI_Handle, SPI_EVENT_RX_COMPLT);
}
/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_PeripheralCtl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_CLK_SPE);

	}else if(EnorDi == DISABLE)
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_CLK_SPE);
	}
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);

		}else if(EnorDi == DISABLE)
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

/******************************************************************************
 *@function 				-
 *@Brief:					-
 *
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@Parameter[in]:			-
 *@return:					- none
 *
 *@note:
 *
 * ****************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);

		}else if(EnorDi == DISABLE)
		{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}
_weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPI_Handle, uint8_t Event)
{

}
