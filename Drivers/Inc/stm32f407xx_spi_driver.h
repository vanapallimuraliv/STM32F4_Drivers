/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Aug 6, 2025
 *      Author: Murali V
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32F407xx.h"


/*************************************************************************************
 * some useful macros
 *
 * ***********************************************************************************/
// Modes for communication

#define SPI_MASTER					1
#define SPI_SLAVE					0

// SPI SPEED
#define SPI_BAUD_RATE2				0
#define SPI_BAUD_RATE4				1
#define SPI_BAUD_RATE8				2
#define SPI_BAUD_RATE16				3
#define SPI_BAUD_RATE32				4
#define SPI_BAUD_RATE64				5
#define SPI_BAUD_RATE128			6
#define SPI_BAUD_RATE256			7

#define SPI_BUS_CFG_FULLD				1
#define SPI_BUS_CFG_HALFD				2
#define SPI_BUS_CFG_S_TXONLY			3
#define SPI_BUS_CFG_S_RXONLY			4

#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1


#define SPI_CPHA0					0
#define SPI_CPHA1					1

#define SPI_CPOL0					0
#define SPI_CPOL1					1

#define SPI_SSM_HW					0
#define SPI_SSM_SW					1

#define SPI_TXE_FLAG 				(1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG 				(1<< SPI_SR_RXNE)
#define SPI_BUSY_FLAG 				(1<< SPI_SR_BSY)

// SPI Busy states
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

// SPI Event
#define SPI_EVENT_TX_COMPLT		1
#define SPI_EVENT_RX_COMPLT		2
#define SPI_EVENT_OVR_COMPLT	3

//Interrupt
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51
#define IRQ_NO_SPI4					84
#define IRQ_NO_SPI5					85
#define IRQ_NO_SPI6					86

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;
}SPI_config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_config_t SPI_config;
	uint8_t *pTxBuffer; 	// To store the app. Tx Buffer Address Global variable
	uint8_t *pRxBuffer; 	// To store the app. Tx Buffer Address
	uint32_t TxLen;			// To store the app. Tx Lenght
	uint32_t RxLen;			// To store the app. Tx Lenght
	uint8_t TxState;			// To store the app. Tx State
	uint8_t RxState;			// To store the app. Tx State
}SPI_Handle_t;

// API Implementation

void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDis);


//initial and De initial
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// send and receive data
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint16_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint16_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint16_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint16_t Len);

// Interrupt configuration
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pSPI_Handle);

//other API'S
void SPI_PeripheralCtl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagStatus);
void SPI_ClearOverFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle);
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle);


// Application specific
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPI_Handle, uint8_t Event);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
