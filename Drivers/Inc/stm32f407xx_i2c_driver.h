/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Aug 12, 2025
 *      Author: vanap
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_



typedef struct
{
	uint8_t I2C_SclSpeed;
	uint8_t I2C_DeviceAddr;
	uint8_t I2C_AckCtrl;
	uint8_t I2C_FmDutyCycle;
}I2C_config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_config_t I2C_config;
}I2C_Handle_t;

// I2C_SclSpeed
// Enable or Disable
#define I2C_SPEED_SM			100000
#define I2C_SPEED_FM4K			400000
#define I2C_SPEED_FM2K			200000

//I2C_DeviceAddr


//I2C_AckCtrl
#define I2C_ACK_ENABLE		ENABLE
#define I2C_ACK_DISABLE		DISABLE

//I2C_FmDutyCycle;
#define I2C_FM_DUTY_2			0
#define I2C_SM_DUTY16_9			1

// Status Flags
#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_TXE			(1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RxNE)

#define I2C_FLAG_BUSY			(1 << I2C_SR2_BUSY)
#define I2C_FLAG_DUALF			(1 << I2C_SR2_DUALF)
#define I2C_FLAG_MSL			(1 << I2C_SR2_MSL)
#define I2C_FLAG_PEC			(1 << I2C_SR2_PEC)
#define I2C_FLAG_TRA			(1 << I2C_SR2_TRA)



// API Implementation
void I2C_peripheralCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// I2C inti and Deinti
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxBufferu, int8_t Addr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint8_t Addr);
void I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRxBufferu, int8_t Addr);

///other api
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagStatus);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
