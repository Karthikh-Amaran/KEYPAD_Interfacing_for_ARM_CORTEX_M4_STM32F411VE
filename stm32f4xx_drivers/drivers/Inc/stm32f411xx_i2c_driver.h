/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: 16-Jan-2023
 *      Author: Karthikh Amaran
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_
#include "stm32f411xx.h"
/* For the repeated start conditions */
#define I2C_DISABLE_SR    RESET /* Disabling the repeated start*/
#define I2C_ENABLE_SR       SET /* Enabling the Repeated start*/
/*
 *  Configuration Structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;       /* Options for this register @I2C_SCLSpeed Similarly for all the registers*/
	uint8_t  I2C_DeviceAddress;  // Mentioned by the User
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;
/*
 *  Handle Structure for I2Cx Peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t      *pTxBuffer; /* !<To store app. Tx Buffer Address> */
	uint8_t      *pRxBuffer; /* !<To store app. Rx Buffer Address> */
	uint32_t     TxLen;      /* !<To store Tx Len> */
	uint32_t     RxLen;      /* !<To store Rx Len> */
	uint8_t      TxRxState;  /* !<To store communication state> */
	uint8_t      DevAddr;    /* !<To store slave/device address> */
	uint32_t     RxSize;     /* !<To store Rx Size> */
	uint8_t      Sr;         /* !<To store repeated start value> */
}I2C_Handle_t;


/*
 * I2C Application states
 */
#define I2C_READY       0
#define I2C_BUSY_IN_RX  1
#define I2C_BUSY_IN_TX  2
#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM    100000
#define I2C_SCL_SPEED_FM4K  400000
#define I2C_SCL_SPEED_FM2K  200000
/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0
/*
 *  @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

/*
 *  Flags in the I2C status Register
 */

#define I2C_FLAG_SB      	 (1<<I2C_SR1_SB)
#define I2C_FLAG_ADDR   	 (1<<I2C_SR1_ADDR)
#define I2C_FLAG_BTF    	 (1<<I2C_SR1_BTF)
#define I2C_FLAG_ADD10  	 (1<<I2C_SR1_ADD10)
#define I2C_FLAG_STOPF   	 (1<<I2C_SR1_STOPF)
#define I2C_FLAG_RXNE  	     (1<<I2C_SR1_RXNE)
#define I2C_FLAG_TXE  	     (1<<I2C_SR1_TXE)
#define I2C_FLAG_BERR  	     (1<<I2C_SR1_BERR)
#define I2C_FLAG_ARLO   	 (1<<I2C_SR1_ARLO)
#define I2C_FLAG_AF  	     (1<<I2C_SR1_AF)
#define I2C_FLAG_OVR  	     (1<<I2C_SR1_OVR)
#define I2C_FLAG_PECERR  	 (1<<I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT 	 (1<<I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT 	 (1<<I2C_SR1_SMBALERT)
/**************************************************************************/

//peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);

// Init and Deinit
void I2C_Init(I2C_Handle_t *pI2CHandle);

//De-Initialization basically means resetting the bits
// We have a Special register in RCC Called Peripheral Reset enable register to perform this reset
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
// Data Send and Receive for master mode
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);

//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

// TO ENABLE PE BIT
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi); // Enabling or the Enable bit of the Peripheral
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
// Data Send and Receive for master mode
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 *   Application Call Back
 */
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


