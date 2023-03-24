/*
 * stm32f411xx_spi_drivers.h
 *
 *  Created on: 29-Dec-2022
 *      Author: Karthikh Amaran
 */

#ifndef INC_STM32F411XX_SPI_DRIVERS_H_
#define INC_STM32F411XX_SPI_DRIVERS_H_

#include "stm32f411xx.h"
/*
 *  Configuration Structure for SPIx Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;            /* Possible Values from @SPI_DEVICEMODE */
	uint8_t SPI_BusConfig;             /* Possible Values from @SPI_BUSCONFIG */
	uint8_t SPI_SclkSpeed;             /* Possible Values from @SPI_SCLKSPEED */
	uint8_t SPI_DFF;                   /* Possible Values from @SPI_DFF */
	uint8_t SPI_CPOL;                  /* Possible Values from @SPI_CPOL */
	uint8_t SPI_CPHA;                  /* Possible Values from @SPI_CPHA */
	uint8_t SPI_SSM;                   /* Possible Values from @SPI_SSM */
}SPI_Config_t;
// Handle Structure for SPIx Peripheral
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t      *pTxBuffer; /* To store the application Tx Buffer address */
	uint8_t		 *pRxBuffer; /* To store the app. Rx buffer address */
	uint32_t     TxLen;       /* To store Tx len */
	uint32_t     RxLen;       /* To store Rx Len */
	uint8_t      TxState;    /* To store Tx State */
	uint8_t      RxState;    /* To store Rx State */

}SPI_Handle_t;
/*
 *  SPI Application states
 */
#define SPI_READY        0
#define SPI_BUSY_IN_RX   1
#define SPI_BUSY_IN_TX   2
/*
 *  Possible SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_ERR   3
#define SPI_EVENT_CRC_ERR   4
/********************************************************************************/
/*******************USER USABLE MACRO IN APPLICATION IMPLEMENTATION*****************************************/
/********************************************************************************/
/********************************************************************************/

// @SPI_DEVICEMODE
#define SPI_DEVICE_MODE_MASTER 		   1
#define SPI_DEVICE_MODE_SLAVE          0
// @SPI_BUSCONFIG
#define SPI_BUS_CONFIG_FD 			   1
#define SPI_BUS_CONFIG_HD  			   2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY  3 (FOR ACHIEVING THIS JUST CONFIGURE THE BAORD IN FD MODE THEN REMOVE THE RX PIN)
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY  3
// @SPI_SCLKSPEED
#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7
// @SPI_DFF
#define SPI_DFF_8BITS 				   0
#define SPI_DFF_16BITS                 1
// @SPI_CPOL
#define SPI_CPOL_HIGH				   1
#define SPI_CPOL_LOW 				   0
// @SPI_CPHA
#define SPI_CPHA_HIGH				   1
#define SPI_CPHA_LOW 				   0
// @SPI_SSM
#define SPI_SSM_EN					   1
#define SPI_SSM_DI					   0

// SPI RELATED STATUS FLAGS
#define SPI_TXE_FLAG                   (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG                  (1<<SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG                (1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG                   (1<<SPI_SR_UDR)
#define SPI_CRCERR_FLAG                (1<<SPI_SR_CRCERR)
#define SPI_MODF_FLAG                  (1<<SPI_SR_MODF)
#define SPI_OVR_FLAG                   (1<<SPI_SR_OVR)
#define SPI_BSY_FLAG                   (1<<SPI_SR_BSY)
#define SPI_FRE_FLAG                   (1<<SPI_SR_FRE)



//peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
// Init and Deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
//De-Initialization basically means resetting the bits
// We have a Special register in RCC Called Peripheral Reset enable register to perform this reset
void SPI_DeInit(SPI_RegDef_t *pSPIx);
// Data Send and Receive
// In SPI CAN and other few protocols we have two types of Communication protocols
// Namely, 1. Blocking (Without Interrupt) 2. Non Blocking (Interrupt Based) 3. DMA Based
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);

//IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 *   Application Call Back
 */
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif /* INC_STM32F411XX_SPI_DRIVERS_H_ */
