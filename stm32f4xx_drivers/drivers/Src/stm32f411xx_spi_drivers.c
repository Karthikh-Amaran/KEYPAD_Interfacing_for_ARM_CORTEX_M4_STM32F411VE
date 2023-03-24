/*
 * stm32f411xx_spi_drivers.c
 *
 *  Created on: 29-Dec-2022
 *      Author: Karthikh Amaran
 */

#include "stm32f411xx_spi_drivers.h"
// static keyword is o make a function or a variable private to its file only
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
/*
 *  peripheral Clock Setup
 */
/*
    * @Fn            		: GPIO_PeriClockControl
	* @brief         		: Enables or Disables peripheral lock for the given GPIO Port
	* @param[*pGPIOx]		: GPIO Port Base address
	* @param  pin_no 		: ENABLE OR DISABLE MACROS
	* @return 				: None
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1) SPI1_PCLK_EN();
		else if(pSPIx == SPI2) SPI2_PCLK_EN();
		else if(pSPIx == SPI3) SPI3_PCLK_EN();
		else if(pSPIx == SPI4) SPI4_PCLK_EN();
		else if(pSPIx == SPI5) SPI5_PCLK_EN();
	}
	else
	{
		if(pSPIx == SPI1) SPI1_PCLK_DI();
		else if(pSPIx == SPI2) SPI2_PCLK_DI();
		else if(pSPIx == SPI3) SPI3_PCLK_DI();
		else if(pSPIx == SPI4) SPI4_PCLK_DI();
		else if(pSPIx == SPI5) SPI5_PCLK_DI();
	}
}

/*
 *  Init and Deinit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	// Configuring the SPI_CR1 Register
	uint32_t tempreg = 0;
	//1. Configure the Device Mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode<<2;
	//2. Configure the Bus configurations
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Bidi Mode should be cleared
		// This Means Two lines would be used those two lines are UniDirectional
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Bidi Mode Should be Set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// Bidi Mode Should be Cleared and
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		// RXONLY bit should be selected (For enabling the sclock) - See Video No. 138 for understanding this step
		tempreg |= (1<<SPI_CR1_RXONLY);
	}
	//3. Configure the SPI Serial Clock Speed (Baud Rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;
	//4. Configure the DFF
	tempreg |=pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF;
	//5. Configure the CPOL
	tempreg |=pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;
	//6. Configure the CPHA
	tempreg |=pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;
	//7. Config the SSM Mode
	tempreg |=pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM;

	pSPIHandle->pSPIx->SPI_CR1 |= tempreg;

}
/*
 * De-Initialization basically means resetting the bits
 *  We have a Special register in RCC Called Peripheral Reset enable register to perform this reset
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1) SPI1_REG_RESET();
	else if(pSPIx==SPI2) SPI2_REG_RESET();
	else if(pSPIx==SPI3) SPI3_REG_RESET();
	else if(pSPIx==SPI4) SPI4_REG_RESET();
	else if(pSPIx==SPI5) SPI5_REG_RESET();
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 *  Data Send and Receive
 */
/*
 *  In SPI CAN and other few protocols we have two types of Communication protocols
 *  Namely, 1. Blocking (Without Interrupt) 2. Non Blocking (Interrupt Based) 3. DMA Based
 */

//1. This is a Blocking call -
// This means that Until the Entire block that is to be transferred
// This function would be always occupied
// This type of SPI Send data is also called as Polling since if there is any problem in the TXE Register
// The program would be Stuck permanently
// So this should be recovered using WatchDog Timers and Stuff

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len>0)
	{
		//1. Wait Until TX Buffer Is empty (Check in the SPI Status Register)
		// Wait until TXE is Set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);
		//2. Check the DFF Bit in CR1
		if((pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF)))
		{
			// 16 Bit DFF
			//1. Load the data into the DR
			pSPIx->SPI_DR = *((uint16_t *)pTxBuffer); // To load Two Bytes of data
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}else
		{
			//8Bit Dff
			pSPIx->SPI_DR = *pTxBuffer; // *pTXBuffer = &dataLen? -> No, This is called Pass by Value of a Variable
			Len--;                      // So it is *pTXBuffer = dataLen
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len>0)
		{
		// Wait until RXNE is Set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);
		//2. Check the DFF Bit in CR1
		if((pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF)))
		{
			// 16 Bit DFF
			//1. Load the data from the DR to the RXBuffer Address
			*((uint16_t *)pRxBuffer) = pSPIx->SPI_DR; // To load Two Bytes of data
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}else
		{
			//8Bit Dff
			*(pRxBuffer) = pSPIx->SPI_DR; // To load 1 Byte of data
			Len--;                      // So it is *pTXBuffer = dataLen
			pRxBuffer++;
		}
		}
}

/*
 *  For Enabling the SPI Peripheral SPE Bit In CR1
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1<<SPI_CR1_SPE); // Setting
	}else
	{
		pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SPE); // Clearing
	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->SPI_CR1 |= (1<<SPI_CR1_SSI); // Setting
		}else
		{
			pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SSI); // Clearing
		}
}
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->SPI_CR2 |= (1<<SPI_CR2_SSOE); // Setting
		}else
		{
			pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_SSOE); // Clearing
		}
}
/*
 *	IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi)
{
		if(EnOrDi == ENABLE)
		{
			if(IRQNumber <=31) // 0 to 31
			{
				// Program ISER0 Register
				*NVIC_ISER0 |= (1<<IRQNumber);
				// *((volatile uint32_t*)0xE000E100) |= (1<<IRQNumber)
			}
			else if(IRQNumber >31 && IRQNumber<64) // 32 to 63
			{
				// Program ISER1 Register
				*NVIC_ISER1 |= (1<<(IRQNumber%32));
			}
			else if(IRQNumber >=64 && IRQNumber <96) // 64 to 95
			{
				// Program ISER2 Register
				*NVIC_ISER2 |= (1<<(IRQNumber%64));
			}
		}
		else
		{
			if(IRQNumber <=31)
			{
				// Program ICER0 Register
				*NVIC_ICER0 |= (1<<IRQNumber);
			}
			else if(IRQNumber >31 && IRQNumber<64)
			{
				// Program ICER1 Register
				*NVIC_ICER1 |= (1<<(IRQNumber%32));
			}
			else if(IRQNumber >=64 && IRQNumber <96)
			{
				// Program ICER2 Register
				*NVIC_ICER2 |= (1<<(IRQNumber%64));
			}
		}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1.  First lets find out the IPR Register
	uint8_t iprReg =(IRQNumber/4);
	uint8_t regPos = (IRQNumber%4);
	uint8_t shift_amount = (8*regPos) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR +(iprReg)) |= (IRQPriority << shift_amount);
}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	//1. Save a TX Buffer address and Len Information in some global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;
	//2. Mark the SPI state as busy in transmission so that
	//   no other code can take over the same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_TXEIE);
	}
	//4. Data Transmission will be handled by the ISR Code (Will Implement later)
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
	//1. Save a TX Buffer address and Len Information in some global variables
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;
	//2. Mark the SPI state as busy in transmission so that
	//   no other code can take over the same SPI peripheral until transmission is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;
	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	//4. Data Transmission will be handled by the ISR Code (Will Implement later)
	return state;
}



void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//Lets check for TXE
	temp1 = pHandle->pSPIx->SPI_SR &(1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 &(1<<SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Lets check for RXNE
	temp1 = pHandle->pSPIx->SPI_SR &(1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 &(1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Lets check for OVR Flag
	temp1 = pHandle->pSPIx->SPI_SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SPI_CR2 &(1<<SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		// handle OVR Error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

// Some Helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF Bit in CR1
	if((pSPIHandle->pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF)))
	{
		// 16 Bit DFF
		//1. Load the data into the DR
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t *)pSPIHandle->pTxBuffer); // To load Two Bytes of data
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}else
	{
		//8Bit Dff
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer; // To load Two Bytes of data
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen)
	{
		// TxLen is Zero , so close the spi transmission and inform the application that
		// Tx is Over.
		// This prevents interrupts from Setting up of TXE Flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF Bit in CR1
	if((pSPIHandle->pSPIx->SPI_CR1 & (1<<SPI_CR1_DFF)))
	{
		// 16 Bit DFF
		//1. Load the data from the DR to the RXBuffer Address
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->SPI_DR; // To load Two Bytes of data
		pSPIHandle->RxLen-=2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}else
	{
		//8Bit Dff
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->SPI_DR; // To load 1 Byte of data
		pSPIHandle->RxLen--;                      // So it is *pTXBuffer = dataLen
		pSPIHandle->pRxBuffer--;
	}
	if(!pSPIHandle->RxLen)
	{
		// Reception is Complete
		// lets turn off the RXNEIE Interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the OVR Flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void) temp;
	//2. Inform the Application
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void) temp;
}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	// This is a weak implementation. The Application may override this function
}
