/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: 16-Jan-2023
 *      Author: Karthikh Amaran
 */
#include "stm32f411xx_spi_drivers.h"
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
uint32_t RCC_GetPLLOutputClock(void);

void small_delay(void)
{
	for(uint32_t i=0;i<1000;i++);
}
/* Function to generate a START Condition */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}
/* This Function used for Sending the Slave Adsress after Configuring the R/W Bit */
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // Address is only 7bits (Slave Address + Read/Write bit)
	SlaveAddr &= ~(1); // Clearing the LSB since it is Writing mode
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // Address is only 7bits (Slave Address + Read/Write bit)
	SlaveAddr |=(1); // Setting the LSB since it is Reading mode
	pI2Cx->DR = SlaveAddr;
}
/* this Function is used for Clearing the ADDR Bit after it is set, by reading the SR1 and SR2 Registers  */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummy = pI2Cx->SR1; // Dummy reading
	dummy = pI2Cx->SR2; // Dummy Reading
	(void) dummy; // To avoid unused error warning
}
/* Function to Generate the Stop Condition*/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}
//void RCC_GetPLLOutputClock(void) {/* This is just a Dummy Implementation (Since this is not used in this course*/}
/* A Function to identify the APB Bus clock to set the FREQ Bitas*/
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint32_t RCC_GetPCLK1Value(void) // I2C Peripheral is Connected to APB1
{
	// The clock source that is getting supplied to the Bus can be identified based
	// on the values in the respective registers of RCC
	// And it is that we have to also look at the values of the pre-scalar that are
	// before the Peripheral (Taken from the Clock tree given)
	// Here we need to look at the pre-scalar values of AHB Bus and APB Bus
	// 1. Finding the System Clock Source
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p,apbp;
	clksrc = ((RCC->RCC_CFGR) & (0x3<<2));
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		//SystemClk = RCC_GetPLLOutputClock();
	}

	// 2. Finding the Pre-scaler value of the AHBus
	temp = ((RCC->RCC_CFGR >>4) & (0xF));
	if(temp<8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}
	// 3. Finding the APB1 Pre-scaler
	temp = ((RCC->RCC_CFGR >>10)&(0x7));
	if(temp<4)
	{
		apb1p = 1;
		(void) apbp;
	}else
	{
		apb1p = AHB_PreScaler[temp-4];
	}
	pclk1 = (SystemClk/ahbp)/apb1p; // Return the exact clock frequency supplied (Say 16MHz)
	return pclk1;
}

/* To Configure the RCC to the peripheral required */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1) I2C1_PCLK_EN();
		else if(pI2Cx == I2C2) I2C2_PCLK_EN();
		else if(pI2Cx == I2C3) I2C3_PCLK_EN();
	}
	else
	{
		if(pI2Cx == I2C1) I2C1_PCLK_DI();
		else if(pI2Cx == I2C2) I2C2_PCLK_DI();
		else if(pI2Cx == I2C3) I2C3_PCLK_DI();
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// Enable the Clock for I2Cx
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	uint32_t tempreg = 0;
	//tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10); // See the possible values for the I2C_ACKControl
	//(pI2CHandle->pI2Cx->CR1) |= tempreg;
	//(pI2CHandle->pI2Cx->CR1) |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
	// (2) Next step is to configure the FREQ bits that is based on the Peripheral Clock Frequency
	// We should find what is the Peripheral clock frequency and then configure the FREQ Bits
	tempreg = 0;
	uint32_t mytemp = RCC_GetPCLK1Value();
	tempreg |= (mytemp/1000000U); // Divided by 1 Mhz (Given in the Reference Manual for FREQ bits)
	pI2CHandle->pI2Cx->CR2 |= (tempreg&0x3f);
	// (3) Configuring the slave address if our Board is behaving as a slave.
	// We should do this is Own Address Register (I2C_OAR)
	pI2CHandle->pI2Cx->OAR1 &= ~(1<<15); // Setting as 7 bit slave address mode
	pI2CHandle->pI2Cx->OAR1 |= (1<<14); // Given in the Reference Manual that this bit should be always set to 1 by the software
	tempreg = 0;
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress;
	pI2CHandle->pI2Cx->OAR1 |= (tempreg<<1);
	// (4) CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	ccr_value = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is Standard Mode
		tempreg &= ~(1<<15);
		// We need to create a 100 kHz SCL => 10usec = T(SCL)
		// T(pclk1) = 1/16MHz = 62.5nsec
		// T(high) = (CCR)(T(pclk1)) => T(high) = 50% Duty cycle = 10u(T(SCL))/2 => CCR = 80;
		ccr_value = RCC_GetPCLK1Value()/ (2*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value&0xfff);
	}else
	{
		// Mode is Fast Mode
		tempreg |= (1<<15);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			tempreg &= ~(1<<14);
			ccr_value = (RCC_GetPCLK1Value()) / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			pI2CHandle->pI2Cx->CCR |= (tempreg|ccr_value); // 1000 0000 0000 0000
		}
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			tempreg |= (1<<14);
			ccr_value = (RCC_GetPCLK1Value()) / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			pI2CHandle->pI2Cx->CCR |= (tempreg|(ccr_value&0xfff));
		}
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;
	// TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is Standard Mode
		tempreg = ((RCC_GetPCLK1Value() * 1000000U)+1);
	}else
	{
		// Mode is Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300)/1000000000U)+1;
	}
	pI2CHandle->pI2Cx->TRISE |= (tempreg&0x3f);
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE); // Enabling the PE Bit
	// (1) From the I2C_Config_t structure configuring for the "I2C_ACKControl"
	if(pI2CHandle->I2C_Config.I2C_ACKControl == 1)
	{
		(pI2CHandle->pI2Cx->CR1) |= (1<<10);
	}
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1) I2C1_REG_RESET();
	else if(pI2Cx == I2C2) I2C2_REG_RESET();
	else if(pI2Cx == I2C3) I2C3_REG_RESET();
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate the Start Condition
	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: until SB is Cleared SCL will be stretched (pulled to LOW)
	//3. Send the address of the slave with r/nw bit set w(0) (Total 8 bits) - Since it is master send it is always write
	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	//5. Clear the ADDR flag according to its software sequence
	//   Note: until ADDR is Cleared SCL will be stretched (pulled to Low)
	//6. Send the data until Len becomes 0
	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP Condition
	//   Note: TXE1, BTF=1, means that both the SR and DR are empty and next transmission should begin
	//   when BTF=1 SCl will be stretched (pulled to low)
	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
    //Note: Generating STOP, automatically clears the BTF
	/* 1  - Generate Start Condition */
	//(pI2CHandle->pI2Cx->CR1) |= (1<<I2C_CR1_START);
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	/* 2  - Check SB Flag Set to conform Start Generation*/
	//while(! ((pI2CHandle->pI2Cx->SR1 & I2C_FLAG_SB) == 1U));
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));
	/* 3  - Send the Address data */
	//((pI2CHandle->pI2Cx->DR)) |= SlaveAddr<<1;
	//((pI2CHandle->pI2Cx->DR)) &= ~(1U);
	I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx,SlaveAddr);
	/* 4 */
	//uint8_t temp1 = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR);
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	//while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	/* 5 */
	//pI2CHandle->pI2Cx->SR1 &= ~(I2C_FLAG_ADDR); (This is Wrong) Since ADDR bit should be automatically cleared by reading the SR1 and SR2 bits
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	/* 6 */
	while(Len>0)
	{
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))); // Wait Until TXE is Set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
		//small_delay();
	}
	/* 7 */
	//while(!((pI2CHandle->pI2Cx->SR1 & I2C_FLAG_TXE) == 1U));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	//while(!((pI2CHandle->pI2Cx->SR1 & I2C_FLAG_BTF) == 1U));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));
	/* 8 */
	//pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate The START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that Start generation is completed by checking the SB Flag in the SR1
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	// 3. Send the address of the slave with R/W bit set to R(1) (Total 8 Bits)
	I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx,SlaveAddr);
	// 4. Wait until address phase is completed by checking the ADDR Flag in the SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	// 5. Check whether the length of the data is 1byte or more than 1byte
	if(Len == 1)
	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
		// Check whether the Address phase is successful by checking the ADDR Bit.(Not Sure)
		// Now clear the ADDR Bit
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Check whether the RXNE is Set
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
		if(Sr == I2C_DISABLE_SR)
			// Generate Stop Condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// Read the Data Register
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}
	else if(Len >1)
	{
		// Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Read the data until the Len becomes zero
		for(uint32_t i=Len; i>0; i--)
		{
			// Wait until RXNE Set
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
			if(i==2)// If only last two bytes are remaining (refer notes for special case explanation)
			{
				// Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
				if(Sr == I2C_DISABLE_SR)
					// Generate Stop Condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// Read the data from Data Register
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			// Increment the Buffer address
			pRxbuffer++;
		}
	}
	// Re-enable the Acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
// Controlling the PE Bit
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 = (1<<I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// Enable the Acking
		pI2Cx->CR1 |= 1<<I2C_CR1_ACK;
	}
	else if(EnorDi == DISABLE)
	{
		// Disable the Acking
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data)
{
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = (pI2CHandle->pI2Cx->CR2) & (1<<I2C_CR2_ITEVTEN);
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1<<I2C_CR2_ITBUFEN);
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		// The interrupt is generated because of SB event
		// This block will not be executed in slave mode because for slave SB is always zero
		// In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		// ADDR Flag is Set
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		// BTF Flag is Set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make Sure that TXE is also SET
			if(pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_TXE))
			{
				// BTF and TXE = 1
				// Now close the TXN by
				//(0) Check whether the Length has reached Zero
				if(pI2CHandle->TxLen == 0)
				{
					//(1) Generate STOP Condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					//(2) Reset all the member elements of the Handle Structure
					I2C_CloseSendData(); /* TODO */
					//(3) Modify the application about transmission complete
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT); /* TODO */
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{

		}
	}
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		// STOP Flag is Set
		// Clear the STOPF (i.e 1) Read SR1 2) Write to CR1)
		// (1) Already done by reading the value for temp 3
		pI2CHandle->pI2Cx->CR1 |= 0x0000; // Writing
		// Notify the Application that STOP is detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP); /* TODO */
	}
	//5. Handle For interrupt generated by TXE event
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		// Check for Device mode (Only do when it is in Master mode)
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
		{
			// TXE Flag is Set
			// We have to do the data Transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//1. Load the data into DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
					//2. Decrement the TxLen
					pI2CHandle->TxLen--;
					//3. Increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
		}
	}
	//6. Handle For interrupt generated by RXNE event
	temp3 = (pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		// RXNE Flag is Set
	}
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

}
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi)
{

}
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

}

