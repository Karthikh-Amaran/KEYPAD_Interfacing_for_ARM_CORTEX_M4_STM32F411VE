/*
 * 005_SPI_TX_test.c
 *
 *  Created on: 02-Jan-2023
 *      Author: Karthikh Amaran
 */

// First Step is to See what are the Pins that can be used for SPI
// That can be seen from the Alternate Function Mapping in Data-sheet

/*
 *  From the Data-Sheet We inferred that we can
 *  PB14 --> SPI2_MISO
 *  PB15 --> SPI2_MOSI
 *  PB13 --> SPI2_SCLK
 *  PB12 --> SPI2_NSS
 *  ALT Function Mode : 5
 */

/*
 *
 *  Test the SPI_SendData API to send the string "Hello World"
 *  and use the below configurations
 *
 *  i)   SPI-2 Master Mode
 *  ii)  SCLK = Max Possible
 *  iii) DFF = 0 and DFF = 1
 */
#include"stm32f411xx.h"
#include<stdint.h>
#include<string.h>
void SPI2_GPIOInits()
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2Inits()
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // Generates SCLk of 8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Software Slave management Enable

	SPI_Init(&SPI2Handle);
}

int main()
{
	char user_data[] = "Hello World";
	// This Function is Used to initialize the GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();
	// This Function is Used to initialize  the SPI Registers
	SPI2Inits();
	// This makes NSS Signal Internally high and avoids MODF Error
	SPI_SSIConfig(SPI2,ENABLE);
	// Enabling the SPI Peripheral (SPE Bit of CR1)
	// It is always better to configure all the SPI Modes and then Enable the SPE
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_SendData(SPI2,(uint8_t *)user_data,strlen(user_data));
	// Lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
	SPI_PeripheralControl(SPI2,DISABLE);
	while(1);
	return 0;
}

