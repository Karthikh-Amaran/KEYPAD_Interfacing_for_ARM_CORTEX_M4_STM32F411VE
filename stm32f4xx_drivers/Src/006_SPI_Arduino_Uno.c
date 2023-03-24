/*
 * 006_SPI_Arduino_Uno.c
 *
 *  Created on: 02-Jan-2023
 *      Author: Karthikh Amaran
 */


/*
 * Exercise:
 * SPI Master (STM) and SPI Slave (Arduino) Communication
 * When the button on the master is pressed, master should send string of data to the Arduino Slave
 * connected. The Data received by the Arduino Will be displayed on the Arduino Serial Port.
 *
 * 1. Use SPI Full Duplex Mode
 * 2. ST Board will be in SPI Master Mode and Arduino will be configured for SPI Slave Mode
 * 3. Use DFF = 0 - 8 Bits - 1Byte
 * 4. Use Hardware Slave management (SSM = 0)
 * 5. SCLK Speed = 2MHz, fclk = 16MHz
 *
 * In this exercise master is not going to receive anything or the slave.
 * So you may not configure the MISO Pin
 *
 * Note: Slave does not know how many bytes of data master is going to send. So master first sends the number of bytes
 * info which slave is going to receive next.
 *
 *  From the Data-Sheet We inferred that we can
 *  PB14 --> SPI2_MISO
 *  PB15 --> SPI2_MOSI
 *  PB13 --> SPI2_SCLK
 *  PB12 --> SPI2_NSS
 *  ALT Function Mode : 5
 *
 *  CIRCUIT DIAGRAM FOR THE FOLLOWING PROJECT:
 *  PB13 --> SPI2_SCLK --> LV1 --> HV1 --> PIN 13
 *  PB15 --> SPI2_MOSI --> LV2 --> HV2 --> PIN 11
 *  PB14 --> SPI2_MISO --> -X- --> -X- --> -X-
 *  PB12 --> SPI2_NSS  --> LV3 --> HV3 --> PIN 10
 *  FOR LOGIC ANALYSER:
 *  PB14 --> CH0
 *  PB15 --> CH1
 *  PB13 --> -X-
 *  PB12 --> CH2
 *  FOR LOGIC LEVEL CONVERTER:
 *  LV -> 3V
 *  HV -> 5V
 *  GND -> GND
 */


#include"stm32f411xx.h"
#include<stdint.h>
#include<string.h>
void delay(void)
{
	for(uint32_t i=0;i<1000000 ;i++);
}

void SPI2_GPIOInits()
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN; // Setting the GPII in alt function Mode
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5; // AF5 which is Alt Function Mode 5 Taken From ALternate Function mapping for that GPIO Pin
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2Inits()
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // Generates SCLk of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware Slave management Enable

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);
}

int main()
{
	char user_data[] = "Hello World";
	GPIO_ButtonInit();
	// This Function is Used to initialize the GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();
	// This Function is Used to initialize  the SPI Registers
	SPI2Inits();
	// Enabling the SSOE bit so the we can control NSS in Hardware Slave Management Mode
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1)
	{
		// Wait till the Button is Pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		// Enabling the SPI Peripheral (SPE Bit of CR1)
		// It is always better to configure all the SPI Modes and then Enable the SPE
		SPI_PeripheralControl(SPI2, ENABLE);
		// First lets send the Length information of the Data
		uint8_t dataLen = strlen(user_data); // Contains the Length of the data to sent
		SPI_SendData(SPI2,&dataLen,1);
		SPI_SendData(SPI2,(uint8_t *)user_data,strlen(user_data));
		// Lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
