
/*	THIS CODE IS BASICALLY THE SEPERATE IMPLEMENTATION OF THE PREVIOUS SPI CODE 007.
	IN 007 WE ARE ACTUALLY USING 5 TIMES SAME BUTTON CLICK TO DIFFERENT COMMANDS
	BUT HERE WE CAN DO EACH COMMAND FOR ONE SINGLE PRESS.
	THIS IS BASICALY USED AS A TEST BENCH FOR ALL THE 5 COMMANDS IN 004.
	TAKE A COMMAND'S CODE AND PASTE THAT HERE ANS TEST THE WORKING OF IT.
	I TESTED ALL 5 IN THIS FASHION AND ALL 5 ARE WORKING SUCESSFULLY
*/
/*
 * SPI_RX.c
 *
 *  Created on: 08-Jan-2023
 *      Author: Karthikh Amaran
 */

/*
 *  Exercise:
 *  SPI Master (STM) and SPI Slave (Arduino) command and Response based communication.
 *  When the button on the master is pressed, master sends a command to the slave and
 *  slave responds as per the command implementation.
 *
 *  1. Use SPI Full Duplex Mode
 *  2. ST Board will be in SPI Master Mode and Arduino will be configured or SPI Slave
 *  3. Use DFF=0
 *  4. Use hardware slave management (SSM=0)
 *  5. SCLK Speed = 2MHz, fclk = 16Mhz
 *
 *  PB14 --> SPI2_MISO - CH 2
 *  PB15 --> SPI2_MOSI - CH 1
 *  PB13 --> SPI2_SCLK - CH 0
 *  PB12 --> SPI2_NSS  - CH 3
 */


#include"stm32f411xx.h"
#include<stdint.h>
#include<string.h>
// Command Codes that the slave recognizes
#define COMMAND_LED_CTRL      0x50
#define COMMAND_SENSOR_READ   0x51
#define COMMAND_LED_READ      0x52
#define COMMAND_PRINT         0x53
#define COMMAND_ID_READ       0x54
// Led Status Codes
#define LED_ON                1
#define LED_OFF               0
// Arduino Analog Pins
#define ARDUINO_PIN0          0
#define ARDUINO_PIN1          1
#define ARDUINO_PIN2          2
#define ARDUINO_PIN3          3
#define ARDUINO_PIN4          4
#define ARDUINO_PIN5          5
// Arduino Led Pin
#define LED_PIN               9
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

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
uint8_t SPI_verifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		// ACK
		return 1;
	}
	    // NACK
	  return 0;
}

int main()
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	GPIO_ButtonInit();
	// This Function is Used to initialize the GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();
	// This Function is Used to initialize  the SPI Registers
	SPI2Inits();
	// Enabling the SSOE bit so the we can control NSS in Hardware Slave Management Mode
	SPI_SSOEConfig(SPI2, ENABLE);
	uint8_t commandcode = COMMAND_LED_CTRL;
	uint8_t ackbyte;
	uint8_t args[2];

	while(1)
	{
		// Wait till the Button is Pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		// Enabling the SPI Peripheral (SPE Bit of CR1)
		// It is always better to configure all the SPI Modes and then Enable the SPE
		SPI_PeripheralControl(SPI2, ENABLE);
/**************************************************************************************************/
//2. CMD_SENSOR_READ <analog pin number(1)>
while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
delay();
commandcode = COMMAND_ID_READ;
// Send Command
SPI_SendData(SPI2,&commandcode,1); // call by Reference
// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)

// Send some Dummy bits (1 Byte) to fetch the response from the slave.
SPI_SendData(SPI2,&dummy_write,1); // When this API call returns response from the slave would have arrived at the master. So lets read next.

// Read the ACK Byte Received
SPI_ReceiveData(SPI2,&ackbyte,1);
uint8_t id[11];
uint32_t i=0;
if(SPI_verifyResponse(ackbyte)) // Call by Value
{
	delay();
	// Read 10 Bytes ID from the slave
	for(i=0;i<10;i++)
	{
		// Send the dummy bytes to fetch the dat from the slave:
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2,&id[i],1);
	}
	id[10]='\0';
}
//end
/**************************************************************************************************/

		//Let's Confirm SPI is not Busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		SPI_PeripheralControl(SPI2,DISABLE);

}
	return 0;
}
