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
	while(1)
	{
		// Wait till the Button is Pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		// Enabling the SPI Peripheral (SPE Bit of CR1)
		// It is always better to configure all the SPI Modes and then Enable the SPE
		SPI_PeripheralControl(SPI2, ENABLE);

//1. CMD_LED_CTRL <pin_no(1)>    <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// Send the Command
		SPI_SendData(SPI2,&commandcode,1); // call by Reference

/*
 * Remember: In SPI Communication For each byte transmission from the Master, the same master also receives a Garbage Byte
 * 			 Since the Transmission and Reception in SPI always happens by using the Shift registers.
 *           So Due to this Garbage Byte Reception the RXNE Flag is set. So Do a Dummy Read to clear the RXNE Flag.
 */
		// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
		SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)

		// Send some Dummy bits (1 Byte) to fetch the response from the slave.
		SPI_SendData(SPI2,&dummy_write,1); // When this API call returns response from the slave would have arrived at the master. So lets read next.

		// Read the ACK Byte Received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if(SPI_verifyResponse(ackbyte)) // Call by Value
		{
			// Send Arguments
			args[0]=LED_PIN;
			args[1]=LED_ON;
			// Send Arguments
			SPI_SendData(SPI2,args,2);
		}
// end
//2. CMD_SENSOR_READ <analog pin number(1)>
		// Wait till the Button is Pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		commandcode = COMMAND_SENSOR_READ;
		// Send Command
		SPI_SendData(SPI2,&commandcode,1); // call by Reference
		// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
		SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)

		// Send some Dummy bits (1 Byte) to fetch the response from the slave.
		SPI_SendData(SPI2,&dummy_write,1); // When this API call returns response from the slave would have arrived at the master. So lets read next.

		// Read the ACK Byte Received
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if(SPI_verifyResponse(ackbyte)) // Call by Value
		{
			// Send Arguments
			args[0]=ARDUINO_PIN0;
			// Send Arguments
			SPI_SendData(SPI2,args,1);

			// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
			SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)
			// Slave actually takes some time to read the analog value(Slave does some ADC conversion)
			// so we should wait before sending the dummy bits that is to read the analog value
			// insert some delay so that slave can be ready with the data
			delay();
			// Send some Dummy bits (1 Byte) to fetch the response from the slave.
			SPI_SendData(SPI2,&dummy_write,1);
			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
		}
//end
// 3. CMD_LED_READ <pin no(1)> - PIN NO: 9
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();
	commandcode = COMMAND_LED_READ;
	// Send Command
	SPI_SendData(SPI2,&commandcode,1); // call by Reference
	// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
	SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)

	// Send some Dummy bits (1 Byte) to fetch the response from the slave.
	SPI_SendData(SPI2,&dummy_write,1); // When this API call returns response from the slave would have arrived at the master. So lets read next.

	// Read the ACK Byte Received
	SPI_ReceiveData(SPI2,&ackbyte,1);
	if(SPI_verifyResponse(ackbyte)) // Call by Value
	{
		// Send Arguments
		args[0]=LED_PIN;
		// Send Arguments
		SPI_SendData(SPI2,args,1);

		// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
		SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)
		// Send some Dummy bits (1 Byte) to fetch the response from the slave.
		SPI_SendData(SPI2,&dummy_write,1);
		uint8_t led_status;
		SPI_ReceiveData(SPI2,&led_status,1);
	}
//end
// 4. CMD_PRINT <pin no(1)>; Send a string from Master to Slave
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();
	commandcode = COMMAND_PRINT;
	// Send Command
	SPI_SendData(SPI2,&commandcode,1); // call by Reference
	// Do Dummy read (To Clear the RXNE BIT) - Explanation read the Above comment
	SPI_ReceiveData(SPI2,&dummy_read,1); // This is Only to Clear the RXNE Bit (Can't get proved in the Logic Analyzer capture)

	// Send some Dummy bits (1 Byte) to fetch the response from the slave.
	SPI_SendData(SPI2,&dummy_write,1); // When this API call returns response from the slave would have arrived at the master. So lets read next.

	// Read the ACK Byte Received
	SPI_ReceiveData(SPI2,&ackbyte,1);
	uint8_t message[] = "Hello received";
	if(SPI_verifyResponse(ackbyte)) // Call by Value
	{
		// First send the length of the string (This is how Arduino-Uno code is configured)
		args[0] = strlen((char *)message);
		SPI_SendData(SPI2,args,1);
		SPI_ReceiveData(SPI2,&dummy_read,1);

		delay(); // Maybe for initialization

		for(uint8_t i=0;i<=args[0];i++)
		{
			SPI_SendData(SPI2,&message[i],1);
			SPI_ReceiveData(SPI2,&dummy_read,1);
		}
	}
//end
// 5. CMD_ID_READ <pin no(1)> Send a string from Slave to Master
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
		//Let's Confirm SPI is not Busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
