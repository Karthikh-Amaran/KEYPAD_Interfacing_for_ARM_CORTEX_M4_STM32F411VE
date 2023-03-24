/*
 * i2c.c
 *
 *  Created on: 04-Feb-2023
 *      Author: Karthikh Amaran
 */
/*
 *  EXERCISE: 	I2C Master (STM32 Discovery) and I2C Slave(Arduino Board) Communication
 *
 *  When the Button on the STm32 Board (master) is pressed, master should send data to the Arduino Board (Slave).
 *  The data received by the Arduino Board will be displayed on the Serial Monitor Terminal of Arduino IDE
 *  1. Use I2C SCL = 100KHz (Standard mode)
 *  2. Use External Pull up Resistors (3.3Kohm) for SDA and SCL Line
 *  Note: If you don't have external Pull Up resistors, you can also try with
 *  activating the STM32 I2C Pin's Internal Pull Up Resistors.
 *
 */

#include <stdint.h>
#include <string.h>
#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_i2c_driver.h"

#define MYADDR       0x61
I2C_Handle_t I2C1_Handle;
#define SLAVE_ADDR   0x68 // Configured in the Slave

// Some Data
uint8_t some_data[] ="We are Testing I2C...";
void delay(void)
{
	for(uint32_t i=0;i<5000000;i++);
}
void I2C1_GPIO_Inits(void)
{
	GPIO_Handle_t I2C1Pins;
	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode 		=  GPIO_MODE_ALTFN;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed 		=  GPIO_SPEED_FAST;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOpType 		=  GPIO_OP_TYPE_OD;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_PIN_PU;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode  =  4;
	// PA6 Used as SCL
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_6;
	GPIO_Init(&I2C1Pins);
	// PA9 Used as SDA
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_7;
	GPIO_Init(&I2C1Pins);
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
void I2C1Inits()
{
	//I2C_Handle_t I2C1_Handle;
	I2C1_Handle.pI2Cx = I2C1;
	I2C1_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MYADDR;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1_Handle);
}

int main()
{
	/*
    I2C1_Handle.pI2Cx = I2C1;
	(I2C1_Handle.pI2Cx->CR1) |= (1<<8);
	uint32_t * pRCC = (uint32_t *)(0x40023800+0x40);
	*pRCC |= (1<<21);
	uint32_t * pI2C = (uint32_t *)(0x40005400);
	*pI2C |= (1<<0);
	*pI2C |= (1<<10);
	*/

	// I2C Pin Inits
	I2C1_GPIO_Inits();
	// I2C1 inits
	I2C1Inits();
	// Enable the I2C Peripheral - (PE Bit)
	//I2C_PeripheralControl(I2C1_Handle.pI2Cx, ENABLE);
	// Wait for button Press
	// Button Init
	GPIO_ButtonInit();
	while(1)
	{
		// Wait till the Button is Pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		 //Send some data to the Slave
		I2C_MasterSendData(&I2C1_Handle, some_data, strlen((char *)some_data), SLAVE_ADDR);
	}
}
