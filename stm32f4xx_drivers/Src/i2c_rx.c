/*
 * i2c.c
 *
 *  Created on: 04-Feb-2023
 *      Author: Karthikh Amaran
 */
/*
 *  EXERCISE: 	I2C Master (STM32 Discovery) and I2C Slave(Arduino Board) Communication
 *
 *  When the Button on the STm32 Board (master) is pressed, master should read and display the data from the Arduino Board (Slave).
 *  First master has to get the length of the data from the slave to read subsequent data from the slave.
 *  1. Use I2C SCL = 100KHz (Standard Mode)
 *  2. Use Internal Pull up resistors for SDA and SCL Lines
 *
 *  The ARduino Board is programmed in such a way that
 *  the Master should first send/write the command 0x51
 *  to get the length of the data that slave has, so that
 *  master would read them to know the length of the data
 *
 *  Then the master should send/write the command 0x52 to receive
 *  the actual data from the slave
 *
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_i2c_driver.h"

#define MYADDR       0x61
I2C_Handle_t I2C1_Handle;
#define SLAVE_ADDR   0x68 // Configured in the Slave

// For semihosting
extern void initialise_monitor_handles();
// Receive buffer
uint8_t rcv_buf[32];
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
	// To read a data from the Slave a master should forst know about
	// the length of the data
	// So we use command 0x51 to know the length of the data
	// After a button press 0x51 is send to the slvae and ACK is awaited
	uint8_t commandcode,len;
	initialise_monitor_handles();
	printf("Application is Running");
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
		/***DATA WRITE*********************************************/
		// Wait till the Button is Pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		 //Send some data to the Slave
		commandcode = 0x51; // Send it to the slave and receive the length of the data
		I2C_MasterSendData(&I2C1_Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR);
		/***DATA READ*********************************************/
		I2C_MasterReceiveData(&I2C1_Handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR);
		/**********************************************************/
		// Got the length
		/**********************************************************/
		commandcode = 0x52;
		I2C_MasterSendData(&I2C1_Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR);
		I2C_MasterReceiveData(&I2C1_Handle, rcv_buf, len, SLAVE_ADDR,I2C_DISABLE_SR);
		// rcv_buf is an array so no need to use & (See in eclipse Arrays passing to a function code)
		rcv_buf[len+1] = '\0';
		printf("Data:%s",rcv_buf);
	}
}
