/*
 * Testing the drivers:
 *
 * Exercise:
 * Write a Program to Toggle the On Board LED with some delay.
 * Case1: Use Push Pull Configuration for the output Pin
 * Case2: Use open Drain Configuration for the output Pin
 *
 *
 *
 * 001_ledtoggle.c
 *
 *  Created on: 25-Dec-2022
 *      Author: Karthikh Amaran
 */

#include"stm32f411xx.h"
#include"stm32f411xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0;i<1000000/2;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP; // GPIO_OP_TYPE_OD for Open Drain Config
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}



































