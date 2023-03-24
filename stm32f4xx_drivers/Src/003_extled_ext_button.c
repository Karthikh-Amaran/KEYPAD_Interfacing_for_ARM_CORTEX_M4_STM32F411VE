/*
 * 003_extled_ext_button.c
 *
 *  Created on: 25-Dec-2022
 *      Author: Karthikh Amaran
 */
/*
 * Exercise 3:
 * Write a program to connect external button to the pin
 * Number PB_12 and external LED to PA14
 * Toggle the LEd whenever the External button is pressed
 */
#define BTN_PRESSED ENABLE
#include<string.h>
#include"stm32f411xx.h"
#include"stm32f411xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0;i<500000/2 ;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed,GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_4)==BTN_PRESSED)
		{
			delay(); // To avoid De-bouncing of the Button
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}

	}
	return 0;
}
















