/*
 * 002_led_button.c
 *
 *  Created on: 25-Dec-2022
 *      Author: Karthikh Amaran
 */

#define BTN_PRESSED ENABLE
#include"stm32f411xx.h"
#include"stm32f411xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0;i<500000/2 ;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed,GpioBtn;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP; // GPIO_OP_TYPE_OD for Open Drain Config
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)==BTN_PRESSED)
		{
			delay(); // To avoid De-bouncing of the Button
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}

	}
	return 0;
}
















