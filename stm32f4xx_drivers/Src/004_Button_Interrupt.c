/*
 * 004_Button_Interrupt.c
 *
 *  Created on: 26-Dec-2022
 *      Author: Karthikh Amaran
 */


/*
 * 	Exercise 4:
 * 	Connect an external button to PD5 pin and toggle the
 * 	led whenever interrupt is triggered by the button press.
 *
 * 	Interrupt should be triggered during the falling edge of the
 * 	button press.
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
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12; //PD12
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLed); /* This Function is the the One who is loading all the given values to the required Registers */

	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // PD5
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioBtn);  /* This Function is the the One who is loading all the given values to the required Registers */

	// Priority is optional (Useful when there are more interrupts)
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIORITY_15);
	// IRQ Configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	while(1);
	return 0;
}


void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
















