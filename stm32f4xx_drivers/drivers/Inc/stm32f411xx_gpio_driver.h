/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Dec 24, 2022
 *      Author: Karthikh Amaran
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;         /* Possible Values from @GPIO_PIN_NO */
	uint8_t GPIO_PinMode;   		/* Possible Values from @GPIO_PIN_MODE */
	uint8_t GPIO_PinSpeed;  		/* Possible Values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;    /* Possible Values from @GPIO_PIN_PUPDCTR */
	uint8_t GPIO_PinOpType;         /* Possible Values from @GPIO_PIN_OPTYPE */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;



/*
 *  This is a Handle Structure
 */
typedef struct
{
	/* Here the Handler Structure contains Two Structures
	 * 1. GPIO Pin Configurations Structure
	 * 2. The Register Definition for GPIOs structure
	 */
	/* TAKE YOUR TIME AND READ THIS TO UNDERSTAND THE HANDLE STRUCTURE
	 *  Understanding of this handle Structure:
	 *  Handle structure is basically an HAL to avoid the user to know about the bits for configuration
	 *  in the GPIO Register.
	 *  (In Simple Words) This handle structure is Developed in such a way that it prevents to user to not worry about the
	 *  shifting between the bits in the registers of GPIOs.
	 *
	 *  The user needs to just call the GPIO_PinConfig Structure and add the parameters
	 *  he is intended to do so with the GPIOs like the PinNumber, PinMode, PinSpeed, etc.
	 *
	 *   After configuring all the required structures inside the Handle structure.
	 *   The user needs to call the suitable inti function and pass the Handle
	 *
	 *   The reason why GPIO_PinConfig is not a pointer Structure is that it is just
	 *   a collection of GPIO Configuration data entered by the user
	 *
	 *   But whereas the *pGPIOx  represent the base address of the suitable GPIO peripherals
	 *   GPIO A/B/C
	 *
	 *   THe GPIO_Init function would be constructed in such a way that in the address of pGPIOx
	 *   based on the GPIO_PinConfig given by the user
	 *   the GPIO_PinConfig Structure variable would be taken out and based on the values in those
	 *   variables the suitable configuration in the register of the GPIO Address would be done.
	 *
	 */
	GPIO_PinConfig_t GPIO_PinConfig;    /* This holds the GPIO pin configuration settings*/
	GPIO_RegDef_t *pGPIOx; 				/* This holds the base address of the GPIO Register Definition Structure. Refer: stm32f411xx.h */



}GPIO_Handle_t;

/*************************************************************************/
/* MACROS FOR GPIOx'S for configuring the modes and other settings       */
/*************************************************************************/

/*
 *   @GPIO_PIN_NO
 *   Possible GPIO Pin Modes
 */
#define GPIO_PIN_NO_0 					0
#define GPIO_PIN_NO_1 					1
#define GPIO_PIN_NO_2 					2
#define GPIO_PIN_NO_3 					3
#define GPIO_PIN_NO_4 					4
#define GPIO_PIN_NO_5 					5
#define GPIO_PIN_NO_6 					6
#define GPIO_PIN_NO_7 					7
#define GPIO_PIN_NO_8 					8
#define GPIO_PIN_NO_9 					9
#define GPIO_PIN_NO_10 					10
#define GPIO_PIN_NO_11					11
#define GPIO_PIN_NO_12					12
#define GPIO_PIN_NO_13					13
#define GPIO_PIN_NO_14 					14
#define GPIO_PIN_NO_15					15
/*
 *   @GPIO_PIN_MODE
 *   Possible GPIO Pin Modes
 */
#define GPIO_MODE_IN					0
#define GPIO_MODE_OUT    				1
#define GPIO_MODE_ALTFN 				2
#define GPIO_MODE_ANALOG                3
#define GPIO_MODE_IT_FT                 4 // For interrupts(inputs) Falling edge Triggered (USER DEFINED )
#define GPIO_MODE_IT_RT					5 // For interrupts(inputs) Rising edge Triggered  (USER DEFINED)
#define GPIO_MODE_IT_RFT				6 // For interrupts(inputs) Rising and Falling edge Triggered (USER DEFINED)
/*
 *   @GPIO_PIN_OPTYPE
 *   Possible GPIO Output Type
 */
#define GPIO_OP_TYPE_PP					 0
#define GPIO_OP_TYPE_OD					 1
/*
 *   @GPIO_PIN_SPEED
 *   Possible GPIO Output Speed Registers
 */
#define GPIO_SPEED_LOW 					 0
#define GPIO_SPEED_MEDIUM 				 1
#define GPIO_SPEED_FAST 			     2
#define GPIO_SPEED_HIGH 				 3
/*
 *   @GPIO_PIN_PUPDCTR
 *   Possible GPIO PORT  Pull-up/Pull-down Configurations
 */
#define GPIO_NO_PUPD					 0
#define GPIO_PIN_PU			 			 1
#define GPIO_PIN_PD			 			 2

/**************************************************************************************************
 * 								APIs Supported by this driver
 * 				For more information about the APIs check the function definitions
 ***************************************************************************************************/

/*
 *  peripheral Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi);

/*
 *  Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
/*
 * De-Initialization basically means resetting the bits
 *  We have a Special register in RCC Called Peripheral Reset enable register to perform this reset
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *  data read write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 *   IRQ Configuration and ISR Handling
 */

//void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDi);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);









#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
