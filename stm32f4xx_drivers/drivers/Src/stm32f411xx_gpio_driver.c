/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Dec 24, 2022
 *      Author: Karthikh Amaran
 */


#include "stm32f411xx_gpio_driver.h"

/*
 *  peripheral Clock Setup
 */


/*
    * @Fn            		: GPIO_PeriClockControl
	* @brief         		: Enables or Disables peripheral lock for the given GPIO Port
	* @param[*pGPIOx]		: GPIO Port Base address
	* @param  pin_no 		: ENABLE OR DISABLE MACROS
	* @return 				: None
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOH) GPIOH_PCLK_EN();
	}
	else
	{
		if(pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE) GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOH) GPIOH_PCLK_DI();
	}
}

/*
 *  Init and Deinit
 */

/*
    * @Fn            		: GPIO_Init
	* @brief         		: Complete Setup or Initialization of the pin
	* 						  (Like defining the Pin Number, Pin mode, Pin Speed,
	* 						  Pullup or Pulldown, Pin Output Type, Pin Alternative function mode)
	* @param[*pGPIOHandle]  : It is the data present in that structure
	* @return 				: None
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//0. Enabling the Clock for the GPIO
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the Mode of GPIO Pin
	uint32_t temp=0; // temp.register
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		/* 2 multiplication with the Pin Number is because Each Pin Mode takes two bits for their modes*/
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the required bits
		pGPIOHandle->pGPIOx->MODER |= temp;  // Setting the required bits
	}
	else
	{
		// This part will be coded later (Interrupt Mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clearing the Corresponding RTSR Bit
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clearing the Corresponding FTSR Bit
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO Port Selection in SYSCFG_EXTICR
		uint8_t temp1=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4);
		uint8_t temp2=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4);
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] = portcode << (temp2*4);


		//3. Enable the EXTI Interrupt delivery using IMR
		// Setting the bit as 1 enables the Interrupt by Un-masking
		EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// Setting the bit as 1 enables the Interrupt by Un-masking
	}
	temp =0;
	// 2. Configure the Output Speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x11<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the required bits
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp =0;
	// 3. Configure the PUPD Settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x11<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the required bits
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp =0;
	// 4. Configure the OP Type
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x11<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the required bits
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp =0;
	// 5. Configure the Alternate functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			// Configure the Alt Function registers
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
			{
				temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
				pGPIOHandle->pGPIOx->AFRL &= ~(0xF<<4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the required bits
				pGPIOHandle->pGPIOx->AFRL |= temp;
				temp =0;
			}
			else
			{
				temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4*((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-(8))));
				pGPIOHandle->pGPIOx->AFRH &= ~(0xf<<4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8)); // Clearing the required bits
				pGPIOHandle->pGPIOx->AFRH |= temp;
				temp =0;
			}
		}
}
/*
 * De-Initialization basically means resetting the bits
 *  We have a Special register in RCC Called Peripheral Reset enable register to perform this reset
 */

/*
    * @Fn            		: GPIO_DeInit
	* @brief         		: Resetting all the configurations of the GPIO (Using that RCC Peripheral Reset register)
	* @param[*pGPIOx]		: GPIO Port Base address
	* @return 				: None
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if(pGPIOx == GPIOH) GPIOH_REG_RESET();
}

/*
 *  data read write
 */

/*
    * @Fn            		: GPIO_ReadFromInputPin
	* @brief         		: To read from One pin
	* @param[*pGPIOx]		: GPIO Port Base address
	* @param [PinNumber]	: Pin Number to be read
	* @return 				: Read Value from the Pin (0 or 1)
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&(0x00000001));
	return value;
}

/*
    * @Fn            		: GPIO_ReadFromInputPort
	* @brief         		: To read from One Port
	* @param[*pGPIOx]		: GPIO Port Base address
	* @return 				: Read Value from the Port
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*
    * @Fn            		: GPIO_WriteToOutputPin
	* @brief         		: To write on the Output Pin
	* @param[*pGPIOx]		: GPIO Port Base address
	* @param[PinNumber] 	: Pin Number where the write operations has to be done
	* @param[Value]         : Value to be written to the Pin
	* @return 				: None
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit field Corresponding to the Pin Number
		pGPIOx->ODR |= (0x1<<PinNumber);
	}
	else
	{
		// Write 0
		pGPIOx->ODR &= ~(0x1<<PinNumber);
	}
}

/*
    * @Fn            		: GPIO_WriteToOutputPort
	* @brief         		:  To write on the Output Port
	* @param[*pGPIOx]		: GPIO Port Base address
	* @param[Value]         : Value to be written to the Port
	* @return 				: None
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR |= Value;
}

/*
    * @Fn            		: GPIO_ToggleOutputPin
	* @brief         		: To Toggle the state of the Output Pin
	* @param[*pGPIOx]		: GPIO Port Base address
	* @param[PinNumber] 	: Pin Number to be toggled
	* @return 				: None
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (0x1<<PinNumber); // Xor used for toggling
}

/*
 *   IRQ Configuration and ISR Handling
 */

/*
    * @Fn            		: GPIO_IRQConfig
	* @brief         		: Configuring the IRQ
	* @param[IRQNumber]		: IRQ Number to be configured
	* @param[IRQPriority]	: IRQ Priority to be assigned
    * @param[EnOrDi]		: Enable or Disable the registers
	* @return 				: none
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi)
{
		if(EnOrDi == ENABLE)
		{
			if(IRQNumber <=31) // 0 to 31
			{
				// Program ISER0 Register
				*NVIC_ISER0 |= (1<<IRQNumber);
				// *((volatile uint32_t*)0xE000E100) |= (1<<IRQNumber)
			}
			else if(IRQNumber >31 && IRQNumber<64) // 32 to 63
			{
				// Program ISER1 Register
				*NVIC_ISER1 |= (1<<(IRQNumber%32));
			}
			else if(IRQNumber >=64 && IRQNumber <96) // 64 to 95
			{
				// Program ISER2 Register
				*NVIC_ISER2 |= (1<<(IRQNumber%64));
			}
		}
		else
		{
			if(IRQNumber <=31)
			{
				// Program ICER0 Register
				*NVIC_ICER0 |= (1<<IRQNumber);
			}
			else if(IRQNumber >31 && IRQNumber<64)
			{
				// Program ICER1 Register
				*NVIC_ICER1 |= (1<<(IRQNumber%32));
			}
			else if(IRQNumber >=64 && IRQNumber <96)
			{
				// Program ICER2 Register
				*NVIC_ICER2 |= (1<<(IRQNumber%64));
			}
		}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1.  First lets find out the IPR Register
	uint8_t iprReg =(IRQNumber/4);
	uint8_t regPos = (IRQNumber%4);
	uint8_t shift_amount = (8*regPos) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR +(iprReg)) |= (IRQPriority << shift_amount);
}


/*
    * @Fn            		: GPIO_IRQHandling
	* @brief         		: Configure the IRQ handling of a Pin
	* @param[PinNumber]		: PinNumber
	* @return 				: None
*/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI PR Register (Pending register Corresponding to the Pin Number (Pending register)
	if(EXTI->EXTI_PR & (1<<PinNumber))
	{
		// clear
		EXTI->EXTI_PR |= (1<<PinNumber);
	}
}




