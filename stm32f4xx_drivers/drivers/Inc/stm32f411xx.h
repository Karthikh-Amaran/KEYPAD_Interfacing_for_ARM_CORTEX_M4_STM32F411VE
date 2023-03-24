/*
 * stm32f411xx.h
 *
 *  Created on: Dec 24, 2022
 *      Author: Karthikh Amaran
 */
//#include"stm32f411xx_gpio_driver.h"
//#include"stm32f411xx_spi_drivers.h"
#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>
#include<stddef.h>
#define __weak       __attribute__((weak))
/**************************START: Processor Specific Details*****************************/
/*
 *  ARM Cortex M4 Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0 						((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 						((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 						((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 						((volatile uint32_t*)0xE000E10c)

/*
 *  ARM Cortex M4 Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0 						((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1 						((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2 						((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3 						((volatile uint32_t*)0xE000E18c)

/*
 *  ARM Cortex M4 Processor NVIC IPRx Register Addresses
 */

#define NVIC_PR_BASE_ADDR 				((volatile uint32_t*)0xe000e400)

/*
 *  ARM Cortex Mx Processor number of Priority bits implemented in priority register
 *  This Value 4 Is actually vendor specific and so in ST's case its 4
 *  ***** For Details have a look at the notes of the COURSE 2 Section: Interrupt priority Registers
 */
#define NO_PR_BITS_IMPLEMENTED    		4

// Some generic Macros
#define ENABLE    		1
#define DISABLE   		0
#define SET       		ENABLE
#define RESET     		DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
/*
 *  Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR                   0x08000000U 		/* Base Address of the Flash Memory (Taken from memory Map) */
#define SRAM1_BASEADDR                   0x20000000U 		/* Base Address of the SRAM (Taken from Embedded SRAM Memory Map) */
#define ROM_BASEADDR                     0x1FFF0000U 		/* Base address of the System Memory (Taken from Embedded Flash Memory Map) */
#define SRAM                             SRAM1_BASEADDR
/*
 *  Base addresses of Peripheral Buses
 */
#define PERIPH_BASEADDR	                 0x40000000U
#define APB1PERIPH_BASEADDR              PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR              0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR              0x50000000U
/*
 *  Base addresses Peripherals Hanging across the AHB1 Bus
 */
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR                   (AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR					 (AHB1PERIPH_BASEADDR+0x0c00)
#define GPIOE_BASEADDR					 (AHB1PERIPH_BASEADDR+0x1000)
#define GPIOH_BASEADDR    				 (AHB1PERIPH_BASEADDR+0x1c00)
#define CRC_BASEADDR                     (AHB1PERIPH_BASEADDR+0x3000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR+0x3800)
#define FLASHINTERFACE_BASEADDR          (AHB1PERIPH_BASEADDR+0x3c00)
#define DMA1_BASEADDR					 (AHB1PERIPH_BASEADDR+0x6000)
#define DMA2_BASEADDR					 (AHB1PERIPH_BASEADDR+0x6400)
/*
 *  Base addresses Peripherals Hanging across the APB1 Bus
 */
#define TIM2_BASEADDR                   (APB1PERIPH_BASEADDR+0x0000)
#define TIM3_BASEADDR                   (APB1PERIPH_BASEADDR+0x0400)
#define TIM4_BASEADDR                   (APB1PERIPH_BASEADDR+0x0800)
#define TIM5_BASEADDR                   (APB1PERIPH_BASEADDR+0x0c00)
#define RTCBKP_BASEADDR                 (APB1PERIPH_BASEADDR+0x2800)
#define WWDG_BASEADDR                   (APB1PERIPH_BASEADDR+0x2c00)
#define IWDG_BASEADDR                   (APB1PERIPH_BASEADDR+0x3000)
#define I2S2EXT_BASEADDR                (APB1PERIPH_BASEADDR+0x3400)
#define SPI2I2S2_BASEADDR               (APB1PERIPH_BASEADDR+0x3800)
#define SPI3I2S3_BASEADDR               (APB1PERIPH_BASEADDR+0x3c00)
#define I2S3EXT_BASEADDR                (APB1PERIPH_BASEADDR+0x4000)
#define USART2_BASEADDR                 (APB1PERIPH_BASEADDR+0x4400)
//#define I2C1_BASEADDR                 (APB1PERIPH_BASEADDR+0x5400)
#define I2C1_BASEADDR                   (0x40005400)
#define I2C2_BASEADDR                   (APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR                   (APB1PERIPH_BASEADDR+0x5c00)
#define PWR_BASEADDR                    (APB1PERIPH_BASEADDR+0x7000)
/*
 *  Base addresses Peripherals Hanging across the APB2 Bus
 */
#define TIM1_BASEADDR                   (APB2PERIPH_BASEADDR+0x0000)
#define USART1_BASEADDR                 (APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR                 (APB2PERIPH_BASEADDR+0x1400)
#define ADC1_BASEADDR                   (APB2PERIPH_BASEADDR+0x2000)
#define SDIO_BASEADDR                   (APB2PERIPH_BASEADDR+0x2c00)
#define SPI1I2S1_BASEADDR               (APB2PERIPH_BASEADDR+0x3000)
#define SPI4I2S4_BASEADDR               (APB2PERIPH_BASEADDR+0x3400)
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASEADDR+0x3800)
#define EXTI_BASEADDR                   (APB2PERIPH_BASEADDR+0x3c00)
#define TIM9_BASEADDR                   (APB2PERIPH_BASEADDR+0x4000)
#define TIM10_BASEADDR                  (APB2PERIPH_BASEADDR+0x4400)
#define TIM11_BASEADDR                  (APB2PERIPH_BASEADDR+0x4800)
#define SPI5I2S5_BASEADDR               (APB2PERIPH_BASEADDR+0x5000)
/*
 *  Base addresses Peripherals Hanging across the AHB2 Bus
 */
/**************************PERIPHERAL REGISTER DEFINITION STRUCTURES******************************************/
/*
 * 	Note: Registers of a peripheral are specific to the MCU
 */
/********************************************************************************************/
/************ TO KNOW ABOUT STRUCTURES IN C SEE THE MOTES OF COURSE NO 1 ********************/
/********************************************************************************************/
typedef struct
{
	volatile uint32_t MODER;       /* MODER REGISTER TO SELECT THE MODE OF THE PORT */
	volatile uint32_t OTYPER;      /* OTYPER REGISTER TO SELECT THE OUTPUT TYPE OF THE PORT */
	volatile uint32_t OSPEEDR;	   /* OSPEEDR REGISTER TO SELECT THE OUTPUT SPEED OF THE PORT */
	volatile uint32_t PUPDR;	   /* PUPDR REGISTER TO SELECT THE PULL-UP OR PULL-DOWN OF THE PORT */
	volatile uint32_t IDR;		   /* IDR REGISTER TO GET THE INPUT DATA FROM THE PORT */
	volatile uint32_t ODR;		   /* ODR REGISTER TO PUT THE OUTPUT DATA INTO THE PORT */
	volatile uint32_t BSRR;		   /* BSRR REGISTER - BIT SET/RESET REGISTERS */
	volatile uint32_t LCKR;		   /* LCKR REGISTER - PORT CONFIGURATION LOCK REGISTER */
	volatile uint32_t AFRL;		   /* AFRL REGISTER - ALTERNATIVE FUNCTION LOW REGISTER */
	volatile uint32_t AFRH;   	   /* AFRH REGISTER - ALTERNATIVE FUNCTION HIGH REGISTER */
}GPIO_RegDef_t;
/* To access the elements of these structure we have to create a pointer of data type GPIO_RegDef_t as shown below
 * 		GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *)GPIOA_BASEADDR
 *
 * 		So here to make life easier we are creating MACROS for these too
 * 		Ex: #define ACCESS_GPIOAR (GPIO_RegDef_t *)GPIOA_BASEADDR
 */
#define GPIOA                 (GPIO_RegDef_t *)GPIOA_BASEADDR
#define GPIOB 				  (GPIO_RegDef_t *)GPIOB_BASEADDR
#define GPIOC   			  (GPIO_RegDef_t *)GPIOC_BASEADDR
#define GPIOD   			  (GPIO_RegDef_t *)GPIOD_BASEADDR
#define GPIOE 				  (GPIO_RegDef_t *)GPIOE_BASEADDR
#define GPIOH 				  (GPIO_RegDef_t *)GPIOH_BASEADDR

#define SPI1 				  (SPI_RegDef_t *)SPI1I2S1_BASEADDR
#define SPI2				  (SPI_RegDef_t *)SPI2I2S2_BASEADDR
#define SPI3                  (SPI_RegDef_t *)SPI2I2S2_BASEADDR
#define SPI4                  (SPI_RegDef_t *)SPI2I2S2_BASEADDR
#define SPI5                  (SPI_RegDef_t *)SPI2I2S2_BASEADDR
/*
 *  A structure for registers of RCC
 */
typedef struct
{
	volatile uint32_t RCC_CR;			/* TODO: Explain */
	volatile uint32_t RCC_PLLCFGR;		/* TODO: Explain */
	volatile uint32_t RCC_CFGR;			/* TODO: Explain */
	volatile uint32_t RCC_CIR;			/* TODO: Explain */
	volatile uint32_t RCC_AHB1RSTR;		/* TODO: Explain */
	volatile uint32_t RCC_AHB2RSTR;		/* TODO: Explain */
	volatile uint32_t RES1;				/* TODO: Explain */
	volatile uint32_t RES2;				/* TODO: Explain */
	volatile uint32_t RCC_APB1RSTR;		/* TODO: Explain */
	volatile uint32_t RCC_APB2RSTR;		/* TODO: Explain */
	volatile uint32_t RES3;				/* TODO: Explain */
	volatile uint32_t RES4;				/* TODO: Explain */
	volatile uint32_t RCC_AHB1ENR;		/* TODO: Explain */
	volatile uint32_t RCC_AHB2ENR;		/* TODO: Explain */
	volatile uint32_t RES5;				/* TODO: Explain */
	volatile uint32_t RES6;				/* TODO: Explain */
	volatile uint32_t RCC_APB1ENR;		/* TODO: Explain */
	volatile uint32_t RCC_APB2ENR;		/* TODO: Explain */
	volatile uint32_t RES7;				/* TODO: Explain */
	volatile uint32_t RES8;				/* TODO: Explain */
	volatile uint32_t RCC_AHB1LPENR;	/* TODO: Explain */
	volatile uint32_t RCC_AHB2LPENR;	/* TODO: Explain */
	volatile uint32_t RES9;				/* TODO: Explain */
	volatile uint32_t RES10;			/* TODO: Explain */
	volatile uint32_t RCC_APB1LPENR;	/* TODO: Explain */
	volatile uint32_t RCC_APB2LPENR; 	/* TODO: Explain */
	volatile uint32_t REV_11;			/* TODO: Explain */
	volatile uint32_t REV_12;			/* TODO: Explain */
	volatile uint32_t RCC_BDCR;			/* TODO: Explain */
	volatile uint32_t RCC_CSR;			/* TODO: Explain */
	volatile uint32_t REV_13;			/* TODO: Explain */
	volatile uint32_t REV_14;			/* TODO: Explain */
	volatile uint32_t RCC_SSCGR;		/* TODO: Explain */
	volatile uint32_t RCC_PLLI2SCFGR;	/* TODO: Explain */
	volatile uint32_t REV_15;			/* TODO: Explain */
	volatile uint32_t RCC_DCKCFGR;		/* TODO: Explain */
}RCC_RegDef_t;
/* To access the elements of these structure we have to create a pointer of data type GPIO_RegDef_t as shown below
 * 		GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *)GPIOA_BASEADDR
 *
 * 		So here to make life easier we are creating MACROS for these too
 * 		Ex: #define ACCESS_GPIOAREG (GPIO_RegDef_t *)GPIOA_BASEADDR
 * 		So we can do just this "GPIO_RegDef_t *pGPIOA = ACCESS_GPIOAREG"
 */
#define RCC                ((RCC_RegDef_t *)RCC_BASEADDR)
/*
 *  Clock Enable macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()    				(RCC -> RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()    				(RCC -> RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()    				(RCC -> RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()    				(RCC -> RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()    				(RCC -> RCC_AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()    				(RCC -> RCC_AHB1ENR |= (1<<7))
/*
 *  Clock Enable macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()    				(RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()    				(RCC->RCC_APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()    				(RCC->RCC_APB1ENR |= (1<<23))
/*
 *  Clock Enable macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()    				(RCC->RCC_APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()    				(RCC->RCC_APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()    				(RCC->RCC_APB2ENR |= (1<<20))
#define SPI2_PCLK_EN()    				(RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()    				(RCC->RCC_APB1ENR |= (1<<15))
/*
 *  Clock Enable macros for USARTx Peripherals
 */
/* Only 3 USARTS available in STM32F411VE */
#define USART1_PCLK_EN()    				(RCC->RCC_APB2ENR |= (1<<4))
#define USART6_PCLK_EN()    				(RCC->RCC_APB2ENR |= (1<<5))
#define USART2_PCLK_EN()    				(RCC->RCC_APB1ENR |= (1<<17))
/*
 *  Clock Enable macros for SYSTEM CONFIGURATION CONTROLLER CLOCK ENABLE Peripherals
 */
#define SYSCFG_PCLK_EN()    				(RCC->RCC_APB2ENR |= (1<<14))
/***********************************************************************************/
/*
 *  Clock DISABLE macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<7))
/*
 *  Clock DISABLE macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()    				(RCC->RCC_AHB1ENR &= ~(1<<23))
/*
 *  Clock DISABLE macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI()    				(RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()    				(RCC->RCC_APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()    				(RCC->RCC_APB2ENR &= ~(1<<20))
#define SPI2_PCLK_DI()    				(RCC->RCC_APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()    				(RCC->RCC_APB1ENR &= ~(1<<15))
/*
 *  Clock DISABLE macros for USARTx Peripherals
 */
/* Only 3 USARTS available in STM32F411VE */
#define USART1_PCLK_DI()    			(RCC->RCC_APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()    			(RCC->RCC_APB2ENR &= ~(1<<5))
#define USART2_PCLK_DI()    			(RCC->RCC_APB1ENR &= ~(1<<17))
/*
 *  Clock DISABLE macros for SYSTEM CONFIGURATION CONTROLLER CLOCK ENABLE Peripherals
 */
#define SYSCFG_PCLK_DI()    			(RCC->RCC_APB2ENR &= ~(1<<14))
/*
 *  GPIO Peripheral Reset macros
 */
#define GPIOA_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1<<7)); (RCC->RCC_AHB1RSTR &= ~(1<<7)); }while(0)


/****************************************************************************************************************************/
/************************** These are all for Interrupt Sections of the code ************************************************/
/****************************************************************************************************************************/
/*
 *  IRQ(Interrupt Request) Numbers of STM32F411xx MCU
 */
#define IRQ_NO_EXTI0					6  			/* TODO: Explain */
#define IRQ_NO_EXTI1					7			/* TODO: Explain */
#define IRQ_NO_EXTI2					8		 	/* TODO: Explain */
#define IRQ_NO_EXTI3   					9			/* TODO: Explain */
#define IRQ_NO_EXTI4					10			/* TODO: Explain */
#define IRQ_NO_EXTI9_5					23			/* TODO: Explain */
#define IRQ_NO_EXTI15_10				40			/* TODO: Explain */

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2						36
#define IRQ_NO_SPI3						51
#define IRQ_NO_SPI4						84
#define IRQ_NO_SPI5						85

#define IRQ_NO_I2C1_EV                  31
#define IRQ_NO_I2C1_ER                  32
#define IRQ_NO_I2C2_EV                  33
#define IRQ_NO_I2C2_ER                  34
#define IRQ_NO_I2C3_EV                  79
#define IRQ_NO_I2C3_ER                  80
/*
 *  IRQ PRIORITY VALUE MACROS
 */
#define NVIC_IRQ_PRIORITY_15 			15
/*
 *  Return port-code for the given GPIOx Base address (FOR Button Interrupt stuff)
 */

#define GPIO_BASEADDR_TO_CODE(x) 	   ((x == GPIOA)?  0:\
										(x == GPIOB)?  1:\
										(x == GPIOC)?  2:\
							            (x == GPIOD)?  3:\
										(x == GPIOE)?  4:\
										(x == GPIOH)?  7:0)

/*
 *  A structure for EXTI Configuration Registers
 */
#define EXTI                ((EXTI_RegDef_t *)EXTI_BASEADDR)

typedef struct
{
	volatile uint32_t EXTI_IMR; 		/* TODO: Explain */
	volatile uint32_t EXTI_EMR;			/* TODO: Explain */
	volatile uint32_t EXTI_RTSR;		/* TODO: Explain */
	volatile uint32_t EXTI_FTSR;		/* TODO: Explain */
	volatile uint32_t EXTI_SWIER;		/* TODO: Explain */
	volatile uint32_t EXTI_PR;			/* TODO: Explain */

}EXTI_RegDef_t;


/*
 *  A structure for EXTI Configuration Registers
 */

#define SYSCFG                ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)
typedef struct
{
	volatile uint32_t SYSCFG_MEMRMP;	/* TODO: Explain */
	volatile uint32_t SYSCFG_PMC;		/* TODO: Explain */
	volatile uint32_t SYSCFG_EXTICR[4]; /* TODO: Explain */
	uint32_t RES1[2];					/* TODO: Explain */
	volatile uint32_t SYSCFG_CMPCR;		/* TODO: Explain */
	uint32_t RES2[2];					/* TODO: Explain */
	volatile uint32_t SYSCFG_CFGR;		/* TODO: Explain */

}SYSCFG_RegDef_t;

/*******************************************************************************************/
/*******************************************************************************************/
/****************************** DRIVER FOR SPI INTERFACING *********************************/
/*******************************************************************************************/
/*******************************************************************************************/


typedef struct
{
	volatile uint32_t SPI_CR1;              /* TODO   */
	volatile uint32_t SPI_CR2;              /* TODO   */
	volatile uint32_t SPI_SR;	            /* TODO   */
	volatile uint32_t SPI_DR;	            /* TODO   */
	volatile uint32_t SPI_CRCPR;		    /* TODO   */
	volatile uint32_t SPI_RXCRCR;		    /* TODO   */
	volatile uint32_t SPI_TXCRCR;		    /* TODO   */
	volatile uint32_t SPI_I2SCFGR;		    /* TODO   */
	volatile uint32_t SPI_I2SPR;		    /* TODO   */
}SPI_RegDef_t;
// Macros For the Options available for SPI CR1 Register
#define SPI_CR1_CPHA 			0     /* TODO   */
#define SPI_CR1_CPOL			1     /* TODO   */
#define SPI_CR1_MSTR            2     /* TODO   */
#define SPI_CR1_BR			    3     /* TODO   */
#define SPI_CR1_SPE             6     /* TODO   */
#define SPI_CR1_LSB			    7     /* TODO   */
#define SPI_CR1_SSI			    8     /* TODO   */
#define SPI_CR1_SSM			    9     /* TODO   */
#define SPI_CR1_RXONLY          10    /* TODO   */
#define SPI_CR1_DFF             11    /* TODO   */
#define SPI_CR1_CRCNEXT         12    /* TODO   */
#define SPI_CR1_CRCEN           13    /* TODO   */
#define SPI_CR1_BIDIOE          14    /* TODO   */
#define SPI_CR1_BIDIMODE        15    /* TODO   */
// Macros For the options available for SPI CR2 Register
#define SPI_CR2_RXDMAEN         0     /* TODO   */
#define SPI_CR2_TXDMAEN         1     /* TODO   */
#define SPI_CR2_SSOE            2     /* TODO   */
#define SPI_CR2_FRF             4     /* TODO   */
#define SPI_CR2_ERRIE           5     /* TODO   */
#define SPI_CR2_RXNEIE          6     /* TODO   */
#define SPI_CR2_TXEIE           7     /* TODO   */
// MACROS for the Options available for SPI SR Register
#define SPI_SR_RXNE				0     /* TODO   */
#define SPI_SR_TXE				1     /* TODO   */
#define SPI_SR_CHSIDE			2     /* TODO   */
#define SPI_SR_UDR				3     /* TODO   */
#define SPI_SR_CRCERR			4     /* TODO   */
#define SPI_SR_MODF				5     /* TODO   */
#define SPI_SR_OVR				6     /* TODO   */
#define SPI_SR_BSY				7     /* TODO   */
#define SPI_SR_FRE				8     /* TODO   */
//SPI Peripheral Reset macros
#define SPI1_REG_RESET()               do{ (RCC->RCC_APB2RSTR |= (1<<12)); (RCC->RCC_APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->RCC_APB1RSTR |= (1<<14)); (RCC->RCC_APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->RCC_APB1RSTR |= (1<<15)); (RCC->RCC_APB1RSTR &= ~(1<<15)); }while(0)
#define SPI4_REG_RESET()               do{ (RCC->RCC_APB2RSTR |= (1<<13)); (RCC->RCC_APB2RSTR &= ~(1<<13)); }while(0)
#define SPI5_REG_RESET()               do{ (RCC->RCC_APB2RSTR |= (1<<20)); (RCC->RCC_APB2RSTR &= ~(1<<20)); }while(0)

#define FLAG_RESET               RESET
#define FLAG_SET                 SET


/*******************************************************************************************/
/*******************************************************************************************/
/****************************** DRIVER FOR I2C INTERFACING *********************************/
/*******************************************************************************************/
/*******************************************************************************************/

/*
 *  Peripheral Register Definition structure for I2C
 */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;
/* Base Address Macros */
#define I2C1       ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2       ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3       ((I2C_RegDef_t*)I2C3_BASEADDR)

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_PECERR                  12
#define I2C_SR1_TIMEOUT 				14
#define I2C_SR1_SMBALERT                15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7
/*
 * Bit position definitions I2C_TRISE
 */
#define I2C_TRISE                       0
/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


//I2C Peripheral Reset macros
#define I2C1_REG_RESET()               do{ (RCC->RCC_APB1RSTR |= (1<<21)); (RCC->RCC_APB1RSTR &= ~(1<<21)); }while(0)
#define I2C2_REG_RESET()               do{ (RCC->RCC_APB1RSTR |= (1<<22)); (RCC->RCC_APB1RSTR &= ~(1<<22)); }while(0)
#define I2C3_REG_RESET()               do{ (RCC->RCC_APB1RSTR |= (1<<23)); (RCC->RCC_APB1RSTR &= ~(1<<23)); }while(0)


#include"stm32f411xx_gpio_driver.h"
#include"stm32f411xx_spi_drivers.h"
#include"stm32f411xx_i2c_driver.h"
#endif /* INC_STM32F411XX_H_ */

