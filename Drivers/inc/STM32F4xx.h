/*
 * STM3F4xx.h
 *
 *  Created on: Nov 7, 2024
 *      Author: Raj paddy
 */

#ifndef STM3F4XX_H_
#define STM3F4XX_H_

//#include "STM32F407_gpio_driver.h"
#include <stdint.h>
#define __vo volatile

/*
 * Generic Macros
 * */
#define ENABLE 					1
#define DISBLE 					0
#define SET 					ENABLE
#define RESET 					DISBLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
/*
 *
 * */

#define FLASH_BASE_ADDR			0x08000000UL
#define SRAM1_BASE_ADDR			0x20000000UL
#define SRAM2_BASE_ADDR			0x2001C000UL
#define ROM_BASE_ADDR			0x1FFF0000UL
#define SRAM_BASE_ADDR			SRAM1_BASE_ADDR

//BUS BASE ADDRESS
#define PERIPH_BASE 			0x40000000UL
#define APB1_BASE_ADDR			PERIPH_BASE
#define APB2_BASE_ADDR			0x40010000UL

//AHB BUSES CONFRIGATION
#define AHB1_BASE_ADDR			0x40020000UL
#define AHB2_BASE_ADDR			0x50000000UL


// GPIO confrigation on AHB1 bus

#define GPIOA_BASE_ADDR			(AHB1_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR			(AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR			(AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR			(AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR			(AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR			(AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR			(AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR			(AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR			(AHB1_BASE_ADDR + 0x2000)

//RCC BASE ADDRESS

#define RCC_BASE_ADDR			(AHB1_BASE_ADDR + 3800)

// I2C COMM. ADDRESS( ON APB1 BUS )

#define I2C1_ADDR 				(APB1_BASE_ADDR + 0x5400)
#define I2C2_ADDR 				(APB1_BASE_ADDR + 0x5800)
#define I2C3_ADDR 				(APB1_BASE_ADDR + 0x5C00)

// UART and USART COMM. ADDRESS ( ON APB1 BUS )

#define USART2_ADDR 			(APB1_BASE_ADDR + 0x4400)
#define USART3_ADDR 			(APB1_BASE_ADDR + 0x4800)
#define UART4_ADDR 				(APB1_BASE_ADDR + 0x4C00)
#define UART5_ADDR 				(APB1_BASE_ADDR + 0x5000)
#define UART7_ADDR 				(APB1_BASE_ADDR + 0x7800)
#define UART8_ADDR 				(APB1_BASE_ADDR + 0x7C00)

//USART COMM. ADDRESS ( ON APB2 BUS )

#define UART1_ADDR 				(APB2_BASE_ADDR + 0x1000)
#define UART6_ADDR 				(APB2_BASE_ADDR + 0x1400)

//SPI COMM. ADDRESS ( ON APB1 BUS )

#define SPI2_ADDR 				(APB1_BASE_ADDR + 0x3800)
#define SPI3_ADDR 				(APB1_BASE_ADDR + 0x3C00)

// SPI COMM. ADDRESS ( ON APB2 BUS )

#define SPI1_ADDR 				(APB2_BASE_ADDR + 0x3000)
#define SPI4_ADDR 				(APB2_BASE_ADDR + 0x3400)

// // ExternalTimer ADDRESS ( ON APB2 BUS )

#define EXTI_ADDR				(APB2_BASE_ADDR + 0x3C00)
#define SYSCFG					(APB2_BASE_ADDR + 0x3800)




//*******************Peripheral Definition Structure**********************

//PERIPHERAL LAYER ( Specific to STM32F07xx-DISCO, check REF Manual )

typedef struct
{
	__vo uint32_t MODER;				// mode register
	__vo uint32_t OTYPER;				//output type register
	__vo uint32_t OSPEEDR;				//output speed register
	__vo uint32_t PUPDR;				//port push-pull register
	__vo uint32_t IDR;					//port input data register
	__vo uint32_t ODR;					//port output data register
	__vo uint32_t BSRR;					//port bit set/reset register
	__vo uint32_t LCKR;					//port configuration lock register
	__vo uint32_t AFR[2];				//port alternative function register [0]low and [1]high register

	} GPIO_RegDef_t ;

	/*Peripheral Definitions (peripheral base address type-casted to xxx_RegDef_t)
	 * */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASE_ADDR)




/*
 * 	GPIO RESET Macros
 * */

#define GPIOA_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOC_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOD_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOE_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOF_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOG_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOH_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)
#define GPIOI_REG_RESET()		do{(RCC->RCC_AHB1ENR |= (1<<0));  (RCC->RCC_AHB1ENR &= ~ (1<<0)); }while(0)

typedef struct {						//OFFSET Address

	__vo uint32_t RCC_CR; 					//0x00
	__vo uint32_t RCC_PLL_CFGR;				//0x04
	__vo uint32_t RCC_CFGR;					//0x08
	__vo uint32_t CIR;						//0x0C
	__vo uint32_t RCC_AHB1_RSTR;			//0x10
	__vo uint32_t RCC_AHB2_RSTR;			//0x14
	__vo uint32_t RCC_AHB3_RSTR;			//0x18
	__vo uint32_t reserved0;				//0x1C
	__vo uint32_t RCC_APB1_RSTR;			//0x20
	__vo uint32_t RCC_APB2_RSTR;			//0x24
	__vo uint32_t reserved1;				//0x28
	__vo uint32_t reserved2;				//0x2c
	__vo uint32_t RCC_AHB1ENR;				//0x30
	__vo uint32_t RCC_AHB2ENR;				//0x34
	__vo uint32_t RCC_AHB3ENR;				//0x38
	__vo uint32_t reserved3;				//0x3C
	__vo uint32_t RCC_APB1ENR;				//0x40
	__vo uint32_t RCC_APB2ENR;				//0x44
	__vo uint32_t reserved4;				//0x48
	__vo uint32_t reserved5;				//0x4C
	__vo uint32_t RCC_AHB1LPENR;			//0x50
	__vo uint32_t RCC_AHB2LPENR;			//0x54
	__vo uint32_t RCC_AHB3LPENR;			//0x58
	__vo uint32_t reserved6;				//0x5C
	__vo uint32_t RCC_APB1LPENR;			//0x60
	__vo uint32_t RCC_APB2LPENR;			//0x64
	__vo uint32_t reserved7;				//0x68
	__vo uint32_t reserved8;				//0x6C
	__vo uint32_t RCC_BDCR;					//0x70
	__vo uint32_t RCC_CSR;					//0x74
	__vo uint32_t reserved9;				//0x78
	__vo uint32_t reserved10;				//0x7C
	__vo uint32_t RCC_SS_CGR;				//0x80
	__vo uint32_t RCC_PLLI2SCFGR;			//0x84
	__vo uint32_t RCC_PLLSAICFGR;			//0x88
	__vo uint32_t RCC_DCKCFGR;				//0x8C
	__vo uint32_t RCC_CKGATEENR;			//0x90
	__vo uint32_t RCC_DCKCFGR2;				//0x94
} RCC_RegDef_t;


//RCC RegDef specifications
#define RCC 					((RCC_RegDef_t*)RCC_BASE_ADDR)

/* Clock enable macros for GPIO
*/

#define GPIOA_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<0) 	)
#define GPIOB_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<1)	)
#define GPIOC_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<2)	)
#define GPIOD_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<3)	)
#define GPIOE_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<4)	)
#define GPIOF_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<5)	)
#define GPIOG_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<6)	)
#define GPIOH_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<7)	)
#define GPIOI_PCLK_EN()		(	RCC->RCC_AHB1ENR |= (1<<8)	)


/* Clock disable macros for GPIO
*/

#define GPIOA_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<0)	)
#define GPIOB_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<1)	)
#define GPIOC_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<2)	)
#define GPIOD_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<3)	)
#define GPIOE_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<4)	)
#define GPIOF_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<5)	)
#define GPIOG_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<6)	)
#define GPIOH_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<7)	)
#define GPIOI_PCLK_DI()		(	RCC->RCC_AHB1ENR &= ~(1<<8)	)

// Clock enable macros for SPIx

#define SPI1_PCLK_EN()			(	RCC->RCC_APB2ENR |= (1<<12)	)
#define SPI4_PCLK_EN()			(	RCC->RCC_APB2ENR |= (1<<13)	)
#define SPI2_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<14)	)
#define SPI3_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<15)	)

//Clock disable macros for SPIx

#define SPI1_PCLK_DI()			(	RCC->RCC_APB2ENR &= ~(1<<12)	)
#define SPI4_PCLK_DI()			(	RCC->RCC_APB2ENR &= ~(1<<13)	)
#define SPI2_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<14)	)
#define SPI3_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<15)	)

//Clock enable macros for I2Cx

#define I2C1_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<21)	)
#define I2C2_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<22)	)
#define I2C3_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<23)	)

//Clock disable macros for I2Cx

#define I2C1_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<21)	)
#define I2C2_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<22)	)
#define I2C3_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<23)	)

// Clock enable macros for USARTx and UART
#define USART1_PCLK_EN()		(	RCC->RCC_APB2ENR |= (1<<4)	)
#define USART2_PCLK_EN()		(	RCC->RCC_APB1ENR |= (1<<17)	)
#define USART3_PCLK_EN()		(	RCC->RCC_APB1ENR |= (1<<18)	)
#define UART4_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<19)	)
#define UART5_PCLK_EN()			(	RCC->RCC_APB1ENR |= (1<<20)	)
#define USART6_PCLK_EN()		(	RCC->RCC_APB2ENR |= (1<<5)	)

// Clock Disable macros for USARTx and UART
#define USART1_PCLK_DI()		(	RCC->RCC_APB2ENR &= ~(1<<4)		)
#define USART2_PCLK_DI()		(	RCC->RCC_APB1ENR &= ~(1<<17)	)
#define USART3_PCLK_DI()		(	RCC->RCC_APB1ENR &= ~(1<<18)	)
#define UART4_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<19)	)
#define UART5_PCLK_DI()			(	RCC->RCC_APB1ENR &= ~(1<<20)	)
#define USART6_PCLK_DI()		(	RCC->RCC_APB2ENR &= ~(1<<5)		)

//Clock enable macros for SYSCFG

#define SYSCFG_PCLK_EN()		(	RCC->RCC_APB2ENR |=(1>>14) 	)

//Clock enable macros for SYSCFG

#define SYSCFG_PCLK_DI()		(	RCC->RCC_APB2ENR &= ~(1>>14) 	)


//#include "STM32F407_gpio_driver.h"

#endif /* STM3F4XX_H_ */
