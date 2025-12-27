/*
 * stm32f401xx.h
 *
 *  Created on: Dec 26, 2025
 *      Author: yasbh
 *
 *
 * Bare-metal register definitions for STM32F401xx (base addresses, register maps, clock control)
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

#define __vo volatile									// HW register qualifier (prevents unwanted compiler optimizations)


/******************************** Memory Base Addresses ********************************/

#define FLASH_BASEADDR						0x08000000U		// Flash base (user application code)
#define SRAM_BASEADDR						0x20000000U		// SRAM base (stack/heap/runtime data)
#define ROM_BASEADDR						0x1FFF0000U		// System memory base (bootloader)


/****************************** Peripheral Bus Domains *********************************/

#define PERIPH_BASE							0x40000000U		// Peripheral base address
#define APB1PERIPH_BASEADDR					PERIPH_BASE		// APB1 domain base address
#define APB2PERIPH_BASEADDR					0x40010000U		// APB2 domain base address
#define AHB1PERIPH_BASEADDR					0x40020000U		// AHB1 domain base address
#define AHB2PERIPH_BASEADDR					0x50000000U		// AHB2 domain base address


/******************************* Peripheral Base Addresses ******************************/

/* AHB1 peripherals (GPIO, RCC, DMA, ...) */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000U)	// GPIO Port A base
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400U)	// GPIO Port B base
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800U)	// GPIO Port C base
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00U)	// GPIO Port D base
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000U)	// GPIO Port E base
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00U)	// GPIO Port H base
#define CRC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3000U)	// CRC unit base
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800U)	// Reset & Clock Control base
#define FLASH_R_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3C00U)	// Flash interface register base
#define DMA1_BASEADDR						(AHB1PERIPH_BASEADDR + 0x6000U)	// DMA1 controller base
#define DMA2_BASEADDR						(AHB1PERIPH_BASEADDR + 0x6400U)	// DMA2 controller base

/* APB2 peripherals (high-speed peripheral domain) */
#define TIM1_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000U)	// TIM1 base
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000U)	// USART1 base
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400U)	// USART6 base
#define ADC1_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000U)	// ADC1 base
#define SDIO_BASEADDR						(APB2PERIPH_BASEADDR + 0x2C00U)	// SDIO base
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000U)	// SPI1 base
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400U)	// SPI4 base
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800U)	// System configuration controller base
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00U)	// External interrupt/event controller base
#define TIM9_BASEADDR						(APB2PERIPH_BASEADDR + 0x4000U)	// TIM9 base
#define TIM10_BASEADDR						(APB2PERIPH_BASEADDR + 0x4400U)	// TIM10 base
#define TIM11_BASEADDR						(APB2PERIPH_BASEADDR + 0x4800U)	// TIM11 base

/* APB1 peripherals (general peripheral domain) */
#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000U)	// TIM2 base
#define TIM3_BASEADDR						(APB1PERIPH_BASEADDR + 0x0400U)	// TIM3 base
#define TIM4_BASEADDR						(APB1PERIPH_BASEADDR + 0x0800U)	// TIM4 base
#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00U)	// TIM5 base
#define RTC_BKP_BASEADDR					(APB1PERIPH_BASEADDR + 0x2800U)	// RTC & backup domain register base
#define WWDG_BASEADDR						(APB1PERIPH_BASEADDR + 0x2C00U)	// Window watchdog base
#define IWDG_BASEADDR						(APB1PERIPH_BASEADDR + 0x3000U)	// Independent watchdog base
#define I2S2EXT_BASEADDR					(APB1PERIPH_BASEADDR + 0x3400U)	// I2S2 extended interface base
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800U)	// SPI2 base
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00U)	// SPI3 base
#define I2S3EXT_BASEADDR					(APB1PERIPH_BASEADDR + 0x4000U)	// I2S3 extended interface base
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400U)	// USART2 base
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400U)	// I2C1 base
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800U)	// I2C2 base
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00U)	// I2C3 base
#define PWR_BASEADDR						(APB1PERIPH_BASEADDR + 0x7000U)	// Power control base


/***************************** Peripheral Register Definitions **************************/

/* GPIO register map (offsets from GPIOx base address) */
typedef struct
{
	__vo uint32_t MODER;									/*!< GPIO mode register,                Offset: 0x00 */
	__vo uint32_t OTYPER;									/*!< GPIO output type register,         Offset: 0x04 */
	__vo uint32_t OSPEEDR;									/*!< GPIO output speed register,        Offset: 0x08 */
	__vo uint32_t PUPDR;									/*!< GPIO pull-up/pull-down register,   Offset: 0x0C */
	__vo uint32_t IDR;										/*!< GPIO input data register,          Offset: 0x10 */
	__vo uint32_t ODR;										/*!< GPIO output data register,         Offset: 0x14 */
	__vo uint32_t BSRR;										/*!< GPIO bit set/reset register,       Offset: 0x18 */
	__vo uint32_t LCKR;										/*!< GPIO configuration lock register,  Offset: 0x1C */
	__vo uint32_t AFR[2];									/*!< Alternate function registers,       Offsets: 0x20, 0x24 */
} GPIO_RegDef_t;

/* RCC register map (offsets from RCC base address) */
typedef struct
{
	__vo uint32_t CR;										/*!< Clock control register,                    Offset: 0x00 */
	__vo uint32_t PLLCFGR;									/*!< PLL configuration register,                Offset: 0x04 */
	__vo uint32_t CFGR;										/*!< Clock configuration register,              Offset: 0x08 */
	__vo uint32_t CIR;										/*!< Clock interrupt register,                  Offset: 0x0C */
	__vo uint32_t AHB1RSTR;									/*!< AHB1 peripheral reset register,            Offset: 0x10 */
	__vo uint32_t AHB2RSTR;									/*!< AHB2 peripheral reset register,            Offset: 0x14 */
	uint32_t      RESERVED0[2];								/*!< Reserved,                                  Offset: 0x18–0x1C */
	__vo uint32_t APB1RSTR;									/*!< APB1 peripheral reset register,            Offset: 0x20 */
	__vo uint32_t APB2RSTR;									/*!< APB2 peripheral reset register,            Offset: 0x24 */
	uint32_t      RESERVED1[2];								/*!< Reserved,                                  Offset: 0x28–0x2C */
	__vo uint32_t AHB1ENR;									/*!< AHB1 peripheral clock enable register,     Offset: 0x30 */
	__vo uint32_t AHB2ENR;									/*!< AHB2 peripheral clock enable register,     Offset: 0x34 */
	uint32_t      RESERVED2[2];								/*!< Reserved,                                  Offset: 0x38–0x3C */
	__vo uint32_t APB1ENR;									/*!< APB1 peripheral clock enable register,     Offset: 0x40 */
	__vo uint32_t APB2ENR;									/*!< APB2 peripheral clock enable register,     Offset: 0x44 */
	uint32_t      RESERVED3[2];								/*!< Reserved,                                  Offset: 0x48–0x4C */
	__vo uint32_t AHB1LPENR;									/*!< AHB1 low-power clock enable register,      Offset: 0x50 */
	__vo uint32_t AHB2LPENR;									/*!< AHB2 low-power clock enable register,      Offset: 0x54 */
	uint32_t      RESERVED4[2];								/*!< Reserved,                                  Offset: 0x58–0x5C */
	__vo uint32_t APB1LPENR;									/*!< APB1 low-power clock enable register,      Offset: 0x60 */
	__vo uint32_t APB2LPENR;									/*!< APB2 low-power clock enable register,      Offset: 0x64 */
	uint32_t      RESERVED5[2];								/*!< Reserved,                                  Offset: 0x68–0x6C */
	__vo uint32_t BDCR;										/*!< Backup domain control register,            Offset: 0x70 */
	__vo uint32_t CSR;										/*!< Control/status register,                   Offset: 0x74 */
	uint32_t      RESERVED6[2];								/*!< Reserved,                                  Offset: 0x78–0x7C */
	__vo uint32_t SSCGR;									/*!< Spread spectrum clock generation register, Offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;								/*!< PLLI2S configuration register,             Offset: 0x84 */
	uint32_t      RESERVED7;									/*!< Reserved,                                  Offset: 0x88 */
	__vo uint32_t DCKCFGR;									/*!< Dedicated clocks configuration register,   Offset: 0x8C */
} RCC_RegDef_t;


/******************************** Peripheral Pointer Macros ********************************/

/* Peripheral pointers used for direct register access */
#define GPIOA							((GPIO_RegDef_t*) GPIOA_BASEADDR)	/* GPIOA register access */
#define GPIOB							((GPIO_RegDef_t*) GPIOB_BASEADDR)	/* GPIOB register access */
#define GPIOC							((GPIO_RegDef_t*) GPIOC_BASEADDR)	/* GPIOC register access */
#define GPIOD							((GPIO_RegDef_t*) GPIOD_BASEADDR)	/* GPIOD register access */
#define GPIOE							((GPIO_RegDef_t*) GPIOE_BASEADDR)	/* GPIOE register access */
#define GPIOH							((GPIO_RegDef_t*) GPIOH_BASEADDR)	/* GPIOH register access */

#define RCC								((RCC_RegDef_t*) RCC_BASEADDR)		/* RCC register access */


/******************************** Peripheral Clock Gate Macros ******************************/

/* Enable peripheral clocks before touching the peripheral registers */
#define GPIOA_PCLK_EN()					(RCC->AHB1ENR |=  (1U << 0))		/* Enable clock for GPIOA */
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |=  (1U << 1))		/* Enable clock for GPIOB */
#define GPIOC_PCLK_EN()					(RCC->AHB1ENR |=  (1U << 2))		/* Enable clock for GPIOC */
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |=  (1U << 3))		/* Enable clock for GPIOD */
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |=  (1U << 4))		/* Enable clock for GPIOE */
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |=  (1U << 7))		/* Enable clock for GPIOH */

#define I2C1_PCLK_EN()					(RCC->APB1ENR |=  (1U << 21))		/* Enable clock for I2C1 */
#define I2C2_PCLK_EN()					(RCC->APB1ENR |=  (1U << 22))		/* Enable clock for I2C2 */
#define I2C3_PCLK_EN()					(RCC->APB1ENR |=  (1U << 23))		/* Enable clock for I2C3 */

#define SPI1_PCLK_EN()					(RCC->APB2ENR |=  (1U << 12))		/* Enable clock for SPI1 */
#define SPI2_PCLK_EN()					(RCC->APB1ENR |=  (1U << 14))		/* Enable clock for SPI2 */
#define SPI3_PCLK_EN()					(RCC->APB1ENR |=  (1U << 15))		/* Enable clock for SPI3 */

#define USART1_PCLK_EN()				(RCC->APB2ENR |=  (1U << 4))		/* Enable clock for USART1 */
#define USART6_PCLK_EN()				(RCC->APB2ENR |=  (1U << 5))		/* Enable clock for USART6 */
#define USART2_PCLK_EN()				(RCC->APB1ENR |=  (1U << 17))		/* Enable clock for USART2 */

#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |=  (1U << 14))		/* Enable clock for SYSCFG (EXTI routing) */


/* Disable clocks to save power when peripheral is not used */
#define GPIOA_PCLK_DI()					(RCC->AHB1ENR &= ~(1U << 0))		/* Disable clock for GPIOA */
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1U << 1))		/* Disable clock for GPIOB */
#define GPIOC_PCLK_DI()					(RCC->AHB1ENR &= ~(1U << 2))		/* Disable clock for GPIOC */
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1U << 3))		/* Disable clock for GPIOD */
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1U << 4))		/* Disable clock for GPIOE */
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1U << 7))		/* Disable clock for GPIOH */

#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 21))		/* Disable clock for I2C1 */
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 22))		/* Disable clock for I2C2 */
#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 23))		/* Disable clock for I2C3 */

#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1U << 12))		/* Disable clock for SPI1 */
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 14))		/* Disable clock for SPI2 */
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1U << 15))		/* Disable clock for SPI3 */

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1U << 4))		/* Disable clock for USART1 */
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1U << 5))		/* Disable clock for USART6 */
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1U << 17))		/* Disable clock for USART2 */

#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1U << 14))		/* Disable clock for SYSCFG */


#define ENABLE  			1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F401XX_H_ */
