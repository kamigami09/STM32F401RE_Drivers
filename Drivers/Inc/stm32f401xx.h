/*
 * stm32f401xx.h
 *
 *  Created on: Dec 26, 2025
 *      Author: yasbh
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

#define __vo volatile


/******************************** Memory Base Addresses ********************************/

#define FLASH_BASEADDR						0x08000000U		// Flash memory base (user application code)
#define SRAM_BASEADDR						0x20000000U		// SRAM base (stack, heap, runtime data)
#define ROM_BASEADDR						0x1FFF0000U		// System memory base (ST bootloader)

/****************************** Peripheral Bus Domains *********************************/

#define PERIPH_BASE							0x40000000U		// Peripheral base address
#define APB1PERIPH_BASEADDR					PERIPH_BASE		// APB1 peripheral base
#define APB2PERIPH_BASEADDR					0x40010000U		// APB2 peripheral base
#define AHB1PERIPH_BASEADDR					0x40020000U		// AHB1 peripheral base
#define AHB2PERIPH_BASEADDR					0x50000000U		// AHB2 peripheral base

/******************************* Peripheral Base Addresses ******************************/

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000U)	// GPIOA base address
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400U)	// GPIOB base address
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800U)	// GPIOC base address
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00U)	// GPIOD base address
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000U)	// GPIOE base address
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00U)	// GPIOH base address
#define CRC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3000U)	// CRC peripheral base
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800U)	// RCC base address
#define FLASH_R_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3C00U)	// Flash interface registers base
#define DMA1_BASEADDR						(AHB1PERIPH_BASEADDR + 0x6000U)	// DMA1 controller base
#define DMA2_BASEADDR						(AHB1PERIPH_BASEADDR + 0x6400U)	// DMA2 controller base

/* APB2 peripherals */
#define TIM1_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000U)	// TIM1 base address
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000U)	// USART1 base address
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400U)	// USART6 base address
#define ADC1_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000U)	// ADC1 base address
#define SDIO_BASEADDR						(APB2PERIPH_BASEADDR + 0x2C00U)	// SDIO base address
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000U)	// SPI1 base address
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400U)	// SPI4 base address
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800U)	// System configuration controller base
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00U)	// EXTI controller base
#define TIM9_BASEADDR						(APB2PERIPH_BASEADDR + 0x4000U)	// TIM9 base address
#define TIM10_BASEADDR						(APB2PERIPH_BASEADDR + 0x4400U)	// TIM10 base address
#define TIM11_BASEADDR						(APB2PERIPH_BASEADDR + 0x4800U)	// TIM11 base address

/* APB1 peripherals */
#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000U)	// TIM2 base address
#define TIM3_BASEADDR						(APB1PERIPH_BASEADDR + 0x0400U)	// TIM3 base address
#define TIM4_BASEADDR						(APB1PERIPH_BASEADDR + 0x0800U)	// TIM4 base address
#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00U)	// TIM5 base address
#define RTC_BKP_BASEADDR					(APB1PERIPH_BASEADDR + 0x2800U)	// RTC and backup registers base
#define WWDG_BASEADDR						(APB1PERIPH_BASEADDR + 0x2C00U)	// Window watchdog base
#define IWDG_BASEADDR						(APB1PERIPH_BASEADDR + 0x3000U)	// Independent watchdog base
#define I2S2EXT_BASEADDR					(APB1PERIPH_BASEADDR + 0x3400U)	// I2S2 extended interface base
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800U)	// SPI2 base address
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00U)	// SPI3 base address
#define I2S3EXT_BASEADDR					(APB1PERIPH_BASEADDR + 0x4000U)	// I2S3 extended interface base
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400U)	// USART2 base address
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400U)	// I2C1 base address
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800U)	// I2C2 base address
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00U)	// I2C3 base address
#define PWR_BASEADDR						(APB1PERIPH_BASEADDR + 0x7000U)	// Power control base

/***************************** Peripheral Register Definitions **************************/


/***************************** GPIO Peripheral Register Definition *****************************
 * GPIO register map (STM32F401xx) – offsets are from GPIOx base address (GPIOA_BASEADDR, etc.).
 * Used to access GPIO configuration and data registers directly via memory-mapped I/O.
 ***********************************************************************************************/

typedef struct
{
	uint32_t MODER;		/*!< GPIO port mode register,               Address offset: 0x00 */
	uint32_t OTYPER;	/*!< GPIO port output type register,        Address offset: 0x04 */
	uint32_t OSPEEDR;	/*!< GPIO port output speed register,       Address offset: 0x08 */
	uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C */
	uint32_t IDR;		/*!< GPIO port input data register,         Address offset: 0x10 */
	uint32_t ODR;		/*!< GPIO port output data register,        Address offset: 0x14 */
	uint32_t BSRR;		/*!< GPIO port bit set/reset register,      Address offset: 0x18 */
	uint32_t LCKR;		/*!< GPIO port configuration lock register, Address offset: 0x1C */
	uint32_t AFR[2];	/*!< AFR[0]: AF low register (0–7)  0x20
	                         AFR[1]: AF high register (8–15) 0x24 */

} GPIO_RegDef_t;


/***************************** RCC Peripheral Register Definition ******************************
 * RCC register map (STM32F401xx)
 * Controls system clocks, resets, and peripheral clock enables.
 * Offsets verified from reference manual RCC register table.
 ***********************************************************************************************/
typedef struct
{
	__vo uint32_t CR;			/*!< Clock control register,                    Offset: 0x00 */
	__vo uint32_t PLLCFGR;		/*!< PLL configuration register,                Offset: 0x04 */
	__vo uint32_t CFGR;			/*!< Clock configuration register,              Offset: 0x08 */
	__vo uint32_t CIR;			/*!< Clock interrupt register,                  Offset: 0x0C */
	__vo uint32_t AHB1RSTR;		/*!< AHB1 peripheral reset register,            Offset: 0x10 */
	__vo uint32_t AHB2RSTR;		/*!< AHB2 peripheral reset register,            Offset: 0x14 */
	uint32_t      RESERVED0[2];/*!< Reserved,                                   Offset: 0x18–0x1C */
	__vo uint32_t APB1RSTR;		/*!< APB1 peripheral reset register,            Offset: 0x20 */
	__vo uint32_t APB2RSTR;		/*!< APB2 peripheral reset register,            Offset: 0x24 */
	uint32_t      RESERVED1[2];/*!< Reserved,                                   Offset: 0x28–0x2C */
	__vo uint32_t AHB1ENR;		/*!< AHB1 peripheral clock enable register,     Offset: 0x30 */
	__vo uint32_t AHB2ENR;		/*!< AHB2 peripheral clock enable register,     Offset: 0x34 */
	uint32_t      RESERVED2[2];/*!< Reserved,                                   Offset: 0x38–0x3C */
	__vo uint32_t APB1ENR;		/*!< APB1 peripheral clock enable register,     Offset: 0x40 */
	__vo uint32_t APB2ENR;		/*!< APB2 peripheral clock enable register,     Offset: 0x44 */
	uint32_t      RESERVED3[2];/*!< Reserved,                                   Offset: 0x48–0x4C */
	__vo uint32_t AHB1LPENR;	/*!< AHB1 low-power clock enable register,      Offset: 0x50 */
	__vo uint32_t AHB2LPENR;	/*!< AHB2 low-power clock enable register,      Offset: 0x54 */
	uint32_t      RESERVED4[2];/*!< Reserved,                                   Offset: 0x58–0x5C */
	__vo uint32_t APB1LPENR;	/*!< APB1 low-power clock enable register,      Offset: 0x60 */
	__vo uint32_t APB2LPENR;	/*!< APB2 low-power clock enable register,      Offset: 0x64 */
	uint32_t      RESERVED5[2];/*!< Reserved,                                   Offset: 0x68–0x6C */
	__vo uint32_t BDCR;			/*!< Backup domain control register,            Offset: 0x70 */
	__vo uint32_t CSR;			/*!< Control/status register,                   Offset: 0x74 */
	uint32_t      RESERVED6[2];/*!< Reserved,                                   Offset: 0x78–0x7C */
	__vo uint32_t SSCGR;		/*!< Spread spectrum clock generation register, Offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;	/*!< PLLI2S configuration register,             Offset: 0x84 */
	uint32_t      RESERVED7;	/*!< Reserved,                                   Offset: 0x88 */
	__vo uint32_t DCKCFGR;		/*!< Dedicated clocks configuration register,   Offset: 0x8C */

} RCC_RegDef_t;



#define GPIOA							((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*) GPIOH_BASEADDR)


#endif /* INC_STM32F401XX_H_ */

