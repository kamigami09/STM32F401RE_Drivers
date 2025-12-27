/*
 * stm32f4xx_gpio_driver.h
 *
 * GPIO driver definitions for STM32F4 series
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

/* ============================= Includes ============================= */

#include "stm32f401xx.h"

/* ========================= GPIO Pin Configuration ========================= */

/*
 * Holds configuration parameters for a single GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;      /* GPIO pin number (0–15) */
	uint8_t GPIO_PinMode;        /* Pin mode (input, output, AF, analog, interrupt) */
	uint8_t GPIO_PinSpeed;       /* Output speed */
	uint8_t GPIO_PuPdControl;    /* Pull-up / Pull-down configuration */
	uint8_t GPIO_OPType;         /* Output type (push-pull / open-drain) */
	uint8_t GPIO_AltFunMode;     /* Alternate function selection */

} GPIO_PinConfig_t;

/* ========================= GPIO Handle Structure ========================= */

/*
 * GPIO handle structure
 * Binds GPIO port base address with pin configuration
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       /* Pointer to GPIO port registers */
	GPIO_PinConfig_t GPIO_PinConfig; /* GPIO pin configuration */

} GPIO_Handle_t;


/* ========================= GPIO Clock Control ========================= */

/* Enable or disable GPIO peripheral clock */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/* ========================= GPIO Initialization ========================= */

/* Initialize GPIO pin */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
/* Reset GPIO port */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/* ========================= GPIO Read / Write APIs ========================= */

/* Read logic level from a specific input pin */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/* Read entire input data register */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
/* Write logic level to a specific output pin */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
/* Write value to entire output data register */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
/* Toggle output pin state */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/* ========================= GPIO Interrupt APIs ========================= */

/* Configure GPIO interrupt */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
/* Handle GPIO interrupt */
void GPIO_IRQHandling(uint8_t PinNumber);










#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
