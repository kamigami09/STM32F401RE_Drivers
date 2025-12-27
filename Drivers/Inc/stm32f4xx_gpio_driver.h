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
	uint8_t GPIO_PinNumber;      /* Possible pin numebrs  @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;        /* Possible values from  @GPIO_PIN_MODE */
	uint8_t GPIO_PinSpeed;       /* Possible output speed @GPIO_SPEED */
	uint8_t GPIO_PuPdControl;    /* Possible PUPD modes   @GPIO_PUPD */
	uint8_t GPIO_OPType;         /* Possible Output type  @GPIO_OP_TYPE */
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

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODE
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_SPEED
 * GPIO pin possible output speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_FAST		3

/*
 * @GPIO_PUPD
 * GPIO pin pull up pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
/* Sets the priority level of an interrupt */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
/* Handle GPIO interrupt */
void GPIO_IRQHandling(uint8_t PinNumber);










#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
