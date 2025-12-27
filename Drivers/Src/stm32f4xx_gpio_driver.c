/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Dec 27, 2025
 *      Author: yasbh
 */

#include "stm32f4xx_gpio_driver.h"

/* ========================= GPIO Clock Control ========================= */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		//To enable the clock
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else
	{
		//To disable the clock
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/* ========================= GPIO Initialization ========================= */

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initialize the GPIO Pin (Mode, Speed, Pull UP or Pull Down, Alternate Function, Output Type)
 *
 * @param[in]         - GPIO Handle with the base address and Pin configs.
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Resets the GPIO Port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}


/* ========================= GPIO Read / Write APIs ========================= */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Read a specific Pin In a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number of the GPIO [0, 1, 2... 15]
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Read a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write to a specific Pin In a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number of the GPIO [0, 1, 2... 15]
 * @param[in]         - Value to Write (SET or RESET)
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Write to a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Value to write (SET or RESET)
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggle the Pin in GPIO port if it's 1 then write 0 if it's 0 then write 1.
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - Pin Number of the GPIO [0, 1, 2... 15]
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}


/* ========================= GPIO Interrupt APIs ========================= */

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}



