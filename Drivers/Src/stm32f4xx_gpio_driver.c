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
	uint32_t temp = 0;

	//1 . configure the mode of the gpio pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//The non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else
	{
		//Interrupt mode
		//a . pin must be in input mode
		//b . configure the edge trigger
		//c . enable interrupt delivery from peripheral to the processor
		//d . identify the IRQ number on which the processor accepts the interrupt
		//e . configure IRQ Priority
		//f . enable interrupt reception
		//g . implement IRQ handler

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1 . configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1 . configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1 . configure both the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2 . configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp3, temp4;

		temp3 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp4 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp3] |= (portcode << (4 * temp4));

		//3 . enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2 . configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3 . configure the pull up pull down resis
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4 . configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_OPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5 . configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alternate function
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_AltFunMode << (4 * temp2));
	}
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
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
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
	uint8_t value;

	value = (uint8_t) (~(0xfe) & (pGPIOx->IDR >> PinNumber));

	return value;
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
	uint16_t value;

	value = pGPIOx->IDR;

	return value;
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
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else if (Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
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
	pGPIOx->ODR = Value;
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
	pGPIOx->ODR ^= (1 << PinNumber);
}


/* ========================= GPIO Interrupt APIs ========================= */

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;
	if (EnorDi == ENABLE)
	{
		switch (temp1)
		{
		case 0:
			*NVIC_ISER0 |= (1 << temp2);
		case 1:
			*NVIC_ISER1 |= (1 << temp2);
		case 2:
			*NVIC_ISER2 |= (1 << temp2);
		case 3:
			*NVIC_ISER3 |= (1 << temp2);
		case 4:
			*NVIC_ISER4 |= (1 << temp2);
		case 5:
			*NVIC_ISER5 |= (1 << temp2);
		case 6:
			*NVIC_ISER6 |= (1 << temp2);
		case 7:
			*NVIC_ISER7 |= (1 << temp2);
		}
	}else
	{
		switch (temp1)
		{
		case 0:
			*NVIC_ICER0 |= (1 << temp2);
		case 1:
			*NVIC_ICER1 |= (1 << temp2);
		case 2:
			*NVIC_ICER2 |= (1 << temp2);
		case 3:
			*NVIC_ICER3 |= (1 << temp2);
		case 4:
			*NVIC_ICER4 |= (1 << temp2);
		case 5:
			*NVIC_ICER5 |= (1 << temp2);
		case 6:
			*NVIC_ICER6 |= (1 << temp2);
		case 7:
			*NVIC_ICER7 |= (1 << temp2);
		}
	}
}

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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1 . first let's find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section =IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (4 * iprx)) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - IRQHanling with the clearing of the pending bit
 *
 * @param[in]         - PinNumber of the Port we are working with
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}



