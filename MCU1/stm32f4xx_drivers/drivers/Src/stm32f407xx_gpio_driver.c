/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 5, 2022
 *      Author: rsamonte
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */

/********************************************************************
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO Port
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}

	}

}

/*
 * Initialization and De-initialization
 */

/********************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function initialize GPIO register
 *
 * @param[in]				- GPIO register configuration structure

 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	//enable peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp=0;		//temp register
	//1. Configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting


	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR

			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure RTSR
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE( pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//3. Enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp=0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;
	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;
	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4* temp2) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2) );
	}
}

/********************************************************************
 * @fn						- GPIO_DeInit
 *
 * @brief					- This function de-initialize GPIO register
 *
 * @param[in]				- GPIO address
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if(pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

/*
 * Data Read and Write
 */

/********************************************************************
 * @fn						- GPIO_readFromInputPin
 *
 * @brief					- This function read PinNumber in input data register
 *
 * @param[in]				- pGPIOx and PinNumber
 *
 * @return					- value
 *
 * @Note					- moving PinNumber to LSB and reading it by masking the rest but LSB
 *
 */
uint8_t GPIO_readFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}

/********************************************************************
 * @fn						- GPIO_readFromInputPort
 *
 * @brief					- This function read the Port
 *
 * @param[in]				- GPIOx

 * @return					- value
 *
 * @Note					- none
 *
 */
uint16_t GPIO_readFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}

/********************************************************************
 * @fn						- GPIO_writeToOutputPin
 *
 * @brief					- This function Set or Reset an output pin of a GPIO
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- Pin Number
 * @param[in]				- Set or Reset
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1<< PinNumber);

	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1<< PinNumber);
	}
}

/********************************************************************
 * @fn						- GPIO_writeToOutputPort
 *
 * @brief					- This function Set or Reset a port
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- Set or Reset
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/********************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- Toggles an output pin
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- Pin number
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1<<PinNumber );
}



/*
 * IRQ Configuration and ISR Handling
 */

/********************************************************************
 * @fn						- GPIO_IRQInterruptConfig
 *
 * @brief					- This function enables or disables IRQ set/clear register
 *
 * @param[in]				- IRQ number
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/********************************************************************
 * @fn						- GPIO_IRQPriorityConfig
 *
 * @brief					- chose the right IRQPriority register and position
 *
 * @param[in]				- IRQNumber
 * @param[in]				- IRQPriority
 * @return					- none
 *
 * @Note					- none
 *
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx) |=  ( IRQPriority << shift_amount );
}
/********************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					- This function takes care if IRQHAndling for ISR
 *
 * @param[in]				- PinNumber
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}

}


