/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Feb 12, 2022
 *      Author: rsamonte
 */


#include "stm32f407xx_spi_driver.h"



//helper functions prototype
static void spi_txe_handle_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rnxe_handle_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovrr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral Clock Setup
 */

/********************************************************************
 * @fn						- SPI_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given SPI peripheral
 * @param[in]				- base address of the spi peripheral
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}

	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}

	}

}



/*
 * SPI Initialization and De-initialization
 */

/********************************************************************
 * @fn						- SPI_PeriClockControl
 *
 * @brief					- This function initialize SPI peripheral
 *
 * @param[in]				- base address of the spi peripheral

 *
 * @return					- none
 *
 * @Note					- none
 *
 */


void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//First let's configure the SPI_CR1 register

	uint32_t tempreg = SPI_CR1_CPHA;

	//1. Configure if master/slave
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXonly bit must be set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//5. Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = tempreg;


}

/********************************************************************
 * @fn						- SPI_DeInit
 *
 * @brief					- This function de-initialize SPI peripheral
 *
 * @param[in]				- base address of the spi peripheral

 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * Data Send and Receive
 */


/********************************************************************
 * @fn						- SPI_SendData
 *
 * @brief					- This function Send Data Tx
 * @param[in]				- base address of the spi peripheral
 * @param[in]				- pointer to TX buffer
 * @param[in]				-length of data
 *
 * @return					- none
 *
 * @Note					- This is a blocking code (polling mode)
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//1. Wait until TXE is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1<<SPI_CR1_DFF)) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}


/********************************************************************
 * @fn						- SPI_ReceiveData
 *
 * @brief					- This function enables or disables peripheral clock for the given SPI Port
 *
 * @param[in]				- base address of the SPI peripheral
 * @param[in]				- buffer
 * @param[in]				- Len
 *
 * @return					- none
 *
 * @Note					- none
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//1. Wait until RXE is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		//2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1<<SPI_CR1_DFF)) )
		{
			//16 bit DFF
			//1. load the data from DR to Rx buffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}



/*
 * IRQ Configuration and ISR Handling
 */

/********************************************************************
 * @fn						- SPI_IRQInterruptConfig
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
/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn						- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx) |=  ( IRQPriority << shift_amount );
}


/********************************************************************
 * @fn						- SPI_IRQHandling
 *
 * @brief					- This function takes care if IRQHAndling for ISR
 *
 * @param[in]				- pHandle
 *
 * @return					- none
 *
 * @Note					- none
 *
 */


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//1. check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1<<SPI_SR_TXE );
	temp2 = pHandle->pSPIx->CR2 & ( 1<<SPI_CR2_TXEIE );

	if( temp1 && temp2 )
	{
		//handle TXE
		spi_txe_handle_interrupt_handle(pHandle);

	}

	//2. check for RXE
	temp1 = pHandle->pSPIx->SR & ( 1<<SPI_SR_RXNE );
	temp2 = pHandle->pSPIx->CR2 & ( 1<<SPI_CR2_RXNEIE );

	if( temp1 && temp2 )
	{
		//handle TXE
		spi_rnxe_handle_interrupt_handle(pHandle);

	}

	//3. check for OVR flag
	temp1 = pHandle->pSPIx->SR & ( 1<<SPI_SR_OVR );
	temp2 = pHandle->pSPIx->CR2 & ( 1<<SPI_CR2_ERRIE );

	if( temp1 && temp2 )
	{
		//handle TXE
		spi_ovrr_err_interrupt_handle(pHandle);
	}
}


/********************************************************************
 * @fn						- SPI_SendDataIT
 *
 * @brief					- This function Send Data Tx
 * @param[in]				- base address of the spi peripheral
 * @param[in]				- pointer to TX buffer
 * @param[in]				-length of data
 *
 * @return					- none
 *
 * @Note					- This is a blocking code (polling mode)
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;
}





/********************************************************************
 * @fn						- SPI_ReceiveData
 *
 * @brief					- This function enables or disables peripheral clock for the given SPI Port
 *
 * @param[in]				- base address of the SPI peripheral
 * @param[in]				- buffer
 * @param[in]				- Len
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;

}






/*
 *Other Peripheral Control APIs
 */


/********************************************************************
 * @fn						- SPI_PeripheralControl
 *
 * @brief					- This function enables SPI
 *
 * @param[in]				- base address of the SPI peripheral
 * @param[in]				- ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1<<SPI_CR1_SPE );
	}else
	{
		pSPIx->CR1 &= ~( 1<<SPI_CR1_SPE );
	}

}

/********************************************************************
 * @fn						- SPI_SSIConfig
 *
 * @brief					- This function sets SSI
 *
 * @param[in]				- base address of the SPI peripheral
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1<<SPI_CR1_SSI );
	}else
	{
		pSPIx->CR1 &= ~( 1<<SPI_CR1_SSI );
	}
}

/********************************************************************
 * @fn						- SPI_SSOEConfig
 *
 * @brief					- This function sets SSOE
 *
 * @param[in]				- base address of the SPI peripheral
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= ( 1<<SPI_CR2_SSOE );
	}else
	{
		pSPIx->CR2 &= ~( 1<<SPI_CR2_SSOE );
	}
}

//helper functions
static void spi_txe_handle_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void spi_rnxe_handle_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void spi_ovrr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVRR_ERR);

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}







