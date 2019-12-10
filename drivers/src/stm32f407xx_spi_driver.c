/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 17, 2019
 *      Author: Matt
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include<stdint.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/***********
 * PClk setup
 ************/

/* Function: peripheral clock control
 * Parameters: SPI base addr, Enable or Disable macro
 * Return: none
 * Note:
 */
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{	SPI1_PCLK_EN();

			}else if (pSPIx == SPI2)
			{		SPI2_PCLK_EN();

			}else if (pSPIx == SPI3)
			{		SPI3_PCLK_EN();

			}else if (pSPIx == SPI4)
			{		SPI4_PCLK_EN();

			}else if (pSPIx == SPI5)
			{		SPI5_PCLK_EN();

			}else if (pSPIx == SPI6)
			{		SPI6_PCLK_EN();

			}
		}
		else
		{
			if(pSPIx == SPI1)
			{	SPI1_PCLK_DI();

			}else if (pSPIx == SPI2)
			{		SPI2_PCLK_DI();

			}else if (pSPIx == SPI3)
			{		SPI3_PCLK_DI();

			}else if (pSPIx == SPI4)
			{		SPI4_PCLK_DI();

			}else if (pSPIx == SPI5)
			{		SPI5_PCLK_DI();

			}else if (pSPIx == SPI6)
			{		SPI6_PCLK_DI();

			}
		}

}


/***********
 * Initialize SPI
 ************/
/* Function: initialize SPI
 * Parameters: SPI handler (SPI base addr and SPI config settings)
 * Return: none
 * Note:
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PClkControl(pSPIHandle->pSPIx, ENABLE);

	//config SPI_CR1 reg tempreg
	uint32_t tempreg = 0;

	//1.config the device mode MSTR is the reg postion 2
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR; // 1:Master 0:slave

	//2. config the bus config: Fullduplex, Halfduplex, simplexTx
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidimode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//rxonly bit set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. config spi serial clk speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. config the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. config the CPOL polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. config the CPHA phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg; //save value of tempreg into cr1 reg


}

/* Function: de-initialize SPI
 * Parameters: SPI base addr
 * Return: none
 * Note: reset with RCC_APB2RSTR reg
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{	SPI1_REG_RESET();

	}else if (pSPIx == SPI2)
	{		SPI2_REG_RESET();

	}else if (pSPIx == SPI3)
	{		SPI3_REG_RESET();

	}else if (pSPIx == SPI4)
	{		SPI4_REG_RESET();

	}else if (pSPIx == SPI5)
	{		SPI5_REG_RESET();

	}else if (pSPIx == SPI6)
	{		SPI6_REG_RESET();

	}
}


/***********
 * SPI get the flag status
 ************/
/* Function: SPI get the flag status
 * Parameters: SPI base addr, FlagName: requested flag name (its macro in spidriver.h)
 * Return: 1 or 0
 * Note:
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;



}

/***********
 * data send and receive
 ************/
/* Function: SPI send data
 * Parameters: SPI base addr, a pointer to data, number of bytes Tx'd
 * Return: none
 * Note: a blocking API (waits until all bytes Tx)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait till TXE is set
		while((SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)) == FLAG_RESET)

		//check DFF bit in CR1
		if ( ( pSPIx->CR1 & (1 << SPI_CR1_DFF))) // dff 1 is 16bit, 0 is 8bit
		{
			//16 bit DFF
			//load the data into the data register
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

/* Function: SPI receive data
 * Parameters: SPI base addr, a pointer to data, number of bytes Rx'd
 * Return: none
 * Note: a blocking API (waits until all bytes Rx)
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait till RXNE is set
		while((SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)) == FLAG_RESET)

		//check DFF bit in CR1
		if ( ( pSPIx->CR1 & (1 << SPI_CR1_DFF))) // dff 1 is 16bit, 0 is 8bit
		{
			//16 bit DFF
			//load the data into the data register
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


/* Function: SPI send data w/ Interrupt
 * Parameters: SPI handler (SPI base addr and SPI config settings), a pointer to data, number of bytes Tx'd
 * Return: 1 or 0
 * Note: enables Tx flag for ISR to do Txing
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_TX)
	{
		// save Tx buffer address and Len info in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// mark SPI state as busy in transmission so same SPI peripheral cant be used
		pSPIHandle->TxState = SPI_BUSY_TX;

		// enable TXEIE for when TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// data transmission will be handled by the ISR code

	}

	return state;

}

/* Function: SPI receive data w/ Interrupt
 * Parameters: SPI handler (SPI base addr and SPI config settings), a pointer to data, number of bytes Tx'd
 * Return: 1 or 0
 * Note: enables Rx flag for ISR to do Rxing
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_RX)
	{
		// save Rx buffer address and Len info in global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// mark SPI state as busy in receiving so same SPI peripheral cant be used
		pSPIHandle->RxState = SPI_BUSY_RX;

		// enable RXNEIE for when RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// data reception will be handled by the ISR code

	}

	return state;
}




/***********
 *IRQ config and ISR handling
 ************/
/* Function: SPI IRQ config
 * Parameters: IRQNumber, Enable or Disable macro
 * Return: none
 * Note:
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
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
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
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/* Function: SPI IRQ priority config
 * Parameters: IRQNumber, IRQPriority
 * Return: none
 * Note:
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

		//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );



	uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
	{
		uint8_t state = pSPIHandle->TxState;

		if(state != SPI_BUSY_TX)
		{
			//1 . Save the Tx buffer address and Len information in some global variables
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Len;
			//2.  Mark the SPI state as busy in transmission so that
			//    no other code can take over same SPI peripheral until transmission is over
			pSPIHandle->TxState = SPI_BUSY_TX;

			//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

		}


		return state;
	}
}

/* Function: SPI IRQ handle
 * Parameters: SPI handler (SPI base addr and SPI config settings)
 * Return: none
 * Note:
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2; // to hold status flags

	// check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{	//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{	//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for OVR flag, not doing flags for CRC, MODF, and TI
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{	//handle OVR
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}


/***********
 * other peripheral control api
 ************/

/* Function: SPI peripheral control
 * Parameters: SPI base addr, Enable or Disable macro
 * Return: none
 * Note:
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/* Function: SPI SSI config
 * Parameters: SPI base addr, Enable or Disable macro
 * Return: none
 * Note:
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/* Function: SPI SSOE config
 * Parameters: SPI base addr, Enable or Disable macro
 * Return: none
 * Note:
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * some helper functions implementations
 */


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{	//check DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{	//16bit DFf, load data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{	//8bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;

	}

	if(! pSPIHandle->TxLen) // if TexLen is zero
	{	//TxLen is zero. Close SPI transmission and inform app that Tx is over
		//prevent interrupt by clearing TXEIE bit
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{	//check DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{	//16bit DFf, load data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{	//8bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pRxBuffer;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;

	}

	if(! pSPIHandle->RxLen) // if TexLen is zero
	{	//RxLen is zero. Close SPI receiving and inform app that Rx is over
		//prevent interrupt by clearing RXNEIE bit
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}

}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the OVR flag, from datasheet: read from DR and SR
	if(pSPIHandle->TxState != SPI_BUSY_TX) // only if not busy
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);


}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{	// clear the OVR flag, from datasheet: read from DR and SR
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp; // otherwise you see warnings as temp unused

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{	// this is a weak implementation. app may override this fn to avoid complier issue

}

