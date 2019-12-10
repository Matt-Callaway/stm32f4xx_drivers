/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Oct 17, 2019
 *      Author: Matt
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

/*
 * Configuration struct for SPIx Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode; // possible modes @SPI_DeviceMode in reg CR1->MSTR
	uint8_t SPI_BusConfig;	// possible modes @SPI_BusConfig
	uint8_t SPI_SclkSpeed;  // possible modes @SPI_SclkSpeed
	uint8_t SPI_DFF;		// possible modes @SPI_DFF
	uint8_t SPI_CPOL;		// possible modes @SPI_CPOL
	uint8_t SPI_CPHA;		// possible modes @SPI_CPHA
	uint8_t SPI_SSM;		// possible modes @SPI_SSM

}SPI_Config_t;

/*
 * handle struct for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx; // spi1-spi6
	SPI_Config_t	SPIConfig;
	uint8_t 		*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;

}SPI_Handle_t;

/*****  modes for SPI_Config_t *****/

/*
 * @SPI_DeviceMode
 * master or slave modes
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 * FullDuplex, HalfDuplex, Simplex receive only
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_SclkSpeed
 * peripheral clock divided by 2->256. on APB1 or APB2 bus speeds for SPIx
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 * data frame format 8bits or 16bits
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPOL
 * Clock polarity. low=0 idle. high=1 idle
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @SPI_CPHA
 * Clock phase. low=rising edge(1st). high=falling edge(2nd)
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 * @SPI_SSM
 * Software slave management.
 */
#define SPI_SSM_EN			1
#define SPI_SsM_DI			0

/*
 *SPI related status flag defn
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/*
 * SPI app state macros
 */
#define SPI_READY		0
#define SPI_BUSY_RX		1
#define	SPI_BUSY_TX		2

/*
 * SPI app state macros
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4


/*
 *
 * APIs supported by this driver
 *
 */
/*
 * PClk setup
 */
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * data send and receive, polling and non-polling(IT)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ config and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * other peripheral control api
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
