/*
 * stm3f407xx.h
 *
 *  Created on: Oct 10, 2019
 *      Author: Matt
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#include<stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))


/*********************************
 * ARM Cortex M4: NVIC reg addr
 *********************************/
/*
 * NVIC ISERx reg addr
 */
#define NVIC_ISER0			(__vo uint32_t*)0xE000E100
#define NVIC_ISER1			(__vo uint32_t*)0xE000E104
#define NVIC_ISER2			(__vo uint32_t*)0xE000E108
#define NVIC_ISER3			(__vo uint32_t*)0xE000E10C

/*
 * NVIC ICERx reg addr
 */
#define NVIC_ICER0			(__vo uint32_t*)0xE000E180
#define NVIC_ICER1			(__vo uint32_t*)0xE000E184
#define NVIC_ICER2			(__vo uint32_t*)0xE000E188
#define NVIC_ICER3			(__vo uint32_t*)0xE000E18C

/*
 * NVIC Priority Register
 */
#define NVIC_PR_BASEADDR	(__vo uint32_t*)0xE000E400

/*
 * arm m4 number of priority bits implemented in Priority Reg
 */
#define NO_PR_BITS_IMPLEMENTED		4
/*********************************/



/*
 *
 *  base addresses for peripherals
 *
 */

/*
 * base address of flash and SRAM memories
 */
#define FLASH_BASEADDR			0x00000000U
#define SRAM1_BASEADDR			0x20000000U // Main internal SRAM1 (112 KB)
#define SRAM2_BASEADDR			0x20001C00U
#define ROM_BASEADDR			0X1FFF0000U // system memory
#define SRAM					SRAM1_BASEADD

/*
 * base address of bus peripheral AHBx and APBx
 */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * base addr of peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * base addr of peripherals on APB1
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

/*
 * base addr of peripherals of APB2
 */
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400)
#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)


/*
 *
 * clock enable macros for xx peripherals
 *
 *
 */

/*
 * clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

/*
 * clock enable macro for SPIx peripheral
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1<<20))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1<<21))

/*
 * clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * clock enable macros for USARTx peripherals
 */
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define USART4_PCLK_EN() 	    (RCC->APB1ENR |= (1<<19))
#define USART5_PCLK_EN() 	    (RCC->APB1ENR |= (1<<20))

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5))


/*
 * clock enable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))

/*
 *
 * clock disable macros for xx peripherals
 *
 *
 */

/*
 * clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

/*
 * clock disable macro for SPIx peripheral
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1<<20))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<21))

/*
 * clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

/*
 * clock disable macros for USARTx peripherals
 */
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI() 	    (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() 	    (RCC->APB1ENR &= ~(1<<20))

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))

/*
 * clock disable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))

/*
 * macro to reset GPIOx peripheral
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)

/*
 * macro to reset SPIx peripheral
 */
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)
#define SPI4_REG_RESET()		do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13)); }while(0)
#define SPI5_REG_RESET()		do{(RCC->APB2RSTR |= (1<<20)); (RCC->APB2RSTR &= ~(1<<20)); }while(0)
#define SPI6_REG_RESET()		do{(RCC->APB2RSTR |= (1<<21)); (RCC->APB2RSTR &= ~(1<<21)); }while(0)


/*
 * macro return 0 to 8 for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)		  ( (x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
											(x == GPIOD)?3:\
											(x == GPIOE)?4:\
											(x == GPIOF)?5:\
											(x == GPIOG)?6:\
											(x == GPIOH)?7:\
											(x == GPIOI)?8:0  )


/*
 *
 * peripheral register definition structures
 *
 */

/*
 * GPIO Register struct
 */
typedef struct
{
	__vo uint32_t MODER; // 	offset 0x00		GPIO port mode register
	__vo uint32_t OTYPER; //	offset 0x04		GPIO port output type register
	__vo uint32_t OSPEEDR; //	offset 0x08		GPIO port output speed register
	__vo uint32_t PUPDR; // 	offset 0x0C		GPIO port pull-up/pull-down register
	__vo uint32_t IDR; // 		offset 0x10		GPIO port input data register
	__vo uint32_t ODR; // 		offset 0x14		GPIO port output data register
	__vo uint32_t BSRR; // 		offset 0x18		GPIO port bit set/reset register
	__vo uint32_t LCKR; // 		offset 0x1C		GPIO port configuration lock register
	__vo uint32_t AFR[2]; // 	offset 0x20    	AFR[0]=GPIO alternate function low register
                          // 	offset 0x24		AFR[1]=GPIO alternate function high register

} GPIO_RegDef_t;

/*
 * SPI Register struct
 */
typedef struct
{
	__vo uint32_t CR1; // 	offset 0x00		SPI control register 1
	__vo uint32_t CR2; //	offset 0x04		SPI control register 2
	__vo uint32_t SR; //	offset 0x08		SPI status register
	__vo uint32_t DR; // 	offset 0x0C		SPI data register
	__vo uint32_t CRCPR; // offset 0x10		SPI CRC polynomial register
	__vo uint32_t RXCRCR; //offset 0x14		SPI RX CRC register
	__vo uint32_t TXCRCR; //offset 0x18		SPI TX CRC register
	__vo uint32_t I2SCFGR; //offset 0x1C	SPI_I2S configuration register
	__vo uint32_t I2SPR; // offset 0x20    	SPI_I2S prescaler register

} SPI_RegDef_t;

/*
 * RCC Register struct
 */
typedef struct
{
	__vo uint32_t CR; // 		offset 0x00		RCC clock control register
	__vo uint32_t PLLCFGR; // 	offset 0x04 	RCC PLL configuration register
	__vo uint32_t CFGR; // 		offset 0x08 	RCC clock configuration register
	__vo uint32_t CIR; // 		offset 0x0C 	RCC clock interrupt register
	__vo uint32_t AHB1RSTR; // 	offset 0x10 	RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR; //	offset 0x14 	RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR; // 	offset 0x18 	RCC AHB3 peripheral reset register
	uint32_t Reserved1; // 		offset 0x1C 	--------------------------
	__vo uint32_t APB1RSTR; // 	offset 0x20 	RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR; // 	offset 0x24 	RCC APB2 peripheral reset register
	uint32_t Reserved2; // 		offset 0x28 	--------------------------
	uint32_t Reserved3; // 		offset 0x2C 	--------------------------
	__vo uint32_t AHB1ENR; // 	offset 0x30		RCC AHB1 peripheral clock register
	__vo uint32_t AHB2ENR; // 	offset 0x34 	RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR; // 	offset 0x38 	RCC AHB3 peripheral clock enable register
	uint32_t Reserved4; // 		offset 0x3C  	--------------------------
	__vo uint32_t APB1ENR; // 	offset 0x40 	RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR; // 	offset 0x44 	RCC APB2 peripheral clock enable register
	uint32_t Reserved5; // 		offset 0x48 	--------------------------
	uint32_t Reserved6; // 		offset 0x4C 	--------------------------
	__vo uint32_t AHB1LPENR; // offset 0x50 	RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR; // offset 0x54 	RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR; // offset 0x58 	RCC AHB3 peripheral clock enable in low power mode register
	uint32_t Reserved7; // 		offset 0x5C 	--------------------------
	__vo uint32_t APB1LPENR; // offset 0x60 	RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR; // offset 0x64 	RCC APB2 peripheral clock enabled in low power mode register
	uint32_t Reserved8; // 		offset 0x68 	--------------------------
	uint32_t Reserved9; // 		offset 0x6C 	--------------------------
	__vo uint32_t BDCR; // 		offset 0x70 	RCC Backup domain control register
	__vo uint32_t CSR; // 		offset 0x74 	RCC clock control & status register
	uint32_t Reserved10; // 	offset 0x78 	--------------------------
	uint32_t Reserved11; // 	offset 0x7C 	--------------------------
	__vo uint32_t SSCGR; // 	offset 0x80 	RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR; //offset 0x84 	RCC PLLI2S configuration register
	__vo uint32_t PLLSAICFGR; //offset 0x88 	RCC PLL configuration register
	__vo uint32_t DCKCFGR; // 	offset 0x8C 	RCC Dedicated Clock Configuration Register

}RCC_RegDef_t;

/*
 * EXTI Register struct
 */
typedef struct
{
	__vo uint32_t IMR; //	offset 0x00			Interrupt mask register
	__vo uint32_t EMR; //	offset 0x04			Event mask register
	__vo uint32_t RTSR; //	offset 0x08			Rising trigger selection register
	__vo uint32_t FTSR; //	offset 0x0c			Falling trigger selection register
	__vo uint32_t SWIER; //	offset 0x10			Software interrupt event register
	__vo uint32_t PR; //	offset 0x14			Pending register

}EXTI_RegDef_t;

/*
 * SYSCFG Register struct
 */
typedef struct
{
	__vo uint32_t MEMRMP; //	offset 0x00			SYSCFG memory remap register
	__vo uint32_t PMC; //		offset 0x04			SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4]; //	offset 0x08-0x14	SYSCFG external interrupt configuration register 1-4
	uint32_t RESERVED1[2]; //	offset 0x18-0x1C	--------------------------
	__vo uint32_t CMPCR; //		offset 0x20			Compensation cell control register
	uint32_t RESERVED2[2]; //	offset 0x24-0x28	--------------------------
	__vo uint32_t CFGR; //		offset 0x2c			????????

}SYSCFG_RegDef_t;

/*
 * I2C register struct
 */
typedef struct
{
	__vo uint32_t	CR1;
	__vo uint32_t	CR2;
	__vo uint32_t	OAR1;
	__vo uint32_t	OAR2;
	__vo uint32_t	DR;
	__vo uint32_t	SR1;
	__vo uint32_t	SR2;
	__vo uint32_t	CCR;
	__vo uint32_t	TRISE;
	__vo uint32_t	FLTR;

}I2C_RegDef_t;

/*
 * USART register struct
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */

} USART_RegDef_t;

/*
 *
 * peripherals typecasted xx_RegDef_t
 *
 */

/*
 * GPIOx peripheral base addresses typcasted to GPIO_RegDef_t
 */
#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 * SPIx peripheral base addresses typcasted to SPI_RegDef_t
 */
#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5				((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6				((SPI_RegDef_t*)SPI6_BASEADDR)

/*
 * I2Cx peripheral base addresses typcasted to I2C_RegDef_t
 */
#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)


/*
 * RCC, SYSCFG and EXTI peripheral base addresses typcasted to XXX_RegDef_t
 */
#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 *
 * IRQ (interrupt request) numbers for stm32f407x MCU
 *
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
#define IRQ_NO_SPI6			86

#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0   	 0
#define NVIC_IRQ_PRI15 		 15

// generic macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


/******************** bit postion defn of XX peripheral **************************/
/*
 * bit pos. defn SPI_CR1 reg
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * bit pos. defn SPI_CR2 reg
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


/*
 * bit pos. defn SPI_SR reg
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*
 * bit pos. defn I2C_CR1 reg
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		14

/*
 * bit pos. defn I2C_CR2 reg
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFFEN	10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * bit pos. defn I2C_OAR1 reg
 */
#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_ADDMODE	15

/*
 * bit pos. defn I2C_OAR2 reg
 */
#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2		1

/*
 * bit pos. defn I2C_DR reg
 */
#define I2C_DR_DR			0

/*
 * bit pos. defn I2C_SR1 reg
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * bit pos. defn I2C_SR2 reg
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 * bit pos. defn I2C_CRR reg
 */
#define I2C_CRR_CCR			0
#define I2C_CRR_DUTY		14
#define I2C_CRR_FS			15

/*
 * bit pos. defn I2C_TRISE reg
 */
#define I2C_TRISE_TRISE			0

/*
 * bit pos. defn I2C_FLTR reg
 */
#define I2C_FLTR_DNF		0
#define I2C_FLTR_ANOFF		4

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
