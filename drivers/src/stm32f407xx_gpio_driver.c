/*
 * stm32f407xx_gpio_driver.c
 *
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include<stdint.h>

/***********
 * PClk setup
 ************/

/* Function: peripheral clock control
 * Parameters: GPIO base addr, Enable or Disable macro
 * Return: none
 * Note:
 */
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{	GPIOA_PCLK_EN();

		}else if (pGPIOx == GPIOB)
		{		GPIOB_PCLK_EN();

		}else if (pGPIOx == GPIOC)
		{		GPIOC_PCLK_EN();

		}else if (pGPIOx == GPIOD)
		{		GPIOD_PCLK_EN();

		}else if (pGPIOx == GPIOE)
		{		GPIOE_PCLK_EN();

		}else if (pGPIOx == GPIOF)
		{		GPIOF_PCLK_EN();

		}else if (pGPIOx == GPIOG)
		{		GPIOG_PCLK_EN();

		}else if (pGPIOx == GPIOH)
		{		GPIOH_PCLK_EN();

		}
		else if (pGPIOx == GPIOI)
		{		GPIOI_PCLK_EN();

			}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{	GPIOA_PCLK_DI();

		}else if (pGPIOx == GPIOB)
		{		GPIOB_PCLK_DI();

		}else if (pGPIOx == GPIOC)
		{		GPIOC_PCLK_DI();

		}else if (pGPIOx == GPIOD)
		{		GPIOD_PCLK_DI();

		}else if (pGPIOx == GPIOE)
		{		GPIOE_PCLK_DI();

		}else if (pGPIOx == GPIOF)
		{		GPIOF_PCLK_DI();

		}else if (pGPIOx == GPIOG)
		{		GPIOG_PCLK_DI();

		}else if (pGPIOx == GPIOH)
		{		GPIOH_PCLK_DI();

		}
		else if (pGPIOx == GPIOI)
		{		GPIOI_PCLK_DI();

		}
	}

}

/************
 * Initialize GPIO
 ************/

/* Function: initialize GPIO
 * Parameters: GPIO handler (GPIO base addr and GPIO config settings)
 * Return: none
 * Note:
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; // temp register

	GPIO_PClkControl(pGPIOHandle->pGPIOx, ENABLE);

	//config the mode of the gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt modes (0,1,2,3) AND MODER is a 2 bit field for each pin
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing bit field before setting, 0x3 is 11 binary
		pGPIOHandle->pGPIOx->MODER |= temp; // store in actual register
	}else
	{
		// this is for the interrupt modes 4 5 6
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. config the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clearing RTSR just incase
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. config the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clearing FTSR just incase
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. config both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. config the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // returns 0 for GPIOA, 1 for GPIOB... to config SYSCFG_EXTICR reg
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}
	temp = 0;

	//config the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//config the PU PD settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//config the optype
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//config the alt function and AFR is a 4 bit field for each pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) // sets alt functionality from pin mode
	{
		//config the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // for 0(AFR[0]) OR 1(AFR[1])
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // temp2 * 4(AFR is a 4 bit field for each pin)
		pGPIOHandle->pGPIOx->AFR[temp1] |= (0xF << (4 * temp2)); // clearing, 1111 bin is 0xF
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}

}

/* Function: de-initialize GPIO
 * Parameters: GPIO base addr
 * Return: none
 * Note: reset with RCC_AHB1RSTR reg
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{	GPIOA_REG_RESET();

	}else if (pGPIOx == GPIOB)
	{		GPIOB_REG_RESET();

	}else if (pGPIOx == GPIOC)
	{		GPIOC_REG_RESET();

	}else if (pGPIOx == GPIOD)
	{		GPIOD_REG_RESET();

	}else if (pGPIOx == GPIOE)
	{		GPIOE_REG_RESET();

	}else if (pGPIOx == GPIOF)
	{		GPIOF_REG_RESET();

	}else if (pGPIOx == GPIOG)
	{		GPIOG_REG_RESET();

	}else if (pGPIOx == GPIOH)
	{		GPIOH_REG_RESET();

	}
	else if (pGPIOx == GPIOI)
	{		GPIOI_REG_RESET();

		}

}

/************
 * data read and write for pins and ports
 ************/

/* Function: Read From Input Pin
 * Parameters: GPIO base addr, pin number
 * Return: 0 or 1
 * Note:
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); //only care about lsb mask everything else
	return value; // value = 0 or 1
}

/* Function: Read From Input Port
 * Parameters: GPIO base addr
 * Return: 16 bits because 16 pins
 * Note:
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR); // reading entire port
	return value; // value = 0 or 1

}

/* Function: Write To Output Pin
 * Parameters: GPIO base addr, pin number, value(1 or 0)
 * Return: none
 * Note:
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to output data reg for bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/* Function: Write To Output Port
 * Parameters: GPIO base addr, value(1 or 0)
 * Return: none
 * Note:
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/* Function: Toggle Output Pin
 * Parameters: GPIO base addr, pin number
 * Return: none
 * Note:
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/************
 * IRQ Config and ISR handling
 ************/

/* Function: IRQ Config
 * Parameters: IRQNumber, Enable or Disable macro
 * Return: none
 * Note:
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program is ISER0 reg
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program is ISER1 reg
			*NVIC_ISER1 |= (1 << (IRQNumber % 32)); // IRQ32 is in 0th position for ISER1

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program is ISER2 reg
			*NVIC_ISER2 |= (1 << (IRQNumber % 64)); // IRQ64 is in 0th position for ISER2

		}


	}else
	{
		if(IRQNumber <= 31)
		{
			//program is ISCR0 reg
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program is ISCR1 reg
			*NVIC_ICER1 |= (1 << (IRQNumber % 32)); // IRQ32 is in 0th position for ICER1

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program is ISCR2 reg
			*NVIC_ICER2 |= (1 << (IRQNumber % 64)); // IRQ64 is in 0th position for ICER2

		}
	}
}


/* Function: IRQ Priority Config
 * Parameters: IRQNumber, IRQPriority
 * Return: none
 * Note:
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 -  NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/* Function: IRQ Handling
 * Parameters: pin number
 * Return: none
 * Note:
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear EXTI Pending Reg correspoding to pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clearing PR is 1, from datasheet
		EXTI->PR |= (1 << PinNumber);
	}
}



