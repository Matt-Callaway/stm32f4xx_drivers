/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 11, 2019
 *      Author: Matt
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"
#include<stdint.h>

/*
 * config struct for GPIO pin
 *
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			// possible pin numbers @GPIO_PinNumbers
	uint8_t GPIO_PinMode;			// possible pin modes @GPIO_PinModes
	uint8_t GPIO_PinSpeed;			// possible pin modes @GPIO_PinSpeeds
	uint8_t GPIO_PinPuPdControl;	// possible pin modes @GPIO_PUPD
	uint8_t GPIO_PinOPType;			// possible pin modes @GPIO_PinOPType
	uint8_t GPIO_PinAltFunMode;		// possible pin modes

}GPIO_PinConfig_t;

/*
 * handle struct for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; // holds base addr of gpio port to which pin it belongs to
	GPIO_PinConfig_t GPIO_PinConfig; // holds GPIO pin config settings

}GPIO_Handle_t;

/*
 * @GPIO_PinNumbers
 * GPIO pin possible numbers
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
 * @GPIO_PinModes
 * GPIO pin possible modes, if value is greater than 3, its in interrupt mode
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4  // mode: Input interrupt falling edge
#define GPIO_MODE_IT_RT		5  // mode: Input interrupt rising edge
#define GPIO_MODE_IT_RFT	6  // mode: Input interrupt rising edge, falling edge

/*
 * @GPIO_PinOPType
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0  // push-pull
#define GPIO_OP_TYPE_OD		1  // open-drain

/*
 * @GPIO_PinSpeeds
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD
 * GPIO pin pull-up AND pull-down config
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/*
 *
 * APIs supported by this driver
 *
 */

/*
 * PClk setup
 */
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Config and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
