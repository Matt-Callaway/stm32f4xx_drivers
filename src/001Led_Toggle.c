/*
 * 001Led_Toggle.c
 *
 *  Created on: Dec 3, 2019
 *      Author: Matt
 */
#include "stm32f407xx.h"

void delay(void) // polling delay
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed; //create variable for the gpio handle struct

	GpioLed.pGPIOx = GPIOD; // LED is on PD12
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; // LED
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    // for case1: PushPull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // bc its PP

 //	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;   // for case2: OpenDrain
//  GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // bc its OD, need PU to avoid floating state
															 // PU is 40k ohms, making LED dim
															// can use smaller external resistor w/o PU to fix!
	GPIO_PClkControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed); // call API & send the address

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12); // toggle API gpio_driver.c
		delay();
	}

	return 0;
}
