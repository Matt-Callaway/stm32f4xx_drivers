/*
 * 002Led_Button.c
 *
 *  Created on: Dec 3, 2019
 *      Author: Matt
 */

#include "stm32f407xx.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void) // polling delay
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn; //create variable for the gpio handle struct

	GpioLed.pGPIOx = GPIOD; // LED is on PD12
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; // LED
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    // for case1: PushPull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // bc its PP


	GPIO_PClkControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed); // call API & send the address

	GpioBtn.pGPIOx = GPIOA; // Btn is on PA0
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN; // Btn
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need


	GPIO_PClkControl(GPIOA, ENABLE);

	GPIO_Init(&GpioBtn); // call API & send the address

	while(1)
	{   // if high(Btn pressed) then toggle w/ delay for 'debouncing'
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12); // toggle API gpio_driver.c

		}
	}

	return 0;
}


