/*
 * 003Button_Interrupt.c
 *
 *  Created on: Dec 4, 2019
 *      Author: Matt
 */
#include "stm32f407xx.h"
#include <string.h>


#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void) // polling delay about 200ms when sysclk is 16MHz
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn; //create variable for the gpio handle struct
	memset(&GpioLed, 0, sizeof(GpioLed)); // setting every member of struct to 0
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOD; // LED is on PD12
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT; // LED
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    //  PushPull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // bc its PP


	GPIO_PClkControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed); // call API & send the address

	GpioBtn.pGPIOx = GPIOD; // Btn is on PD5
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // interrupt @ falling edge
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // bc external btn


	GPIO_PClkControl(GPIOD, ENABLE);

	GPIO_Init(&GpioBtn); // call API & send the address
	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET); // led off when @ rest

	//IRQ Configs
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15); //PD5 is on EXTI5, priority is 5
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);
}

void EXTI9_5_IRQHandler(void) //find this ISR name in startup file, in vector table
{
	delay(); // ~200ms for debounce
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}


