/*
 * 001led_toggle.c
 *
 *  Created on: Feb 6, 2022
 *      Author: rsamonte
 */


#include "stm32f407xx.h"
#include <string.h>

#if 1
void delay(void)
{
	for(uint32_t i = 0; i<500000/2; i++);
}
#endif

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	//initialize every member element of structure to zero
	memset(&GpioLed,0, sizeof(GpioLed));
	memset(&GpioButton,0, sizeof(GpioButton));

	//LED Init
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);

	//Button Init
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioButton);


	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);
	//IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);


}

void EXTI9_5_IRQHandler(void)
{
	delay();										//delay200ms
	GPIO_IRQHandling(GPIO_PIN_NO_5);				//clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}


