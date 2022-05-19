/*
 * 001led_toggle.c
 *
 *  Created on: Feb 6, 2022
 *      Author: rsamonte
 */


#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i = 0; i<500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	//LED Init
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);

	//Button Init
	GpioButton.pGPIOx = GPIOB;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioButton);


	while(1)
	{
		if (GPIO_readFromInputPin(GPIOB, GPIO_PIN_NO_12)==1){
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
		}
	}

	return 0;

}
