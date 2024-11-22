/*
 * 001test2led.c
 *
 *  Created on: Nov 21, 2024
 *      Author: Raj paddy
 */

#include "STM32F4xx.h"
#include "STM32F407_gpio_driver.h"
#include <stdint.h>

void delay(void){
		for(uint32_t i = 0; i < 5999999999; i++);

	}
	int main(void){

		GPIO_Handle_t test;
		//GPIO_Init(&test);
		test.pGPIOx = GPIOD;
		test.GPIO_PinConfig->GPIOPinNumber = GPIO_PIN_12;
		test.GPIO_PinConfig->GPIOMode = GPIO_MODE_OUT;
		test.GPIO_PinConfig->GPIOSpeed = GPIO_MODE_F_SPEED;
		test.GPIO_PinConfig->GPIOOutputType = GPIO_MODE_PP;
		test.GPIO_PinConfig->GPIO_pull_Up_Dwn =GPIO_MODE_NOPP;

		GPIO_PClk_Ctr(GPIOD , ENABLE);
		GPIO_Init(&test);
		while (1){
			delay();
			return 0 ;
		}
		return 0;
}
