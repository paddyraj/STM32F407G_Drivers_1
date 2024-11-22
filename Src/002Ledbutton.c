/*
 * 002Ledbutton.c
 *
 *  Created on: Nov 22, 2024
 *      Author: Raj paddy
 */

#include "STM32F4xx.h"
#include "STM32F407_gpio_driver.h"
#include <stdint.h>


#define HIGH 1
#define BTN_P HIGH

void delay(void){
		for(uint32_t i = 0; i < 500000/2; i++);

	}
	int main(void){

		GPIO_Handle_t ledwbutton;

		ledwbutton.pGPIOx = GPIOA;
		ledwbutton.GPIO_PinConfig->GPIOPinNumber = GPIO_PIN_0;
		ledwbutton.GPIO_PinConfig->GPIOMode = GPIO_MODE_IN;
		ledwbutton.GPIO_PinConfig->GPIOSpeed = GPIO_MODE_F_SPEED;
		ledwbutton.GPIO_PinConfig->GPIO_pull_Up_Dwn =GPIO_MODE_NOPP;
		GPIO_PClk_Ctr(GPIOA, ENABLE);
		GPIO_Init(&ledwbutton);

		while(1){
			if(GPIO_Read_From_Input_Pin(GPIOA, GPIO_PIN_0)== BTN_P){
				delay();
				GPIO_Toggle_Pin(GPIOD,GPIO_PIN_12);
			}
		}
	}
