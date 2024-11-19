/*
 * STM32F407_gpio_driver.h
 *
 *  Created on: Nov 15, 2024
 *      Author: Raj paddy
 */

#ifndef STM32F407_GPIO_DRIVER_H_
#define STM32F407_GPIO_DRIVER_H_
#include "STM32F4xx.h"
#include <Stdint.h>
typedef struct {
	uint8_t GPIOPinNumber; 				/* Pin Number */
	uint8_t GPIOmode;					/*GPIO Mode*/
	uint8_t GPIOSpeed;					/*GPIO Speed*/
	uint8_t GPIOOutputType;				/*GPIO Output Type*/
	uint8_t GPIO_pull_Up_Dwn;			/*GPIO pull-up or pull-down */
	uint8_t Alt_Fn_Mode;				/*GPIO alternate function mode*/

}GPIO_PinConfig_t;

typedef struct {
	//pointer to hold the base address of the peripheral

	GPIO_RegDef_t *pGPIOx;				/*this holds the base address of the GPIO to which pin it belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/*This holds the GPIO pin Configuration Settings*/

	}GPIO_Handle_t ;

/**********************************************************************************************************
 *		API supported by this driver
 * 		for more information about API, check Function Definitions
 * *********************************************************************************************************/

	/*
	 * Peripheral Clock Setup
	 * */

	void GPIO_PClk_Ctr(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi);

	/*
	 * Initialize , De-Initialize
	 * */

	void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
	void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

	/*
	 * Data Read and Write
	 * */

	uint8_t GPIO_Read_From_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
	uint16_t GPIO_Read_From_Input_Port(GPIO_RegDef_t *pGPIOx);
	void GPIO_To_Output_Port(GPIO_RegDef_t *pGPIOx , uint8_t Value);
	void GPIO_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint_8t PinNumber,uint8_t Value);
	void GPIO_Toggle_Pin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber );

	/*
	 * Interrupts Requests
	 * */

	void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t ORQPrority , uint8_t EnorDi);
	void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* STM32F407_GPIO_DRIVER_H_ */
