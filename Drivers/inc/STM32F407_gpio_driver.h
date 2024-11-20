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

/*
 * @PIN_MODES
 * GPIO possible modes
 * */
#define GPIO_MODE_IN 			0 		//INPUT MODE
#define GPIO_MODE_OUT 			1		//GENRAL PURPOSE OUTPUT MODE
#define GPIO_MODE_ALT_FN 		2		//ALTERNATE FUNCTION MODE
#define GPIO_MODE_ANG_MD 		3		//ANALOG MODE

#define GPIO_MODE_IRQ_FT 		4		//Interrupt falling-egde trigger
#define GPIO_MODE_IRQ_RT		5		//Interrupt rising-egde trigger
#define GPIO_MODE_IRQ_RFT		6		//Interrupt rising and falling-edge trigger



//@OUTPUT_TYPE

#define GPIO_MODE_PP			0		//Output type is push-pull
#define GPIO_MODE_OD			1		//output type is open drain


// @SPEED

#define GPIO_MODE_L_SPEED		0		//Low speed
#define GPIO_MODE_M_SPEED		1		//Medium speed
#define GPIO_MODE_F_SPEED		2		//Fast speed
#define GPIO_MODE_H_SPEED		3		//High Speed

//@PULL_UP_DOWN

#define GPIO_MODE_NOPP			0		//NO pull-up or pull-down
#define GPIO_MODE_PU			1		//Pull-up Configuration
#define GPIO_MODE_PD			3		//Pull-down Configuration


//@Pin_Number

#define GPIO_PIN_0				0		//GPIOx pin number 0
#define GPIO_PIN_1				1		//GPIOx pin number 1
#define GPIO_PIN_2				2		//GPIOx pin number 2
#define GPIO_PIN_3				3		//GPIOx pin number 3
#define GPIO_PIN_4				4		//GPIOx pin number 4
#define GPIO_PIN_5				5		//GPIOx pin number 5
#define GPIO_PIN_6				6		//GPIOx pin number 6
#define GPIO_PIN_7				7		//GPIOx pin number 7
#define GPIO_PIN_8				8		//GPIOx pin number 8
#define GPIO_PIN_9				9		//GPIOx pin number 9
#define GPIO_PIN_10				10		//GPIOx pin number 10
#define GPIO_PIN_11				11		//GPIOx pin number 11
#define GPIO_PIN_12				12		//GPIOx pin number 12
#define GPIO_PIN_13				13		//GPIOx pin number 13
#define GPIO_PIN_14				14		//GPIOx pin number 14
#define GPIO_PIN_15				15		//GPIOx pin number 15




typedef struct {
	uint8_t GPIOPinNumber; 				/*GPIO Pin Number check ref. @Pin_Number*/
	uint8_t GPIOMode;					/*GPIO Mode check ref. @PIN_MODES */
	uint8_t GPIOSpeed;					/*GPIO Speed check ref.@SPEED */
	uint8_t GPIOOutputType;				/*GPIO Output Type check ref. @OUTPUT_TYPE */
	uint8_t GPIO_pull_Up_Dwn;			/*GPIO pull-up or pull-down check ref. @PULL_UP_DOWN_*/
	uint8_t Alt_Fn_Mode;				/*GPIO alternate function mode*/

}GPIO_PinConfig_t;

typedef struct {
	//pointer to hold the base address of the peripheral

	GPIO_RegDef_t *pGPIOx;				/*this holds the base address of the GPIO to which pin it belongs */
	GPIO_PinConfig_t *GPIO_PinConfig;	/*This holds the GPIO pin Configuration Settings*/

	} GPIO_Handle_t ;


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
	void GPIO_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
	void GPIO_Toggle_Pin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber );

	/*
	 * Interrupts Requests
	 * */

	void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t ORQPrority , uint8_t EnorDi);
	void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* STM32F407_GPIO_DRIVER_H_ */
