/*
 * STM32F407_gpio_driver.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Raj paddy
 */

#include "Stm32f4xx.h"
#include "STM32F407_gpio_driver.h"


	/*
	 * Peripheral Clock Setup
	 * */

	/***************************************************************************************
	 * @fn 					- GPIO_PClk_Ctr
	 *
	 * @brief 				- This function enables or disables peripheral clock for given
	 * 						  GPIO port
	 *
	 * @param[in]			- base address of the GPIO port
	 *
	 * @param[in]			- ENABLE OR DISABLE Macros.
	 *
	 * @param[in]			- Not Defined
	 *
	 * @return				- None
	 *
	 * @note				- None
	 *
	 * ****************************************************************************************/

	void GPIO_PClk_Ctr(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi){


	}

	/*
	 * Initialize , De-Initialize
	 * */

	/***************************************************************************************
	 * @fn 					- GPIO_PClk_Ctr
	 *
	 * @brief 				-
	 *
	 *
	 *
	 * @param[in]			-
	 *
	 * @param[in]			-
	 *
	 * @param[in]			-
	 *
	 *
	 * @return				-
	 *
	 *
	 * @note				-
	 *
	 *
	 *
	 *
	 * ****************************************************************************************/
	void GPIO_Init(GPIO_Handle_t *pGPIOHandle){


	}

	/***************************************************************************************
	 * @fn 					- GPIO_PClk_Ctr
	 *
	 * @brief 				-
	 *
	 *
	 *
	 * @param[in]			-
	 *
	 * @param[in]			-
	 *
	 * @param[in]			-
	 *
	 *
	 * @return				-
	 *
	 *
	 * @note				-
	 *
	 *
	 *
	 *
	 * ****************************************************************************************/
	void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){


	}

	/*
	 * Data Read and Write
	 * */

	uint8_t GPIO_Read_From_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){


	}
	uint16_t GPIO_Read_From_Input_Port(GPIO_RegDef_t *pGPIOx){



	}
	void GPIO_To_Output_Port(GPIO_RegDef_t *pGPIOx , uint8_t Value){

	}
	void GPIO_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint_8t PinNumber,uint8_t Value){

	}
	void GPIO_Toggle_Pin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber ){

	}

	/*
	 * Interrupts Requests
	 * */

	void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t ORQPrority , uint8_t EnorDi){

	}
	void GPIO_IRQHandling(uint8_t PinNumber){

	}



