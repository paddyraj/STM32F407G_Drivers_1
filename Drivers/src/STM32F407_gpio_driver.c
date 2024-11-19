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
		if (EnorDi == ENABLE){
			if(GPIOx == GPIOA){
			GPIOA_PCLK_EN();
			}
			else if(GPIOx == GPIOB)
			{
					GPIOB_PCLK_EN();
			}
			else if(GPIOx == GPIOB)
			{
					GPIOB_PCLK_EN();
			}
			else if(GPIOx == GPIOC)
			{
				 	GPIOC_PCLK_EN();
			}
			else if(GPIOx == GPIOD)
			{
					GPIOD_PCLK_EN();
			}
			else if (GPIOx == GPIOE)
			{
					GPIOE_PCLK_EN();
			}
			else if (GPIOX == GPIOF){
					GPIOE_PCLK_EN();
			}
			else if (GPIOX == GPIOF){
					GPIOF_PCLK_EN();
			}
			else if (GPIOX == GPIOG){
					GPIOG_PCLK_EN();
			}
			else if (GPIOX == GPIOH){
					GPIOH_PCLK_EN();
			}
			else if (GPIOX == GPIOI){
			}
		}

		else {

			if(GPIOx == GPIOA){
			GPIOA_PCLK_DI()();
			}
			else if(GPIOx == GPIOB)
			{
					GPIOB_PCLK_DI()();
			}
			else if(GPIOx == GPIOB)
			{
					GPIOB_PCLK_DI()();
			}
			else if(GPIOx == GPIOC)
			{
				 	GPIOC_PCLK_DI()();
			}
			else if(GPIOx == GPIOD)
			{
					GPIOD_PCLK_DI()();
			}
			else if (GPIOx == GPIOE)
			{
					GPIOE_PCLK_DI()();
			}
			else if (GPIOX == GPIOF){
					GPIOE_PCLK_DI()();
			}
			else if (GPIOX == GPIOF){
					GPIOF_PCLK_DI()();
			}
			else if (GPIOX == GPIOG){
					GPIOG_PCLK_DI()();
			}
			else if (GPIOX == GPIOH){
					GPIOH_PCLK_DI()();
			}
			else if (GPIOX == GPIOI){
			}
			}
		}

	/*
	 * Initialize , De-Initialize
	 * */

	/***************************************************************************************
	 * @fn 					- GPIO_Init
	 *
	 * @brief 				- GPIO port initiation function
	 *
	 * @param[in] 			- GPIO_Handle_t - Handler Function to which include base address of GPIO and
	 *						  GPIO configuration type.
	 *
	 * @param[in]			- none
	 *
	 * @param[in]			- none
	 *
	 * @return				- none
	 *
	 * @note				- none
	 *
	 * ****************************************************************************************/
	void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
		//1. configure the mode of GPIO
		//2. configure the speed
		//3. configure the pull-up or pull-down register
		//4. configure the output type
		//5. configure the alt-function type
	}

	/***************************************************************************************
	 * @fn 					- GPIO_DeInit
	 *
	 * @brief 				- GPIO  de-initiatation function
	 *
	 * @param[in]			- GPIO base Address of the given GPIO port
	 *
	 * @param[in]			- none
	 *
	 * @param[in]			- none
	 *
	 *
	 * @return				- none
	 *
	 * @note				- none
	 *
	 * ****************************************************************************************/
	void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){


	}

	/*
	 * Data Read and Write
	 * */
	/***************************************************************************************
		 * @fn 					- GPIO_Read_From_Input_Pin
		 *
		 * @brief 				- GPIO read data from Input pin specified
		 *
		 * @param[in]			- GPIO_RegDef_t - GPIO base Address of the given GPIO port
		 *
		 * @param[in]			- PinNumber - Unsigned integer in 1 Byte for pin number
		 *
		 * @param[in]			- none
		 *
		 * @return				- unsigned integer of 1 byte.
		 *
		 * @note				- none
		 *
		 * ****************************************************************************************/

	uint8_t GPIO_Read_From_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){


	}

	/***************************************************************************************
			 * @fn 					- GPIO_Read_From_Input_Port
			 *
			 * @brief 				- GPIO read data from Input port specified
			 *
			 * @param[in]			- GPIO_RegDef_t - GPIO base Address of the given GPIO port
			 *
			 * @param[in]			- none
			 *
			 * @param[in]			- none
			 *
			 * @return				- unsigned integer of 2 byte
			 *
			 * @note				- none
			 *
			 * ****************************************************************************************/

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



