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
			if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
			}
			else if(pGPIOx == GPIOB)
			{
					GPIOB_PCLK_EN();
			}
			else if(pGPIOx == GPIOB)
			{
					GPIOB_PCLK_EN();
			}
			else if(pGPIOx == GPIOC)
			{
				 	GPIOC_PCLK_EN();
			}
			else if(pGPIOx == GPIOD)
			{
					GPIOD_PCLK_EN();
			}
			else if (pGPIOx == GPIOE)
			{
					GPIOE_PCLK_EN();
			}
			else if (pGPIOx == GPIOF){
					GPIOE_PCLK_EN();
			}
			else if (pGPIOx == GPIOF){
					GPIOF_PCLK_EN();
			}
			else if (pGPIOx == GPIOG){
					GPIOG_PCLK_EN();
			}
			else if (pGPIOx == GPIOH){
					GPIOH_PCLK_EN();
			}
			else if (pGPIOx == GPIOI){
				GPIOI_PCLK_EN();
			}
		}

		else {

			if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
					GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
					GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC)
			{
				 	GPIOC_PCLK_DI();
			}
			else if(pGPIOx == GPIOD)
			{
					GPIOA_PCLK_DI();
			}
			else if (pGPIOx == GPIOE)
			{
					GPIOE_PCLK_DI();
			}
			else if (pGPIOx == GPIOF){
					GPIOE_PCLK_DI();
			}
			else if (pGPIOx == GPIOF){
					GPIOF_PCLK_DI();
			}
			else if (pGPIOx == GPIOG){
					GPIOG_PCLK_DI();
			}
			else if (pGPIOx == GPIOH){
					GPIOH_PCLK_DI();
			}
			else if (pGPIOx == GPIOI){
					GPIOI_PCLK_DI();
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

		uint32_t temp = 0;

		if(pGPIOHandle->GPIO_PinConfig->GPIOMode <= GPIO_MODE_ANG_MD){
			//non - intruupt mode code
			temp = pGPIOHandle->GPIO_PinConfig->GPIOMode << (2*pGPIOHandle->GPIO_PinConfig->GPIOPinNumber );
			pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig->GPIOPinNumber);
			pGPIOHandle->pGPIOx->MODER |= temp;
			temp = 0;
		} else {
			// interuppt code

		}

		//2. configure the speed
		temp = 0;
		temp = pGPIOHandle->GPIO_PinConfig->GPIOSpeed << (2*pGPIOHandle->GPIO_PinConfig->GPIOPinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig->GPIOPinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;
		//3.configure the pull-up or pull-down register
		temp = 0;
		temp= pGPIOHandle->GPIO_PinConfig->GPIO_pull_Up_Dwn	<< (2 * pGPIOHandle->pGPIOx->PUPDR);
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig->GPIOPinNumber);
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		//4. configure the output type
		temp = 0;
		temp = pGPIOHandle->GPIO_PinConfig->GPIOOutputType << (2*pGPIOHandle->pGPIOx->OTYPER);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig->GPIOPinNumber);
		pGPIOHandle->pGPIOx->OTYPER = temp;
		//5. configure the alt-function type

		if(pGPIOHandle->GPIO_PinConfig->GPIOMode == GPIO_MODE_ALT_FN){
		 uint32_t temp1, temp2;
		 temp1 = pGPIOHandle->GPIO_PinConfig->GPIOPinNumber / 8;
		 temp2 = pGPIOHandle->GPIO_PinConfig->GPIOPinNumber % 8;
		 pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig->Alt_Fn_Mode <<(4* temp2);


		}
		}

/***.************************************************************************************
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

		if(pGPIOx == GPIOA){
					GPIOA_REG_RESET();
					}
		else if(pGPIOx == GPIOB)
					{
				GPIOB_REG_RESET();
					}

		else if(pGPIOx == GPIOC)
		{
				GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD)
		{
				GPIOD_REG_RESET();
		}
		else if (pGPIOx == GPIOE)
		{
				GPIOE_REG_RESET();
		}
		else if (pGPIOx == GPIOF)
		{
				GPIOE_REG_RESET();
		}
		else if (pGPIOx == GPIOF)
		{
				GPIOF_REG_RESET();
		}
		else if (pGPIOx == GPIOG)
		{
				GPIOG_REG_RESET();
		}
		else if (pGPIOx == GPIOH)
		{
				GPIOH_REG_RESET();
		}
		else if (pGPIOx == GPIOI){
				GPIOI_REG_RESET();
				}
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

		uint8_t value;
		value = (uint8_t)((pGPIOx->IDR << PinNumber) & 0x00000001);
		return value;
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

		uint16_t value;
		value = (uint16_t)pGPIOx->IDR;
		return value;
	}

/***************************************************************************************
			 * @fn 					- GPIO_To_Output_Port
			 *
			 * @brief 				- GPIO write data to output port
			 *
			 * @param[in]			- GPIO_RegDef_t - GPIO base Address of the given GPIO port
			 *
			 * @param[in]			- value ~ unsigned integer 8bit
			 *
			 * @param[in]			- none
			 *
			 * @return				- void
			 *
			 * @note				- none
			 *
******************************************************************************************/
	void GPIO_To_Output_Port(GPIO_RegDef_t *pGPIOx , uint8_t Value){

		Value = pGPIOx->ODR;

	}
/***************************************************************************************
				 * @fn 					- GPIO_To_Output_Pin
				 *
				 * @brief 				- GPIO write data to output pin Specified
				 *
				 * @param[in]			- GPIO_RegDef_t - GPIO base Address of the given GPIO port
				 *
				 * @param[in]			- PinNumber in unsigned 8 bit
				 *
				 * @param[in]			- Value in unsigned 8 bit
				 *
				 * @return				- void
				 *
				 * @note				- none
				 *
******************************************************************************************/

	void GPIO_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value){

		if (pGPIOx == GPIO_PIN_SET){

			//write 1 to the output pin of GPIO
			pGPIOx->ODR |= (1 << PinNumber);
		}

		else {
			//write 0 to the output pin of GPIO
			pGPIOx->ODR |= (0 << PinNumber);
		}

	}
	void GPIO_Toggle_Pin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber ){

		pGPIOx->ODR ^= (1 << PinNumber);
	}

	/*
	 * Interrupts Requests
	 * */

	void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t ORQPrority , uint8_t EnorDi){

	}
	void GPIO_IRQHandling(uint8_t PinNumber){

	}
