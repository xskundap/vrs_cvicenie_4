/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "assignment.h"

void SystemClock_Config(void);
uint8_t check_button_state(GPIO_TypeDef* PORT, uint8_t PIN);

uint8_t switch_state = 0;


int main(void)
{

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  SystemClock_Config();


  /*
   * TASK - configure MCU peripherals so that button triggers external interrupt - EXTI.
   * Button must be connected to the GPIO port C, pin 3.
   * LED must be connected to the GPIO port A and its pin 4.
   *
   * Adjust values of macros defined in "assignment.h".
   * Implement function "checkButtonState" declared in "assignment.h".
   */


  /* Configure external interrupt - EXTI*/

  	  //type your code for EXTI configuration (priority, enable EXTI, setup EXTI for input pin, trigger edge) here:
	  NVIC_SetPriority(EXTI3_IRQn, 2);
  	  NVIC_EnableIRQ(EXTI3_IRQn);
	
	  //SYSCFG->EXTICR[0] &= ~(0xDU << 12U);
	  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
	  //Enable interrupt from EXTI line 3
	  EXTI->IMR |= EXTI_IMR_MR3;
	  //Set EXTI trigger to falling edge
	  EXTI->RTSR &= ~(EXTI_IMR_MR3);
	  EXTI->FTSR |= EXTI_IMR_MR3;


  /* Configure GPIOC-3 pin as an input pin - button */

	  //type your code for GPIO configuration here:
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	  GPIOC->MODER &= ~(GPIO_MODER_MODER3);
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR3_0;


  /* Configure GPIOA-4 pin as an output pin - LED */

	  //type your code for GPIO configuration here:
	  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	  GPIOA->MODER &= ~(GPIO_MODER_MODER4);
	  GPIOA->MODER |= GPIO_MODER_MODER4_0;
	  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4);
	  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);


  while (1)
  {
	  // Modify the code below so it sets/resets used output pin connected to the LED
	  if(switch_state)
	  {
		  GPIOB->BSRR |= GPIO_BSRR_BS_13;
		  for(uint16_t i=0; i<0xFF00; i++){}
		  GPIOB->BRR |= GPIO_BRR_BR_13;
		  for(uint16_t i=0; i<0xFF00; i++){}
	  }
	  else
	  {
		  GPIOB->BRR |= GPIO_BRR_BR_13;
	  }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


uint8_t checkButtonState(GPIO_TypeDef* PORT, uint8_t PIN, uint8_t edge, uint8_t samples_window, uint8_t samples_required)
{
	  uint8_t button_state = 0, timeout = 0;
	  uint32_t ed = edge;
	  volatile uint32_t edge2 = (ed << EXTI_IMR_MR3_Pos);
	
	  //type your code for "checkButtonState" implementation here:
	
	  //if(HAL_GPIO_ReadPin(PORT, PIN)){
	  if(!(PORT->IDR & (1 << PIN))/*LL_GPIO_IsInputPinSet(PORT, PIN)*/){	
		if(EXTI->FTSR == edge2){
			
			if(before){
				//switch_state = 0;
				before = 0;
				return 0;
			}
	  		else{
				//switch_state = 1;
				before = 1;
				return 1;
			}
		}
	}
}



void EXTI3_IRQHandler(void)
{
	if(checkButtonState(GPIO_PORT_BUTTON,
						GPIO_PIN_BUTTON,
						BUTTON_EXTI_TRIGGER,
						BUTTON_EXTI_SAMPLES_WINDOW,
						BUTTON_EXTI_SAMPLES_REQUIRED))
	{
		switch_state ^= 1;
	}

	/* Clear EXTI3 pending register flag */

		//type your code for pending register flag clear here:
	EXTI->PR |= (EXTI_PR_PIF3);
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
