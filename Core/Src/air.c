/* air.c
 *
 *  Created on: Mar 5, 2020
 *      Author: Aaron
 */
#include "main.h"

void StartAirTask(void *argument)
{
  /* USER CODE BEGIN StartAirTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start_DMA(&hadc1, g_ADCValue, 1);
	  g_ADCValue[0] = *(&hadc1.Instance->DR) / 32;
	  adcInt = (int)((g_ADCValue[0] - 69) * (128.0/59));
	  if(adcInt > 128){
		  adcInt = 0;
	  }
	  if(adcInt >= 5){
		  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
	  }
  }
  /* USER CODE END StartAirTask */
}
