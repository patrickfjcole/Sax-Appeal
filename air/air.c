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
  int run = 0;
  int firstVal;
  for(;;)
  {

	  HAL_ADC_Start_DMA(&hadc1, g_ADCValue, 1);
	  g_ADCValue[0] = *(&hadc1.Instance->DR) / 32;
	  if (run == 0){
		  firstVal = g_ADCValue[0];
		  run = 1;
	  }
	  adcInt = (int)((g_ADCValue[0] - 75) * (128.0/(128-75)));
	  if(adcInt > 1000){
		  adcInt = 0;
	  }

	  if(adcInt >= 10){
		  HAL_GPIO_WritePin(led_g1_GPIO_Port, led_g1_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_g1_GPIO_Port, led_g1_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 20){
	  		  HAL_GPIO_WritePin(led_g2_GPIO_Port, led_g2_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_g2_GPIO_Port, led_g2_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 30){
		  HAL_GPIO_WritePin(led_g3_GPIO_Port, led_g3_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_g3_GPIO_Port, led_g3_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 40){
	  		  HAL_GPIO_WritePin(led_g4_GPIO_Port, led_g4_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_g4_GPIO_Port, led_g4_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 50){
		  HAL_GPIO_WritePin(led_y1_GPIO_Port, led_y1_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_y1_GPIO_Port, led_y1_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 60){
	  		  HAL_GPIO_WritePin(led_y2_GPIO_Port, led_y2_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_y2_GPIO_Port, led_y2_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 70){
		  HAL_GPIO_WritePin(led_y3_GPIO_Port, led_y3_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_y3_GPIO_Port, led_y3_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 80){
	  		  HAL_GPIO_WritePin(led_y4_GPIO_Port, led_y4_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_y4_GPIO_Port, led_y4_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 90){
		  HAL_GPIO_WritePin(led_r1_GPIO_Port, led_r1_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_r1_GPIO_Port, led_r1_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 100){
	  		  HAL_GPIO_WritePin(led_r2_GPIO_Port, led_r2_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_r2_GPIO_Port, led_r2_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 110){
		  HAL_GPIO_WritePin(led_r3_GPIO_Port, led_r3_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_r3_GPIO_Port, led_r3_Pin, GPIO_PIN_RESET);
	  }

	  if(adcInt >= 120){
	  		  HAL_GPIO_WritePin(led_r4_GPIO_Port, led_r4_Pin, GPIO_PIN_SET);
	  }
	  else{
		  HAL_GPIO_WritePin(led_r4_GPIO_Port, led_r4_Pin, GPIO_PIN_RESET);
	  }
  }
  /* USER CODE END StartAirTask */
}
