/*
 * GPIO_setup.c
 *
 *  Created on: 2015. okt. 5.
 *      Author: Csabi
 */


#include "GPIO_setup.h"
#include "stm32f4xx_hal_uart.h"


void GPIO_Init(){
	__GPIOD_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	  __GPIOE_CLK_ENABLE();
	  __GPIOC_CLK_ENABLE();
	  __GPIOH_CLK_ENABLE();
	  __GPIOB_CLK_ENABLE();
	  __GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;

	// LEDek
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
							|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// user button
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



}


