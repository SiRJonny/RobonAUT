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

void UART_Initialize()
{
	UART_HandleTypeDef  UART_Handle;
	//UART_HandleTypeDef  UART_Handle2;


	// interrupts
	/*HAL_NVIC_SetPriority(USART2_IRQn,14,0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);*/


		// usart2 init
		UART_Handle.Instance = USART2;
		UART_Handle.Init.BaudRate = 9600;
		UART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
		UART_Handle.Init.StopBits = UART_STOPBITS_1;
		UART_Handle.Init.Parity = UART_PARITY_NONE;
		UART_Handle.Init.Mode = UART_MODE_TX;//_RX;
		UART_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		UART_Handle.Init.OverSampling = UART_OVERSAMPLING_8;

		HAL_UART_Init(&UART_Handle);



		UART_Handle.Instance = USART1;
		UART_Handle.Init.BaudRate = 9600;
		UART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
		UART_Handle.Init.StopBits = UART_STOPBITS_1;
		UART_Handle.Init.Parity = UART_PARITY_NONE;
		UART_Handle.Init.Mode = UART_MODE_TX_RX;
		UART_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		UART_Handle.Init.OverSampling = UART_OVERSAMPLING_16;



		UART_Handle.State = HAL_UART_STATE_RESET;

		HAL_UART_Init(&UART_Handle);


}

/*void HAL_UART_MspInit(UART_HandleTypeDef *husart)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(husart-> Instance==USART2)
	  {
	    __USART2_CLK_ENABLE();
	    __GPIOA_CLK_ENABLE();

	       //USART2 GPIO Configuration
	       //PA2   ------> USART2_TX
	       //PA3   ------> USART2_RX
	    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;                 //Transceiver's R is Hi-Z when !RE=1
	    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  }

	if(husart->Instance == USART1)
	{
		__USART1_CLK_ENABLE();
		__GPIOA_CLK_ENABLE();


		GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;

		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	}
}
*/










