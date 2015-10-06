/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "queue.h"
#include "GPIO_setup.h"
#include "stm32f4xx_hal_uart.h"
#include "BT_MSG.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId BT_TaskHandle;
UART_HandleTypeDef huart2;
QueueHandle_t xQueue_BT;
SemaphoreHandle_t xSem_USART_rdy_to_send = 0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask();
void StartButtonTask();
void SendBluetoothTask();
void USART2_UART_Init();
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */


  /* USER CODE BEGIN 2 */
  GPIO_Init();
  USART2_UART_Init();


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  xSem_USART_rdy_to_send = xSemaphoreCreateBinary();

  xQueue_BT = xQueueCreate(30, sizeof(struct BT_MSG*));		// itt, hogy BT tasknak m�r k�sz legyen
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(ButtonTask, StartButtonTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

  osThreadDef(SendBluetoothTask, SendBluetoothTask, osPriorityNormal, 0, 128);
  BT_TaskHandle = osThreadCreate(osThread(SendBluetoothTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/



void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

// uart2 �s hozz�tartoz� interruptok inicializ�l�sa
// az�rt main-ben, mert � kell adni neki a handle-t
void USART2_UART_Init()
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

  HAL_NVIC_SetPriority(USART2_IRQn,14,0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

// hal uart callback, TODO: n�zni, hogy melyik uart lett k�sz?
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	static BaseType_t xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(xSem_USART_rdy_to_send,&xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// uart2 megszak�t�s kezel�, megh�vjuk a HAL irq kezel�j�t
void USART2_IRQHandler(void){
	HAL_UART_IRQHandler(&huart2);
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
// default tastk, csak egy villog� led
void StartDefaultTask()
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(500);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

  }
  /* USER CODE END 5 */ 
}

void StartButtonTask()
{
	uint8_t wasPressed = 0;

	for (;;){

		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)  == 1) {
			wasPressed = 1;
		}

		if (wasPressed){

			struct BT_MSG msg_int;
			struct BT_MSG * msg_int_ptr = &msg_int;
			int2msg(msg_int_ptr,6237468,"int32\n");
			xQueueSend( xQueue_BT, (void*) &msg_int_ptr, ( TickType_t ) 0 );

			struct BT_MSG msg_float;
			struct BT_MSG * msg_float_ptr = &msg_float;
			float2msg(msg_float_ptr,2345.54533,"float32\n");
			xQueueSend( xQueue_BT, (void*) &msg_float_ptr, ( TickType_t ) 0 );

			struct BT_MSG msg_double;
			struct BT_MSG * msg_double_ptr = &msg_double;
			double2msg(msg_double_ptr,2312345.54533,"double32\n");
			xQueueSend( xQueue_BT, (void*) &msg_double_ptr, ( TickType_t ) 0 );


			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // piros led, debug

			wasPressed = 0;
		}
		osDelay(30);
	}
}

// bluetooth k�ld� task, xQueue_BT-n kereszt�l kapja a struct BT_MSG pointereket, abb�l k�ldi az adatokat
// ha minden igaz csak akkor fut, ha t�nyleg van dolga
void SendBluetoothTask()
{
	struct BT_MSG *msg;
	xSemaphoreGive(xSem_USART_rdy_to_send);

	for( ;; )
	{
		if(xQueue_BT != 0)
		{
			if ( xSemaphoreTake(xSem_USART_rdy_to_send,	portMAX_DELAY) == pdTRUE)
			{
				if (xQueueReceive(xQueue_BT, &(msg), portMAX_DELAY))// blokk am�g nincs adat
				{
					HAL_UART_Transmit_IT(&huart2, (uint8_t*) msg->data, msg->size);
				}
			}
		}
	}
}


























