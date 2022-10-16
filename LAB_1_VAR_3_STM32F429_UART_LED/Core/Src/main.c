/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct
	{
		unsigned MessageReady 	: 1;
		unsigned BufferOverrun 	: 1;
	}uart_flag_t;

typedef struct
	{
		uint8_t * 	buffer;
		uint16_t 	start;
		uint16_t 	end;
		uint16_t	size;
		uart_flag_t flag;
	} uart_ring_buff_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_BUFF_LEN 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t btn_state = 0;              //Cтан кнопки
uint8_t btn_cur  = 0;				//поточне значення кнопки
uint8_t btn_prev = 0; 				//попереднє значення кнопки

/*volatile uint8_t counter = 0;		  //рахівник на колбеці У�?РТ
volatile uint8_t buff[16] = { 0, };   //буфер дл�? збереженн�? введеної інформації value
volatile uint8_t value = 0;	          //змінна в котру запи�?ують байт інформаціїї з терміналу
volatile uint8_t flag = 0;            //флаг дл від�?лідковуванн�? введенн�? команд в термінал

int str[255] = { 0, };				  //ма�?ив де зберігають�?�? введені дані з ьерміналу*/
uint8_t FF = 0;

uint8_t buff[R_BUFF_LEN];
uart_ring_buff_t uart_ring;

uint16_t temprt;
float voltag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ring_init(uart_ring_buff_t * uart_ring, uint8_t * buff, uint16_t buff_size);
void ring_clear(uart_ring_buff_t * uart_ring);
void ring_putchar(uart_ring_buff_t * uart_ring, uint8_t ch);
uint8_t ring_getchar(uart_ring_buff_t * uart_ring);
uint8_t ring_get_message(uart_ring_buff_t * uart_ring, uint8_t * string);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t delayms = 100;
	uint8_t tstring[255];
	uint8_t rstring[R_BUFF_LEN + 1];
	char string[7] = "BLINK";

	//volatile uint16_t temprt;
	//volatile uint16_t voltag;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ring_init(&uart_ring, buff, sizeof(buff) / sizeof(buff[0]) ); // Initialize UART receiver ring buffer.
  sprintf((char*)tstring,"UART IT Enter command 'T MCU','V REF' or 'ALL SENS'\r\n");
  HAL_UART_Transmit_IT(&huart3,tstring,strlen((char*)tstring));
  HAL_UART_Receive_IT(&huart3,uart_ring.buffer,1);						 // Start UART receiver in the non blocking mode
  //setvbuf(stdin, NULL, _IONBF, 0); // определение нулевого буфера
  //HAL_UART_Receive_IT(&huart3, &value, 1); //запу�?каем UART по прерыванию
  //printf("PRINT SOMETHING!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //btn_cur = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(!strcmp(string,"T MCU"))
	  {
		  HAL_ADCEx_InjectedStart(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 100);
		  temprt = (HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)/40);
		  voltag = ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)*3/4096);
		  HAL_ADCEx_InjectedStop(&hadc1);

		  sprintf((char*)tstring,"T MCU = %d C\r\n",temprt, rstring);
		  //printf("T MCU = %d C\r\n",temprt);
	  }

	  if (ring_get_message(&uart_ring, rstring))
	  {

		  sscanf((char*)rstring,"%25[TMCUVREFALSNtmcuvrefalsn ]", string);
		  // Transmit (in non blocking mode) back to the UART the last entered line and prompt for the next input
		  sprintf((char*)tstring,"Echo: %s\n"
				  "Enter command 'T MCU','V REF' or 'ALL SENS'\r\n",rstring);
		  HAL_UART_Transmit_IT(&huart3,tstring,strlen((char*)tstring));
	  }


	  if(btn_state == 0)
	  {
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin, 1);
		  HAL_Delay(500);
		  if(btn_state == 0)
		  {
			  HAL_GPIO_WritePin(GPIOB, LD3_Pin, 1);
			  HAL_Delay(500);
			  if(btn_state == 0)
			  {
				  HAL_GPIO_WritePin(GPIOB, LD2_Pin, 0);
				  HAL_Delay(500);
				  if(btn_state == 0)
				  {
					  HAL_GPIO_WritePin(GPIOB, LD3_Pin, 0);
					  HAL_Delay(500);
				  }
			  }
		  }
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // сразу же отключаем прерывания на этом пине
		// либо выполняем какое-то действие прямо тут, либо поднимаем флажок
		HAL_TIM_Base_Start_IT(&htim1); // запускаем таймер
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM1)
	{
		HAL_TIM_Base_Stop_IT(&htim1); // останавливаем таймер
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);  // очищаем бит EXTI_PR (бит прерывания)
		NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // очищаем бит NVIC_ICPRx (бит очереди)
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);   // включаем внешнее прерывание
		//btn_cur = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
		{
			if(btn_state == 0)
			{
				btn_state = 1;
			}
			else btn_state = 0;
		}
		/*if((btn_prev == 0) && (btn_cur != 0))
		{
		}*/
	}
}
//////////////////////////////////////////////////////////UART_IT////////////////////////////////////////////////////////////////
// UART receive interrupt callback function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Check that interrupt caused by UART1
	if (huart == &huart3)
		{
			// Put new character from the UART receiver data register (RDR) to the ring buffer
			ring_putchar(&uart_ring,huart->Instance->DR);                                             //питання RDR чи DR?
			// Set the overrun flag if the message is longer than ring buffer can hold
			if (uart_ring.end == uart_ring.start) uart_ring.flag.BufferOverrun = 1;
			// Set the message ready flag if the end of line character has been received
			if ((uart_ring.buffer[uart_ring.end -1] == '\r') ||
					(uart_ring.buffer[uart_ring.end -1] == '\n'))
				uart_ring.flag.MessageReady = 1;
			// Receive the next character from UART in non blocking mode
			HAL_UART_Receive_IT(&huart3,&uart_ring.buffer[uart_ring.end],1);
		}
}
// Initializes the ring buffer
void ring_init(uart_ring_buff_t * uart_ring, uint8_t * buff, uint16_t buff_size)
{
	uart_ring->buffer = buff;
	uart_ring->size = buff_size;
	ring_clear(uart_ring);
}
// Clears the ring buffer
void ring_clear(uart_ring_buff_t * uart_ring)
{
	uart_ring->start = 0;
	uart_ring->end = 0;
	uart_ring->flag.BufferOverrun = 0;
	uart_ring->flag.MessageReady = 0;
}
// Puts a new character to the ring buffer
void ring_putchar(uart_ring_buff_t * uart_ring, uint8_t ch)
{
	uart_ring->buffer[uart_ring->end++] = ch;
	if (uart_ring->end >= uart_ring->size) uart_ring->end = 0;
}
// Gets one character from the ring buffer
uint8_t ring_getchar(uart_ring_buff_t * uart_ring)
{
	uint8_t ch = uart_ring->buffer[uart_ring->start++];
	if (uart_ring->start >= uart_ring->size) uart_ring->start = 0;
	return ch;
}
// Reads full message from the ring buffer and clears appropriate flags
uint8_t ring_get_message(uart_ring_buff_t * uart_ring, uint8_t * string)
{
	uint16_t char_count = 0;
	// Check if the message has been received
	if (uart_ring->flag.MessageReady)
		{
			if (uart_ring->flag.BufferOverrun)
				{
					uart_ring->start = uart_ring->end;
					uart_ring->flag.BufferOverrun = 0;
				}
			while ((uart_ring->buffer[uart_ring->start] != '\r') &&
						 (uart_ring->buffer[uart_ring->start] != '\n') &&
						 (uart_ring->size != char_count - 1))
				{
					*string =  ring_getchar(uart_ring);
					string++;
					char_count++;
				}
			*string =  ring_getchar(uart_ring);
			string++;
			char_count++;
			*string = '\0';
			uart_ring->flag.MessageReady = 0;
		}
	return char_count;
}

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
