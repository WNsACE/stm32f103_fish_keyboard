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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "fish_keyboard.h"
#include "tim_pwm_led_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

c_timer_manager_t* timer_manager;
tim_pwm_led_stm32_t* tim_pwm_led;
scan_keyboard_t* scan_keyboard_manager;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t fac_us;
void HAL_Delay_us_init(uint8_t SYSCLK) {
     fac_us=SYSCLK; 
}
 
void HAL_Delay_us(uint32_t nus) {
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;
    ticks=nus*fac_us; 
    told=SysTick->VAL; 
    while(1) {
        tnow=SysTick->VAL;
        if(tnow!=told) {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    };
}

//static inline void set_line_semaphore(uint32_t line_id) {
//	uint8_t i = 0;
//	uint8_t n = 0;
//	uint8_t byte[2] = {0};
//	(void)byte;
//	if (line_id > 8) {
//		return;
//	}

//	HAL_GPIO_WritePin(CS_LINE_KEY_GPIO_Port, CS_LINE_KEY_Pin, GPIO_PIN_SET);

//	HAL_GPIO_WritePin(LINE_KEY0_GPIO_Port, LINE_KEY0_Pin, (line_id & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LINE_KEY1_GPIO_Port, LINE_KEY1_Pin, (line_id & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LINE_KEY2_GPIO_Port, LINE_KEY2_Pin, (line_id & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(SL_GPIO_Port, SL_Pin, GPIO_PIN_RESET);
//	HAL_Delay_us(1);
//	HAL_GPIO_WritePin(SL_GPIO_Port, SL_Pin, GPIO_PIN_SET);
//	for (n = 0; n < sizeof(byte); n++) {
//		uint8_t dat = 0;
//		for(i = 0; i < 8; i++) {
//			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
//			if (HAL_GPIO_ReadPin(DAT_GPIO_Port, DAT_Pin) == GPIO_PIN_SET) {
//				dat |= (1 << (7 - i));
//			}
//			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
//		}
//		byte[n] = dat;
//	}

//	HAL_GPIO_WritePin(CS_LINE_KEY_GPIO_Port, CS_LINE_KEY_Pin, GPIO_PIN_RESET);

//	HAL_GPIO_WritePin(LINE_KEY0_GPIO_Port, LINE_KEY0_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LINE_KEY1_GPIO_Port, LINE_KEY1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LINE_KEY2_GPIO_Port, LINE_KEY2_Pin, GPIO_PIN_SET);
//}


#define LINE_KEY_MAX_SIZE					16

static void get_line_semaphore(uint32_t line_id, uint8_t line_key_list[LINE_KEY_MAX_SIZE]) {
	uint8_t n = 0;
	if (line_id > 8) {
		return;
	}

	HAL_GPIO_WritePin(CS_LINE_KEY_GPIO_Port, CS_LINE_KEY_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LINE_KEY0_GPIO_Port, LINE_KEY0_Pin, (line_id & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LINE_KEY1_GPIO_Port, LINE_KEY1_Pin, (line_id & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LINE_KEY2_GPIO_Port, LINE_KEY2_Pin, (line_id & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SL_GPIO_Port, SL_Pin, GPIO_PIN_RESET);
	HAL_Delay_us(1);
	HAL_GPIO_WritePin(SL_GPIO_Port, SL_Pin, GPIO_PIN_SET);
	for (n = 0; n < LINE_KEY_MAX_SIZE; n++) {
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
		line_key_list[LINE_KEY_MAX_SIZE - n - 1] = HAL_GPIO_ReadPin(DAT_GPIO_Port, DAT_Pin) == GPIO_PIN_SET ? TRUE : FALSE;
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(CS_LINE_KEY_GPIO_Port, CS_LINE_KEY_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LINE_KEY0_GPIO_Port, LINE_KEY0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LINE_KEY1_GPIO_Port, LINE_KEY1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LINE_KEY2_GPIO_Port, LINE_KEY2_Pin, GPIO_PIN_SET);
}

static c_bool_t physical_keyboard_get_scan_keys(uint32_t number_line, uint8_t* key_list, uint32_t* key_list_size) {
  *key_list_size = LINE_KEY_MAX_SIZE;
	if (number_line < 2) {
		get_line_semaphore(number_line, key_list);
		return TRUE;
	}
	return FALSE;
}

int8_t USBD_HID_OutEvent(uint8_t event_idx, uint8_t state)
{
  scan_keyboard_set_num_lock_statue(scan_keyboard_manager, event_idx & USB_HID_NUM_LOCK_STATE_UP);
  scan_keyboard_set_caps_lock_statue(scan_keyboard_manager, event_idx & USB_HID_CAPS_LOCK_STATE_UP);
  scan_keyboard_set_scroll_lock_statue(scan_keyboard_manager, event_idx & USB_HID_SCROLL_LOCK_STATE_UP);
  /* test */
	if ((event_idx & USB_HID_NUM_LOCK_STATE_UP)) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
  if ((event_idx & USB_HID_NUM_LOCK_STATE_UP)) {
    HAL_GPIO_WritePin(NUM_LOCK_LED_GPIO_Port, NUM_LOCK_LED_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(NUM_LOCK_LED_GPIO_Port, NUM_LOCK_LED_Pin, GPIO_PIN_SET);
  }
	
	if ((event_idx & USB_HID_CAPS_LOCK_STATE_UP)) {
    HAL_GPIO_WritePin(CAPS_LOCK_LED_GPIO_Port, CAPS_LOCK_LED_Pin, GPIO_PIN_RESET);		
  } else {
    HAL_GPIO_WritePin(CAPS_LOCK_LED_GPIO_Port, CAPS_LOCK_LED_Pin, GPIO_PIN_SET);
  }
	
	if ((event_idx & USB_HID_SCROLL_LOCK_STATE_UP)) {
    HAL_GPIO_WritePin(SCROLL_LOCK_LED_GPIO_Port, SCROLL_LOCK_LED_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(SCROLL_LOCK_LED_GPIO_Port, SCROLL_LOCK_LED_Pin, GPIO_PIN_SET);
  }

  return 0;
}


void scan_keyboard_init_all_lock_statue() {
  uint8_t KeyBoard[8] = {0,0,71,0,0,0,0,0};
  uint8_t KeyBoard01[8] = {0,0,0,0,0,0,0,0};
  
  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&KeyBoard,sizeof(KeyBoard));
  HAL_Delay(15);
  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&KeyBoard01,sizeof(KeyBoard01));
  HAL_Delay(15);
  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&KeyBoard,sizeof(KeyBoard));
  HAL_Delay(15);
  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&KeyBoard01,sizeof(KeyBoard01));
  HAL_Delay(15);
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//  float ADC_ConvertedValue = HAL_ADC_GetValue(hadc) * 3.3f / 4095.0f;
//  if (ADC_ConvertedValue > 2.0f) {
//    ADC_ConvertedValue = 3.0f;
//  } else {
//    ADC_ConvertedValue = 0.0f;
//  }
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  //HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start(&hadc1);
  
  timer_manager = c_timer_manager_init();
  tim_pwm_led = tim_pwm_led_stm32_init(TIM_PWM_LED_TYPE_WS2812_GRB, &htim2, TIM_CHANNEL_2);
  tim_pwm_led_stm32_set_static_color(tim_pwm_led, 0x00, 0x00, 0xff);
  tim_pwm_led_stm32_set_round_hsv_color_model(tim_pwm_led, 1000 * 10);
//  tim_pwm_led_stm32_set_breathing_color(tim_pwm_led, 1000);
//  tim_pwm_led_stm32_set_curr_color(tim_pwm_led, 0xff, 0x0, 0x0);
	scan_keyboard_manager = scan_keyboard_create(NULL, physical_keyboard_get_scan_keys);
  scan_keyboard_init_all_lock_statue();
  
  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    float ADC_ConvertedValue = 0.0f;
		uint8_t keyboard[8] = {0};
    char rx_buffer[100]={0};
    
    HAL_ADC_PollForConversion(&hadc1, 50);
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
			ADC_ConvertedValue = HAL_ADC_GetValue(&hadc1) * 3.3f / 4095.0f;
			sprintf(rx_buffer, "curr:%f ma\r\n", ADC_ConvertedValue * 1000 * 10 / 20);
			HAL_UART_Transmit_DMA(&huart1, (void*)rx_buffer, sizeof(rx_buffer));
		}
    
    c_timer_manager_dispatch_one(timer_manager);
		scan_keyboard_get_usb_keyboard_code(scan_keyboard_manager, keyboard);
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboard, sizeof(keyboard));
		HAL_Delay(10);

//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		set_line_semaphore(1);
//    HAL_Delay(1000);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  tim_pwm_led_stm32_deinit(tim_pwm_led);

	c_timer_manager_deinit(timer_manager);
	scan_keyboard_destroy(scan_keyboard_manager);
  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SL_Pin|CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_LINE_KEY_GPIO_Port, CS_LINE_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LINE_KEY0_Pin|LINE_KEY1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LINE_KEY2_GPIO_Port, LINE_KEY2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SL_Pin CLK_Pin LINE_KEY2_Pin */
  GPIO_InitStruct.Pin = SL_Pin|CLK_Pin|LINE_KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DAT_Pin */
  GPIO_InitStruct.Pin = DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAPS_LOCK_LED_Pin SCROLL_LOCK_LED_Pin NUM_LOCK_LED_Pin */
  GPIO_InitStruct.Pin = CAPS_LOCK_LED_Pin|SCROLL_LOCK_LED_Pin|NUM_LOCK_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_LINE_KEY_Pin LINE_KEY0_Pin LINE_KEY1_Pin */
  GPIO_InitStruct.Pin = CS_LINE_KEY_Pin|LINE_KEY0_Pin|LINE_KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
