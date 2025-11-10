/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "input_events.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"      // STM32Cube abstraction over FreeRTOS
#include "queue.h"         // Required for queue functions like xQueueReceive
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi1;

/* Definitions for Input_Task */
osThreadId_t Input_TaskHandle;
const osThreadAttr_t Input_Task_attributes = {
  .name = "Input_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for UI_Control_Task */
osThreadId_t UI_Control_TaskHandle;
const osThreadAttr_t UI_Control_Task_attributes = {
  .name = "UI_Control_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Metronome_Task */
osThreadId_t Metronome_TaskHandle;
const osThreadAttr_t Metronome_Task_attributes = {
  .name = "Metronome_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Audio_Out_Task */
osThreadId_t Audio_Out_TaskHandle;
const osThreadAttr_t Audio_Out_Task_attributes = {
  .name = "Audio_Out_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Screen */
osThreadId_t ScreenHandle;
const osThreadAttr_t Screen_attributes = {
  .name = "Screen",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for inputEventQueue */
osMessageQueueId_t inputEventQueueHandle;
const osMessageQueueAttr_t inputEventQueue_attributes = {
  .name = "inputEventQueue"
};
/* Definitions for defaultScreenQueue */
osMessageQueueId_t defaultScreenQueueHandle;
const osMessageQueueAttr_t defaultScreenQueue_attributes = {
  .name = "defaultScreenQueue"
};
/* Definitions for screenQueue */
osMessageQueueId_t screenQueueHandle;
const osMessageQueueAttr_t screenQueue_attributes = {
  .name = "screenQueue"
};
/* USER CODE BEGIN PV */
int bpm = 120;                 // Default BPM
int volume = 50;                // Default volume, scale e.g. 50–100
int metronome_running = 0;
char msg[64];					//char array for debug messages

typedef struct {
    uint8_t bpm;
    uint8_t volume;
    int is_playing;
} DefaultScreenData_t;

typedef enum {
    SCREEN_SETTINGS,
    SCREEN_DEFAULT,
    SCREEN_SAMPLES
} Screen_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
void Start_Input_Task(void *argument);
void Start_UI_Control_Task(void *argument);
void Start_Metronome_Engine_Task(void *argument);
void Start_Audio_Out_Task(void *argument);
void Screen_Task(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_DisplayStartupLogo();
  OLED_Clear();
  OLED_UpdateScreen();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of inputEventQueue */
  inputEventQueueHandle = osMessageQueueNew (50, 8, &inputEventQueue_attributes);

  /* creation of defaultScreenQueue */
  defaultScreenQueueHandle = osMessageQueueNew (50, 8, &defaultScreenQueue_attributes);

  /* creation of screenQueue */
  screenQueueHandle = osMessageQueueNew (50, 8, &screenQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Input_Task */
  Input_TaskHandle = osThreadNew(Start_Input_Task, NULL, &Input_Task_attributes);

  /* creation of UI_Control_Task */
  UI_Control_TaskHandle = osThreadNew(Start_UI_Control_Task, NULL, &UI_Control_Task_attributes);

  /* creation of Metronome_Task */
  Metronome_TaskHandle = osThreadNew(Start_Metronome_Engine_Task, NULL, &Metronome_Task_attributes);

  /* creation of Audio_Out_Task */
  Audio_Out_TaskHandle = osThreadNew(Start_Audio_Out_Task, NULL, &Audio_Out_Task_attributes);

  /* creation of Screen */
  ScreenHandle = osThreadNew(Screen_Task, NULL, &Screen_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCLK_EN_Pin|CANCEL_LED_Pin|RIGHT_LED_Pin|LEFT_LED_Pin
                          |SELECT_LED_Pin|TEMPO_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, N_FLASH_RESET_Pin|N_AMP_SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : OLED_RESET_Pin */
  GPIO_InitStruct.Pin = OLED_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCLK_EN_Pin CANCEL_LED_Pin RIGHT_LED_Pin LEFT_LED_Pin
                           N_FLASH_RESET_Pin SELECT_LED_Pin TEMPO_LED_Pin N_AMP_SHDN_Pin */
  GPIO_InitStruct.Pin = MCLK_EN_Pin|CANCEL_LED_Pin|RIGHT_LED_Pin|LEFT_LED_Pin
                          |N_FLASH_RESET_Pin|SELECT_LED_Pin|TEMPO_LED_Pin|N_AMP_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : N_LEFT_Pin N_RIGHT_Pin N_SELECT_Pin N_CANCEL_Pin
                           N_CODEC_IRQ_Pin */
  GPIO_InitStruct.Pin = N_LEFT_Pin|N_RIGHT_Pin|N_SELECT_Pin|N_CANCEL_Pin
                          |N_CODEC_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_L_B_Pin ENC_L_A_Pin ENC_R_A_Pin ENC_R_B_Pin */
  GPIO_InitStruct.Pin = ENC_L_B_Pin|ENC_L_A_Pin|ENC_R_A_Pin|ENC_R_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//External int. callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case N_SELECT_Pin:
            button_select_flag = 1;
            break;

        case N_CANCEL_Pin:
            button_stop_flag = 1;
            break;

        case N_LEFT_Pin:
            button_left_flag = 1;
            break;

        case N_RIGHT_Pin:
            button_right_flag = 1;
            break;

        case ENC_L_A_Pin:
        case ENC_L_B_Pin:
            // Read both pins to form current 2-bit state for left encoder
            currentState = (HAL_GPIO_ReadPin(ENC_L_A_GPIO_Port, ENC_L_A_Pin) << 1) |
                           HAL_GPIO_ReadPin(ENC_L_B_GPIO_Port, ENC_L_B_Pin);
            combined = (lastEncLeftState << 2) | currentState;

            switch (combined) {
                case 0b0001:
                case 0b0111:
                case 0b1110:
                case 0b1000:
                    encoder_left_delta = -1;
                    break;
                case 0b0010:
                case 0b0100:
                case 0b1101:
                case 0b1011:
                    encoder_left_delta = +1;
                    break;
                default:
                    // Invalid transition, could be noise, ignore
                    break;
            }
            lastEncLeftState = currentState;
            break;

        case ENC_R_A_Pin:
        case ENC_R_B_Pin:
            // Same for right encoder
            currentState = (HAL_GPIO_ReadPin(ENC_R_A_GPIO_Port, ENC_R_A_Pin) << 1) |
                           HAL_GPIO_ReadPin(ENC_R_B_GPIO_Port, ENC_R_B_Pin);
            combined = (lastEncRightState << 2) | currentState;

            switch (combined) {
                case 0b0001:
                case 0b0111:
                case 0b1110:
                case 0b1000:
                    encoder_right_delta = -1;
                    break;
                case 0b0010:
                case 0b0100:
                case 0b1101:
                case 0b1011:
                    encoder_right_delta = +1;
                    break;
                default:
                    // Noise, ignore
                    break;
            }
            lastEncRightState = currentState;
            break;

        default:
            break;
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Input_Task */
/**
  * @brief  Function implementing the Input_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Input_Task */
void Start_Input_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  InputEvent_t evt;

  //Debounce timestamps (FreeRTOS ticks)
  uint32_t lastSelectPress = 0;
  uint32_t lastStopPress   = 0;
  uint32_t lastLeftPress   = 0;
  uint32_t lastRightPress  = 0;
  uint32_t lastEncLeft	   = 0;
  uint32_t lastEncRight	   = 0;

  const uint32_t debounceDelay = 150; //in ms
  const uint32_t encDebounceDelay = 10;
  /* Infinite loop */
  for(;;)
  {
	  //char msg[] = "Hello from Input_Task!\r\n";
	  //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
      uint32_t now = osKernelGetTickCount();

      if (button_select_flag) {
    	  button_select_flag = 0;
    	  if(now - lastSelectPress > debounceDelay){
    		  lastSelectPress = now;
    		  evt.type = EVENT_BUTTON_SELECT;
    		  evt.value = 0;
    		  osMessageQueuePut(inputEventQueueHandle, &evt, 0, 0);
    	  }
      }

      if (button_stop_flag) {
    	  button_stop_flag = 0;
    	  if(now - lastStopPress > debounceDelay){
			  lastStopPress = now;
			  evt.type = EVENT_BUTTON_STOP;
			  evt.value = 0;
			  osMessageQueuePut(inputEventQueueHandle, &evt, 0, 0);
    	  }
      }

      if (button_left_flag) {
    	  button_left_flag = 0;
    	  if(now - lastLeftPress > debounceDelay){
			  lastLeftPress = now;
			  evt.type = EVENT_BUTTON_LEFT;
			  evt.value = 0;
			  osMessageQueuePut(inputEventQueueHandle, &evt, 0, 0);
    	  }
      }

      if (button_right_flag) {
    	  button_right_flag = 0;
    	  if(now - lastRightPress > debounceDelay){
			  lastRightPress = now;
			  evt.type = EVENT_BUTTON_RIGHT;
			  evt.value = 0;
			  osMessageQueuePut(inputEventQueueHandle, &evt, 0, 0);
    	  }
      }

      if (encoder_left_delta != 0 && (now - lastEncLeft > encDebounceDelay)) {
          evt.type = EVENT_ENCODER_LEFT;
          evt.value = encoder_left_delta;
          encoder_left_delta = 0;
          lastEncLeft = now;
          osMessageQueuePut(inputEventQueueHandle, &evt, 0, 0);
      }

      if (encoder_right_delta != 0 && (now - lastEncRight > encDebounceDelay)) {
          evt.type = EVENT_ENCODER_RIGHT;
          evt.value = encoder_right_delta;
          encoder_right_delta = 0;
          lastEncRight = now;
          osMessageQueuePut(inputEventQueueHandle, &evt, 0, 0);
      }

      osDelay(1); // Run task every 10ms
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_UI_Control_Task */
/**
* @brief Function implementing the UI_Control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UI_Control_Task */
void Start_UI_Control_Task(void *argument)
{
  /* USER CODE BEGIN Start_UI_Control_Task */
	InputEvent_t evt;
	static int firstDraw = 1;
	Screen_t screen = SCREEN_DEFAULT;
    DefaultScreenData_t screenData = {
	    .bpm = bpm,
	    .volume = volume,
	    .is_playing = metronome_running
    };
	if ((SCREEN_DEFAULT) && (firstDraw == 1)) {
	    OLED_ShowDefaultScreen(bpm, volume, metronome_running);
	    firstDraw = 0;
	}
  for(;;)
  {
	  if (osMessageQueueGet(inputEventQueueHandle, &evt, NULL, osWaitForever) == osOK){
	      switch (evt.type)
	      	  {
				  case EVENT_ENCODER_LEFT:
					  bpm += evt.value;
					  if (bpm < 30) bpm = 30;
					  if (bpm > 300) bpm = 300;
					  sprintf(msg, "BPM: %d\r\n", bpm);
					  break;

				  case EVENT_ENCODER_RIGHT:
					  volume += evt.value;
					  if (volume < 0) volume = 0;
					  if (volume > 100) volume = 100;
					  sprintf(msg, "Volume: %d\r\n", volume);
					  break;

				  case EVENT_BUTTON_LEFT:
					  screen--;
					  if((screen > 0) && (screen != 1) && (screen != 2)){
						  screen = 0;
					  }
					  sprintf(msg, "Screen: %d\r\n", screen);
					  break;

				  case EVENT_BUTTON_RIGHT:
					  screen++;
					  if(screen >= 2){
						  screen = 2;
					  }
					  sprintf(msg, "Screen: %d\r\n", screen);
					  break;

				  case EVENT_BUTTON_SELECT:
					  sprintf(msg, "Starting metronome...\r\n");
					  if (!metronome_running)
					  {
						  //Metronome_Start(bpm, volume);
						  metronome_running = 1;
					  }
					  break;

				  case EVENT_BUTTON_STOP:
					  sprintf(msg, "Stopping metronome...\r\n");
					  if (metronome_running)
					  {
						  //Metronome_Stop();
						  metronome_running = 0;
					  }
					  break;

				  default:
					  break;
	              }
	          }
		  screenData.bpm = bpm;
		  screenData.volume = volume;
		  screenData.is_playing = metronome_running;

		  osMessageQueuePut(screenQueueHandle, &screen, 0, 0);
		  osMessageQueuePut(defaultScreenQueueHandle, &screenData, 0, 0);

	  	  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

	      osDelay(1); // Optional: prevent flooding
	  }
  /* USER CODE END Start_UI_Control_Task */
}

/* USER CODE BEGIN Header_Start_Metronome_Engine_Task */
/**
* @brief Function implementing the Metronome_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Metronome_Engine_Task */
void Start_Metronome_Engine_Task(void *argument)
{
  /* USER CODE BEGIN Start_Metronome_Engine_Task */
  /* Infinite loop */
  for(;;)
  {
	//char msg[] = "Hello from Metronome_Engine_Task!\r\n";
	//CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    osDelay(10);
  }
  /* USER CODE END Start_Metronome_Engine_Task */
}

/* USER CODE BEGIN Header_Start_Audio_Out_Task */
/**
* @brief Function implementing the Audio_Out_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Audio_Out_Task */
void Start_Audio_Out_Task(void *argument)
{
  /* USER CODE BEGIN Start_Audio_Out_Task */
  /* Infinite loop */
  for(;;)
  {
	//char msg[] = "Hello from Audio_Output_Task!\r\n";
	//CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    osDelay(10);
  }
  /* USER CODE END Start_Audio_Out_Task */
}

/* USER CODE BEGIN Header_Screen_Task */
/**
* @brief Function implementing the Screen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Screen_Task */
void Screen_Task(void *argument)
{
  /* USER CODE BEGIN Screen_Task */
  /* Infinite loop */
  Screen_t currentScreen = SCREEN_DEFAULT;  // Start with default screen
  Screen_t newScreen;
  DefaultScreenData_t defaultData;

  for(;;)
  {
	  if (osMessageQueueGet(screenQueueHandle, &newScreen, NULL, osWaitForever) == osOK) {
		  currentScreen = newScreen;

		  // Flush the queue
		  if (currentScreen == SCREEN_DEFAULT) {
			   osMessageQueueReset(defaultScreenQueueHandle);
		  }
	  }
	  switch (currentScreen) {
		  case SCREEN_DEFAULT:
			  if(osMessageQueueGet(defaultScreenQueueHandle, &defaultData, NULL, osWaitForever) == osOK){
				  OLED_ShowDefaultScreen(defaultData.bpm,defaultData.volume,defaultData.is_playing);
			  }
			  break;
		  case SCREEN_SETTINGS:
			  OLED_ShowSettingsScreen();
			  break;
		  case SCREEN_SAMPLES:
			  OLED_ShowSamplesScreen();
			  break;
	  }
    osDelay(1);
  }
  /* USER CODE END Screen_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
