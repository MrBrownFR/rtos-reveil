/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct temps {
	unsigned char  heure;
	unsigned char  min;
	unsigned char  sec;
};

typedef struct {
	int32_t integer :24;
	uint32_t dec :8;
} Q8;

typedef enum {
	NORMAL, ALARM,
} ScreenMode;

typedef enum {
	OK, AUTO, MAN,
} DriveMode;

typedef struct {
	DriveMode mode;
	Q8 val;
} SensorState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_MASTER  0x00

#define HOUR_CHANGED_EVT (1 << 0)
#define ALARM_CHANGED_EVT (1 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

struct temps horloge = 	{ 0, 0, 0 };

struct {
	struct temps val;
	bool set;
} alarm = { 0 };

static uint8_t rxUART_byte;
QueueHandle_t rxUARTQueue;

QueueHandle_t rxCANQueue;

EventGroupHandle_t update_time_evt_grp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void edit_time( void *pvParameters );
void vHandleMsgTask( void* pvParameters);
void vCanSendTask(void *pvParameters);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void receive(char a);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_LPUART1_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&hlpuart1);
  HAL_UART_Receive_IT(&hlpuart1, &rxUART_byte, 1);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

   /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  update_time_evt_grp = xEventGroupCreate();
  if (update_time_evt_grp == NULL) {
  		char msg[] = "Error allocating update_time_evt_grp\r\n";
  		HAL_UART_Transmit(&huart2, msg, sizeof(msg), HAL_MAX_DELAY);
  		return 1;
  	}
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  rxUARTQueue = xQueueCreate(32, sizeof(uint8_t));
  if (rxUARTQueue == NULL) {
    		char msg[] = "Error allocating rxUARTQueue\r\n";
    		HAL_UART_Transmit(&huart2, msg, sizeof(msg), HAL_MAX_DELAY);
    		return 1;
    }

  rxCANQueue = xQueueCreate(5, sizeof(char) * 8);
  if (rxCANQueue == NULL) {
  		char msg[] = "Error allocating rxCANQueue\r\n";
  		HAL_UART_Transmit(&huart2, msg, sizeof(msg), HAL_MAX_DELAY);
  		return 1;
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(		edit_time,       /* Function that implements the task. */
                    "TIME EDITION",          /* Text name for the task. */
                    1024,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+2,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

	xTaskCreate(	vHandleMsgTask, /* Function that implements the task. */
					"Handle message from CAN", /* Text name for the task. */
					1024, /* Stack size in words, not bytes. */
					NULL, /* Parameter passed into the task. */
					tskIDLE_PRIORITY + 3, /* Priority at which the task is created. */
					NULL); /* Used to pass out the created task's handle. */

	xTaskCreate(vCanSendTask,	/* Function that implements the task. */
			  "Send Msg throught Can",          /* Text name for the task. */
	          1024,      /* Stack size in words, not bytes. */
	          NULL,    /* Parameter passed into the task. */
			  tskIDLE_PRIORITY+1,/* Priority at which the task is created. */
	          NULL );
  /* USER CODE END RTOS_THREADS */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED5_Pin|DIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin DIR_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin DIS_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*----------------------------------------------------------------------------
 *        Functions
 *---------------------------------------------------------------------------*/

// Reads characters until '\r' or '\n'. Returns false on timeout.
static bool read_line(char *buf, size_t size, TickType_t timeout){
  size_t n = 0;
  uint8_t c;

  while (xQueueReceive(rxUARTQueue, &c, timeout) == pdTRUE) {
    if (c == '\r' || c == '\n') {
      if (n == 0)
    	  continue;      // ignore the leftover CR/LF that followed the 'H'
      buf[n] = '\0';
      return true;
    }
    if (n < size - 1)
    	buf[n++] = (char)c;
  }
  return false;                  // timeout
}

// Accepts "HH:MM:SS" avec 24>HH>=0 , 60>MM>=0 et 60>SS>=0
static bool parse_time(const char *s, int *h, int *m, int *sec){
  int n = sscanf(s, " %d:%d:%d", h, m, sec);
  if (n != 3)
	  return false;
  return (*h >= 0 && *h < 24) && (*m >= 0 && *m < 60) && (*sec >= 0 && *sec < 60);
}

/*----------------------------------------------------------------------------
 *        Tasks
 *---------------------------------------------------------------------------*/


void edit_time(void *pvParameters)
{
  uint8_t c;
  char line[16];
  int h, m, s;

  while (1) {
    xQueueReceive(rxUARTQueue, &c, portMAX_DELAY);

    if (c == 'H' || c == 'h') {
      printf("Enter time (HH:MM:SS): \r\n");

      if (!read_line(line, sizeof line, pdMS_TO_TICKS(20000))) {
        printf("Timeout\r\n");
      } else if (parse_time(line, &h, &m, &s)) {
        horloge.heure = h;
    	horloge.min = m;
    	horloge.sec = s;
        printf("Time set to %02d:%02d:%02d\r\n", h, m, s);
        xEventGroupSetBits(update_time_evt_grp, HOUR_CHANGED_EVT);
      } else {
        printf("Invalid format\r\n");
      }

    }

    if (c == 'A' || c == 'a' ) {
          printf("Setup alarm (HH:MM:SS): \r\n");

          if (!read_line(line, sizeof line, pdMS_TO_TICKS(20000))) {
            printf("Timeout\r\n");
          } else if (parse_time(line, &h, &m, &s)) {
            alarm.val.heure = h;
        	alarm.val.min = m;
        	alarm.val.sec = s;
        	alarm.set = true;
            printf("Alarm set to %02d:%02d:%02d\r\n", h, m, s);
            xEventGroupSetBits(update_time_evt_grp, ALARM_CHANGED_EVT);
          } else {
            printf("Invalid format\r\n");
          }

        }

    if (c == 'O' || c == 'o'){
    	alarm.set = false;
    	xEventGroupSetBits(update_time_evt_grp, ALARM_CHANGED_EVT);
    	printf("Alarm OFF \r\n");
    }
  }
}

void vCanSendTask(void *pvParameters){
	HAL_CAN_Start(&hcan1);

	static CAN_TxHeaderTypeDef hdr = {
	        .StdId = CAN_ID_MASTER,
	        .IDE   = CAN_ID_STD,
	        .RTR   = CAN_RTR_DATA,
	        .DLC   = 4,
	        .TransmitGlobalTime = DISABLE
	 };

	EventBits_t uxBits;

	uint32_t mailbox;
	uint8_t  msg[8] = {0};

	while (1){


		uxBits = xEventGroupWaitBits(
			update_time_evt_grp,   /* The event group being tested. */
			HOUR_CHANGED_EVT | ALARM_CHANGED_EVT, /* The bits within the event group to wait for. */
			pdTRUE,
			pdFALSE,       /* Don't wait for both bits, either bit will do. */
			pdMS_TO_TICKS(1000) );/* Wait a maximum of x ms for either bit to be set. */

			if( ( uxBits & HOUR_CHANGED_EVT ) != 0 )
			{
				msg[0] = 'T';
				msg[1] = horloge.heure;
				msg[2] = horloge.min;
				msg[3] = horloge.sec;
				if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
				        HAL_CAN_AddTxMessage(&hcan1, &hdr, msg, &mailbox);
			}
			else if( ( uxBits & ALARM_CHANGED_EVT ) != 0 )
			{
				msg[0] = 'A';
				if (alarm.set){
					msg[1] = alarm.val.heure;
					msg[2] = alarm.val.min;
					msg[3] = alarm.val.sec;
				} else {
					msg[1] = 'O';
					msg[2] = 'F';
					msg[3] = 'F';
				}
				if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
					HAL_CAN_AddTxMessage(&hcan1, &hdr, msg, &mailbox);
			}
	}
}

void vHandleMsgTask(void *parameters) {
	char message[8];
	while (1) {
		xQueueReceive(rxCANQueue, message, portMAX_DELAY);

		/* Environmental data */
		if (message[0] == 'E') {
			switch (message[1]) {
				case 'A':
					if (message[6] == 1){
						/* Ventiler (Aérer) */
					}
					break;
				case 'T':
					if (message[6] == 1){
						/* Balayer (Rafraichir) */
					}
					break;
				case 'S':
					break;
				default:
					break;
			}
		}
	}
}

/*----------------------------------------------------------------------------
 *        Callbacks
 *---------------------------------------------------------------------------*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef hdr;
	uint8_t data[9];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data);
	data[8] = hdr.DLC & 0x1F;
	BaseType_t task_woken;
	xQueueSendFromISR(rxCANQueue, data, &task_woken);
	portYIELD_FROM_ISR(task_woken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == hlpuart1.Instance) {

    BaseType_t woken = pdFALSE;
    //If sending to the queue unblocks a task whose priority is higher than the interrupted task, FreeRTOS sets woken to pdTRUE. Otherwise it leaves it at pdFALSE

    xQueueSendFromISR(rxUARTQueue, &rxUART_byte, &woken);
    HAL_UART_Receive_IT(huart, &rxUART_byte, 1);   // re-arm immediately

    portYIELD_FROM_ISR(woken);
    //If woken == pdTRUE, it requests a context switch right when the ISR exits, so the higher-priority task runs immediately.
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == hlpuart1.Instance) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(huart, &rxUART_byte, 1);   // restart after any error
  }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
