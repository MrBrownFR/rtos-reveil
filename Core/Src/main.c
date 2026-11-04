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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "fonts.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AQ_ADRR 0x59
#define TEMP_ADRR 0x80
#define SENSOR_PERIOD_MS   1000
#define SENSOR_CONV_MS     150
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim7;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

typedef struct {
    int8_t    temp_MSB;
    uint8_t    temp_LSB;
    uint16_t voc_index;      // SRAW_VOC en ticks
    bool     voc_valid;
} SensorData_t;

QueueHandle_t xSensorQueue;  // xQueueCreate(1, sizeof(SensorData_t))
SemaphoreHandle_t xI2CMutex;

static const uint8_t aq_cmd[8]      = {0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93};
static const uint8_t aq_selftest[2] = {0x28, 0x0E};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

void vSensorTask(void *pvParameters);
void vOledTask(void *pvParameters);

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
  MX_SPI2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  xI2CMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  BaseType_t r1 = xTaskCreate(      vSensorTask,       /* Function that implements the task. */
                      "Sensors' reading",          /* Text name for the task. */
                      512,      /* Stack size in words, not bytes. */
                      NULL,    /* Parameter passed into the task. */
                      tskIDLE_PRIORITY+1,/* Priority at which the task is created. */
                      NULL );      /* Used to pass out the created task's handle. */

  BaseType_t r2 =  xTaskCreate(      vOledTask,       /* Function that implements the task. */
                      "Ecran",          /* Text name for the task. */
                      512,      /* Stack size in words, not bytes. */
                      NULL,    /* Parameter passed into the task. */
					  tskIDLE_PRIORITY+1,/* Priority at which the task is created. */
                      NULL );      /* Used to pass out the created task's handle. */


  SSD1306_Init();
  char test_buff[4] = "Test";
  SSD1306_GotoXY (0,30);
  SSD1306_Puts (test_buff, &Font_11x18, 0);

  volatile size_t heap_libre = xPortGetFreeHeapSize();   // à observer en Live Expressions
  (void)r1; (void)r2;

  SSD1306_UpdateScreen();
//  vTaskStartScheduler();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, 1); //CSN actif état bas -> On désactive
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET); //CE actif état haut -> On désactive
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1280-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 62500-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void sensor_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { addr, val };
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi2, tx, 2, 10);
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

uint8_t sensor_read_reg(uint8_t addr)
{
    uint8_t rx = 0;
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi2, &addr, 1, 10);
    HAL_SPI_Receive(&hspi2, &rx, 1, 10);
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
    return rx;
}

static void format_temp(char *out, size_t n, float t)
{
    // Évite %f (non activé par défaut avec newlib-nano)
    int t10 = (int)(t * 10.0f + (t >= 0 ? 0.5f : -0.5f));
    int a = t10 < 0 ? -t10 : t10;
    snprintf(out, n, "Temp: %s%d.%d C", t10 < 0 ? "-" : "", a / 10, a % 10);
}

// CRC8 Sensirion (poly 0x31, init 0xFF)
static uint8_t crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

static bool aq_self_test(void)
{
    uint8_t rx[3];
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1, AQ_ADRR << 1, (uint8_t *)aq_selftest, 2, 10);
    xSemaphoreGive(xI2CMutex);

    vTaskDelay(pdMS_TO_TICKS(330));   // 320 ms + marge

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c1, AQ_ADRR << 1, rx, 3, 10);
    xSemaphoreGive(xI2CMutex);

    return st == HAL_OK && crc8(rx, 2) == rx[2] && rx[0] == 0xD4;
}
/*----------------------------------------------------------------------------
 *        Task
 *---------------------------------------------------------------------------*/
void vSensorTask(void *pvParameters)
{
    bool aq_ok = aq_self_test();      // une seule fois au démarrage
    TickType_t xLastWake = xTaskGetTickCount();

    while(1)    {

        sensor_write_reg(TEMP_ADRR, 0x15);                       // température (SPI)

        if (aq_ok) {
            xSemaphoreTake(xI2CMutex, portMAX_DELAY);
            HAL_I2C_Master_Transmit(&hi2c1, AQ_ADRR << 1, (uint8_t *)aq_cmd, 8, 10); //Air Quality (I2C)
            xSemaphoreGive(xI2CMutex);
        }

        vTaskDelay(160 / portTICK_RATE_MS);                     // couvre 150 ms (temp) et 30 ms (SGP40)

        SensorData_t d = {0};

        //Reading Temp sensor (SPI)
        uint8_t read_adrr = 0x02;
        uint8_t rxSpi[3];

        HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
        HAL_SPI_Transmit(&hspi2, &read_adrr , 1, 10);
        HAL_SPI_Receive(&hspi2, rxSpi, 3, 10);
        HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

        d.temp_LSB= (rxSpi[1] >> 6)*25 ;
        d.temp_MSB = rxSpi[0] ;

        if (aq_ok) {
            uint8_t rx[3];
            xSemaphoreTake(xI2CMutex, portMAX_DELAY);
            HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c1, AQ_ADRR << 1, rx, 3, 10);
            xSemaphoreGive(xI2CMutex);

            if (st == HAL_OK && crc8(rx, 2) == rx[2]) {
                d.voc_index   = (rx[0] << 8) | rx[1];
                d.voc_valid = true;
            }
        }

        xQueueSend(xSensorQueue, &d, portMAX_DELAY);
        vTaskDelayUntil(&xLastWake, 1000 / portTICK_RATE_MS);
    }
}

void vOledTask(void *pvParameters)
{
    char line[24];
    SensorData_t d;

    while (!SSD1306_Init()){
    	vTaskDelay(pdMS_TO_TICKS(500));
    }


    while(1)
    {
        bool got = (xQueueReceive(xSensorQueue, &d, pdMS_TO_TICKS(1500)) == pdTRUE);

        if (got) {
            //format_temp(line, sizeof(line), d.temperature);
            sprintf(line, "Temp: %d.%u C  ", d.temp_MSB, d.temp_LSB);
            SSD1306_GotoXY(0, 0);
            SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

            if (!d.voc_valid)          snprintf(line, sizeof(line), "VOC: erreur");
            else if (d.voc_index == 0) snprintf(line, sizeof(line), "VOC: init...");
            else                       snprintf(line, sizeof(line), "VOC: %u", d.voc_index);
            SSD1306_GotoXY(0, 16);
            SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
            SSD1306_GotoXY(0, 0);  SSD1306_Puts("Temp: --", &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_GotoXY(0, 16); SSD1306_Puts("VOC: --",  &Font_7x10, SSD1306_COLOR_WHITE);
        }

        SSD1306_UpdateScreen();
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
