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
#include "Drivers/BMI088.h"
#include "Drivers/BMP390.h"
#include "Drivers/flash.h"
#include "util/avghistory.h"
#include "kalman.h"
#include "config.h"
#include "log.h"
#include <stdarg.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRINT_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;


/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for readSensors */
osThreadId_t readSensorsHandle;
const osThreadAttr_t readSensors_attributes = {
  .name = "readSensors",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for deploymentTask */
osThreadId_t deploymentTaskHandle;
const osThreadAttr_t deploymentTask_attributes = {
  .name = "deploymentTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for sensorData */
osMutexId_t sensorDataHandle;
const osMutexAttr_t sensorData_attributes = {
  .name = "sensorData"
};
/* USER CODE BEGIN PV */
BMI088 imu;
float accelData[3];
float gyroData[3];

BMP390 baro;
float pressure;
float baro_altitude;
int16_t temp;

static KalmanFilter kf;

PyroChannel pyros[CHANNEL_COUNT] = {
		{RAPTOR_GPIO_Port, RAPTOR_Pin, 0, false},
		{PIRANHA_GPIO_Port, PIRANHA_Pin, 0, false}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN2_Init(void);
void StartPrintTask(void *argument);
void StartReadSensors(void *argument);
void StartDeployment(void *argument);
void StartLog(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void debug_print(const char *format, ...) {
	char buf[PRINT_BUFFER_SIZE];
	va_list args;
	va_start(args, format);
	int n = vsnprintf(buf, sizeof(buf), format, args);
	CDC_Transmit_FS(buf, n);
	va_end(args);
}

void channel_fire(uint8_t index) {
	if (index >= CHANNEL_COUNT) return;
	pyros[index].firing = true;
	pyros[index].fire_time = HAL_GetTick();
	HAL_GPIO_WritePin(pyros[index].port, pyros[index].pin, GPIO_PIN_SET);
}

void channel_update(void) {
	uint32_t now = HAL_GetTick();
	for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
		if (pyros[i].firing && (now - pyros[i].fire_time >= CHANNEL_FIRE_TIME)) {
			pyros[i].firing = false;
			HAL_GPIO_WritePin(pyros[i].port, pyros[i].pin, GPIO_PIN_RESET);

		}
	}
}
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
  MX_SPI1_Init();
  MX_FDCAN2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(ACCEL_nCS_GPIO_Port, ACCEL_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GYRO_nCS_GPIO_Port, GYRO_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BARO_nCS_GPIO_Port, BARO_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  bmi088_init(&imu, &hspi1, ACCEL_nCS_GPIO_Port, GYRO_nCS_GPIO_Port, ACCEL_nCS_Pin, GYRO_nCS_Pin);
  bmp_init(&baro, &hspi1, BARO_nCS_GPIO_Port, BARO_nCS_Pin);
  KalmanFilter_init(&kf, KALMAN_PERIOD, ALTITUDE_SIGMA, ACCELERATION_SIGMA, MODEL_SIGMA);
  Flash_Setup(&hspi1, FLASH_nCS_GPIO_Port, FLASH_nCS_Pin);

  HAL_Delay(2000);
  debug_print("Starting print\r\n");
  log_print_all();
  debug_print("End print\r\n");
  log_setup();
  log_start();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of sensorData */
  sensorDataHandle = osMutexNew(&sensorData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of printTask */
  printTaskHandle = osThreadNew(StartPrintTask, NULL, &printTask_attributes);

  /* creation of readSensors */
  readSensorsHandle = osThreadNew(StartReadSensors, NULL, &readSensors_attributes);

  /* creation of deploymentTask */
  deploymentTaskHandle = osThreadNew(StartDeployment, NULL, &deploymentTask_attributes);

  /* creation of logTask */
  logTaskHandle = osThreadNew(StartLog, NULL, &logTask_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 1;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ACCEL_nCS_Pin|GYRO_nCS_Pin|BARO_nCS_Pin|FLASH_nCS_Pin
                          |PIRANHA_Pin|RAPTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_LED_Pin|Backlight_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACCEL_nCS_Pin GYRO_nCS_Pin BARO_nCS_Pin FLASH_nCS_Pin
                           PIRANHA_Pin RAPTOR_Pin */
  GPIO_InitStruct.Pin = ACCEL_nCS_Pin|GYRO_nCS_Pin|BARO_nCS_Pin|FLASH_nCS_Pin
                          |PIRANHA_Pin|RAPTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_LED_Pin Backlight_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin|Backlight_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPrintTask */
/**
  * @brief  Function implementing the printTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
    float localAccel[3];
    float localGyro[3];
    float localPressure;
    int16_t localTemperature;
    float localBaroAltitude;
    uint32_t last_run_time = HAL_GetTick();
    /* Infinite loop */
    for(;;) {
        if (osMutexAcquire(sensorDataHandle, osWaitForever) == osOK) {
            memcpy(localAccel, accelData, sizeof(accelData));
            memcpy(localGyro, gyroData, sizeof(gyroData));
            localPressure = pressure;
            localBaroAltitude = baro_altitude;
            localTemperature = temp;
            osMutexRelease(sensorDataHandle);

            uint32_t current_tick = HAL_GetTick();
            debug_print("--- Tick: %lu ---\r\n", current_tick);
            debug_print("BMI088 Accel: X: %.2f Y: %.2f Z: %.2f g\r\n", localAccel[0], localAccel[1], localAccel[2]);
            debug_print("BMI088 Gyro:  X: %.2f Y: %.2f Z: %.2f dps\r\n", localGyro[0], localGyro[1], localGyro[2]);
            debug_print("BMP390 Alt:   %.2f m\r\n", localBaroAltitude);
            debug_print("BMP390 Pres:  %.0f Pa\r\n", localPressure);
            debug_print("BMP390 Temp:  %.2f C\r\n", (float)localTemperature / 100.0f);
            debug_print("\r\n");

            HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);

        } else {
            debug_print("PrintTask: Failed to acquire mutex!\r\n");
        }

        osDelayUntil(last_run_time + 500);
        last_run_time = HAL_GetTick();
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReadSensors */
/**
* @brief Function implementing the readSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadSensors */
void StartReadSensors(void *argument)
{
  /* USER CODE BEGIN StartReadSensors */
  /* Infinite loop */
  for(;;)
  {
	 accel_step(&imu);
	 gyro_step(&imu);
	 baro_step(&baro);

	 float *accel = accel_get(&imu);
	 float *gyro = gyro_get(&imu);
	 float local_pressure = baro_get_pressure(&baro);
	 float local_altitude = baro_get_altitude(&baro);
	 int16_t local_temp = baro_get_temp(&baro);

	 osMutexAcquire(sensorDataHandle, osWaitForever);

	 accelData[0] = accel[0];
	 accelData[1] = accel[1];
	 accelData[2] = accel[2];

	 gyroData[0] = gyro[0];
	 gyroData[1] = gyro[1];
	 gyroData[2] = gyro[2];

	 pressure = local_pressure;
	 baro_altitude = local_altitude;
	 temp = local_temp;

	 osMutexRelease(sensorDataHandle);

	 osDelay(100);
  }
  /* USER CODE END StartReadSensors */
}

/* USER CODE BEGIN Header_StartDeployment */
/**
* @brief Function implementing the deploymentTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDeployment */
void StartDeployment(void *argument)
{
  /* USER CODE BEGIN StartDeployment */
	FlightPhase phase = Startup;

	// Running average for acceleration due to gravity
	static AvgHistory gravity_est_state;
	AvgHistory_Init(&gravity_est_state);

	// Running average for ground level at startup
	static AvgHistory ground_level_est_state;
	AvgHistory_Init(&ground_level_est_state);

	float apogee = 0;
	uint32_t land_time = 0;
	bool launched_state = false;

  /* Infinite loop */
  for(;;)
  {
	  uint32_t now = HAL_GetTick();
	  float currentAccel[3];
	  float currentGyro[3];

	  osMutexAcquire(sensorDataHandle, osWaitForever);
	  currentAccel[0] = accelData[0];
	  currentAccel[1] = accelData[1];
	  currentAccel[2] = accelData[2];

	  currentGyro[0] = gyroData[0];
	  currentGyro[1] = gyroData[1];
	  currentGyro[2] = gyroData[2];

	  float current_pressure = pressure;
	  float raw_altitude = baro_altitude;
	  int16_t current_temp = temp;
	  osMutexRelease(sensorDataHandle);

	  float accel_mag = sqrtf(currentAccel[0] * currentAccel[0] + currentAccel[1] * currentAccel[1] + currentAccel[2] * currentAccel[2]);

	  if (phase < Launched) {
		  AvgHistory_Add(&gravity_est_state, accel_mag);
		  AvgHistory_Add(&ground_level_est_state, raw_altitude);
	  }

	  if (phase == Startup) {
		  uint8_t ground_level_full = AvgHistory_Full(&ground_level_est_state);
		  uint8_t gravity_full = AvgHistory_Full(&gravity_est_state);
		  if (!ground_level_full || !gravity_full) {
			  // return?
			  continue;
		  }
		  phase = Idle;
	  }

	  accel_mag -= AvgHistory_OldAvg(&gravity_est_state);
	  float curr_alt = raw_altitude - AvgHistory_OldAvg(&ground_level_est_state);

	  channel_update();

	  bool any_channel_firing = false;
	  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
		  if (pyros[i].firing) {
			  any_channel_firing = true;
			  break;
		  }
	  }

	  if (!any_channel_firing) {
		  KalmanFilter_step(&kf, accel_mag, curr_alt);
	  }

	  if (phase == Idle) {
		  if (kf.est[1] > LAUNCH_VELOCITY && kf.est[2] > LAUNCH_ACCEL) {
			  phase = Launched;
			  log_start();
			  launched_state = true;
		  }
	  } else if (phase == Launched) {
		  if (kf.est[1] < 0) {
			  apogee = kf.est[0];
			  channel_fire(SEPARATION_INDEX);
			  phase = DescendingAfterSeparation;
		  }
	  } else if (phase == DescendingAfterSeparation) {

		  if (kf.est[0] < REEFING_ALTITUDE && (now - pyros[SEPARATION_INDEX].fire_time) > 3000) {
			  channel_fire(REEFING_INDEX);
			  phase = DescendingAfterReefing;
		  }
	  } else if (phase == DescendingAfterReefing) {
		  if (kf.est[0] < LANDED_ALTITUDE && abs(kf.est[1]) < LANDED_VELOCITY && abs(kf.est[2]) < LANDED_ACCEL) {
			  if (land_time == 0) {
				  land_time = now;
				  if (land_time == 0) {
					  land_time = 1;
				  }
			  } else if ((now - land_time) >= LANDED_TIME) {
				  phase = Landed;
				  log_stop();
			  }
		  } else {
			  land_time = 0;
		  }
	  }
	  LogMessage data;
	  data.time_ms = now;
	  data.phase = phase;
	  data.kf_pos = kf.est[0];
	  data.kf_vel = kf.est[1];
	  data.kf_accel = kf.est[2];
	  data.altitude = raw_altitude;
	  data.accel_x = currentAccel[0];
	  data.accel_y = currentAccel[1];
	  data.accel_z = currentAccel[2];
	  data.gyro_x = currentGyro[0];
	  data.gyro_y = currentGyro[0];
	  data.gyro_z = currentGyro[0];
	  data.pressure = current_pressure;
	  data.temp = current_temp;
	  data.apogee = apogee;
	  data.launched = launched_state;
	  data.landed_time = land_time;

	  log_add(&data);
	  debug_print("Phase: %d\r\n",phase);
	  debug_print("kalman pos: %.2f\r\n",kf.est[0]);
	  debug_print("kalman vel: %.2f\r\n",kf.est[1]);
	  debug_print("kalman accel: %.2f\r\n",kf.est[2]);

    osDelay(100);

  }
  /* USER CODE END StartDeployment */
}

/* USER CODE BEGIN Header_StartLog */
/**
* @brief Function implementing the logTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLog */
void StartLog(void *argument)
{
  /* USER CODE BEGIN StartLog */
  /* Infinite loop */
  for(;;)
  {
    log_step();
    osDelay(500);
  }
  /* USER CODE END StartLog */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
