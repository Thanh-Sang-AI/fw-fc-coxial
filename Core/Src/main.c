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
#include "drivers/icm45686/icm45686.h"
#include "drivers/bmp388/bmp388.h"
#include "protocol/mavlink/mavlink.h"
#include "math/filter/filter.h"
#include <math.h>

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

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart7;

/* Definitions for Led_Blink */
osThreadId_t Led_BlinkHandle;
const osThreadAttr_t Led_Blink_attributes = {
  .name = "Led_Blink",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for Barometers_Task */
osThreadId_t Barometers_TaskHandle;
const osThreadAttr_t Barometers_Task_attributes = {
  .name = "Barometers_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* USER CODE BEGIN PV */
icm45686_t imu = {
		.hspi = &hspi1,
		.cs_port = IMU1_CS_GPIO_Port,
		.cs_pin = IMU1_CS_Pin
};

HAL_StatusTypeDef imu_status;
mavlink_t mav;

// Bo loc Gyro, Accel va Kalman uoc luong goc
biquad2p_filter_t filter_gyro[3];
biquad2p_filter_t filter_accel[3];
kalman_angle_t    kalman_roll;
kalman_angle_t    kalman_pitch;

volatile attitude_t att       = {0};
volatile float      imu_dt    = 0.0f;
float               gyro_bias[3] = {0.0f};

BMP388_t            baro;
bool                baro_status = false;

// Pipeline loc Barometer va EKF do cao (Chuan PX4 NED: Z-down noi bo, Z-up xuat ngoai)
outlier_filter_t    filter_baro_outlier;
median_filter_t     filter_baro_median;
biquad2p_filter_t   filter_baro;
altitude_ekf_t      alt_ekf;
volatile bool       baro_has_new_data = false;
volatile float      baro_filtered_alt = 0.0f;
volatile float      ned_z             = 0.0f;  // Vi tri NED Z (m): am khi bay len
volatile float      ned_vz            = 0.0f;  // Van toc NED Vz (m/s): am khi bay len
volatile float      estimated_alt     = 0.0f;  // Do cao thuc te (m): duong khi bay len
volatile float      estimated_vz      = 0.0f;  // Van toc leo (m/s): duong khi bay len

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART7_Init(void);
static void MX_I2C2_Init(void);
void Led_blink(void *argument);
void IMU_task(void *argument);
void Barometers_task(void *argument);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_UART7_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Bat toan bo nguon sensor (Sensor 1..4, Baro)
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  GPIO_InitTypeDef pwr_init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW };
  pwr_init.Pin = GPIO_PIN_4; HAL_GPIO_Init(GPIOF, &pwr_init); HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
  pwr_init.Pin = GPIO_PIN_7; HAL_GPIO_Init(GPIOE, &pwr_init); HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  pwr_init.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOG, &pwr_init); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, GPIO_PIN_SET); // bat nguon imu1
  HAL_Delay(20);

  imu_status = icm45686_init(&imu);

  // Khoi tao thu vien MAVLink tren UART7
  mavlink_init(&mav, &huart7);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Led_Blink */
  Led_BlinkHandle = osThreadNew(Led_blink, NULL, &Led_Blink_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(IMU_task, NULL, &IMU_Task_attributes);

  /* creation of Barometers_Task */
  Barometers_TaskHandle = osThreadNew(Barometers_task, NULL, &Barometers_Task_attributes);

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
	  // Ngat UART tu dong bat byte o che do ngam (background), khong can ham doc o day nua!
//	  if(imu_status == HAL_OK){
//		  icm45686_read(&imu);
//	  }
//
//	  if(HAL_GetTick() - t_led >= 500){
//		  t_led += 500;
//		  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//	  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 460800;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU1_CS_Pin SENSOR1_EN_Pin */
  GPIO_InitStruct.Pin = IMU1_CS_Pin|SENSOR1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU1_DRDY_Pin */
  GPIO_InitStruct.Pin = IMU1_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU1_DRDY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART7)
	{
		mavlink_uart_rx_callback(&mav);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART7)
	{
		mavlink_uart_error_callback(&mav);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Led_blink */
/**
  * @brief  Function implementing the Led_Blink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Led_blink */
void Led_blink(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t last_time = HAL_GetTick();
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

	  uint32_t now = HAL_GetTick();
	  uint32_t dt_ms = now - last_time;
	  if (dt_ms >= 1000) {
		  last_time = now;
		  uint32_t cnt = count_rc_mavlink;
		  count_rc_mavlink = 0;
		  rc_mavlink_hz = (float)cnt * 1000.0f / (float)dt_ms;
	  }

	  vTaskDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_IMU_task */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_task */
void IMU_task(void *argument)
{
  /* USER CODE BEGIN IMU_task */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  for (int i = 0; i < 3; i++) {
    biquad2p_filter_init(&filter_gyro[i], 92.0f, 500.0f);
    biquad2p_filter_init(&filter_accel[i], 25.0f, 500.0f); // LPF 25 Hz triet rung motor
  }
  biquad2p_filter_reset(&filter_accel[2], 1.0f); // Dat san 1g cho truc Z khi de yen tren ban
  kalman_angle_init(&kalman_roll,  0.0015f, 0.003f, 0.03f);
  kalman_angle_init(&kalman_pitch, 0.0015f, 0.003f, 0.03f);
  icm45686_calibrate_gyro(&imu, gyro_bias);

  uint32_t last_cycle = DWT->CYCCNT;

  for(;;)
  {
    uint32_t now_cycle = DWT->CYCCNT;
    imu_dt = (float)(now_cycle - last_cycle) / (float)SystemCoreClock;
    last_cycle = now_cycle;

    if (imu_status == HAL_OK) {
      icm45686_read(&imu);

      // 1. Anh xa he truc tu chip sang than may bay (ROTATION_ROLL_180_YAW_90)
      float ax_raw = -imu.accel_g[1];
      float ay_raw = -imu.accel_g[0];
      float az_raw = -imu.accel_g[2];

      float gx = -imu.gyro_dps[1] - gyro_bias[0];
      float gy =  imu.gyro_dps[0] - gyro_bias[1];
      float gz = -imu.gyro_dps[2] - gyro_bias[2];

      // 2. Loc Gyro (92 Hz) va Accel (25 Hz - triet rung motor)
      att.roll_rate  = biquad2p_filter_apply(&filter_gyro[0], gx);
      att.pitch_rate = biquad2p_filter_apply(&filter_gyro[1], gy);
      att.yaw_rate   = biquad2p_filter_apply(&filter_gyro[2], gz);

      att.ax = biquad2p_filter_apply(&filter_accel[0], ax_raw);
      att.ay = biquad2p_filter_apply(&filter_accel[1], ay_raw);
      att.az = biquad2p_filter_apply(&filter_accel[2], az_raw);

      // 3. Goc tho Accel va Kalman cap nhat goc
      float acc_roll  = atan2f(att.ay, att.az) * 57.29578f;
      float acc_pitch = atan2f(att.ax, sqrtf(att.ay * att.ay + att.az * att.az)) * 57.29578f;

      att.roll  = kalman_angle_update(&kalman_roll,  acc_roll,  att.roll_rate,  imu_dt);
      att.pitch = kalman_angle_update(&kalman_pitch, acc_pitch, att.pitch_rate, imu_dt);

      // 4. EKF du doan do cao lien tuc o 500 Hz (Predict step)
      float r_rad = att.roll  * 0.0174532925f;
      float p_rad = att.pitch * 0.0174532925f;
      float sin_p = sinf(p_rad), cos_p = cosf(p_rad);
      float sin_r = sinf(r_rad), cos_r = cosf(r_rad);

      float az_world_up = (att.ax * sin_p) + (att.ay * sin_r * cos_p) + (att.az * cos_r * cos_p);
      float az_ned_mps2 = (1.0f - az_world_up) * 9.80665f;

      altitude_ekf_predict(&alt_ekf, az_ned_mps2, imu_dt);

      // 5. Chi cap nhat EKF tu Baro khi co mau moi (nhip 50 Hz, moi ~10 chu ky IMU)
      if (baro_has_new_data) {
        baro_has_new_data = false;
        altitude_ekf_update_baro(&alt_ekf, -baro_filtered_alt);
      }

      // 6. Xuat ket qua Fusion o 500 Hz
      ned_z         = alt_ekf.z;
      ned_vz        = alt_ekf.vz;
      estimated_alt = -alt_ekf.z;
      estimated_vz  = -alt_ekf.vz;
    }

    vTaskDelay(2);
  }
  /* USER CODE END IMU_task */
}

/* USER CODE BEGIN Header_Barometers_task */
/**
* @brief Function implementing the Barometers_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Barometers_task */
void Barometers_task(void *argument)
{
  /* USER CODE BEGIN Barometers_task */
  outlier_filter_init(&filter_baro_outlier, 0.2f, 10);
  median_filter_init(&filter_baro_median, 5);
  biquad2p_filter_init(&filter_baro, 10.0f, 50.0f);
  altitude_ekf_init(&alt_ekf, 0.5f, 0.01f, 0.20f);

  baro_status = BMP388_Init(&baro, &hi2c2);
  if (baro_status) {
    BMP388_CalibrateGround(&baro, 50);
    outlier_filter_reset(&filter_baro_outlier, baro.altitude_m);
    median_filter_reset(&filter_baro_median, baro.altitude_m);
    biquad2p_filter_reset(&filter_baro, baro.altitude_m);
    altitude_ekf_reset(&alt_ekf, -baro.altitude_m); // PX4 NED: Z = -altitude
  }

  /* Infinite loop */
  for(;;)
  {
    if (baro_status) {
      BMP388_ReadData(&baro);

      // 1. Loc loai bo nhiễu gai (Outlier Rejection > 0.2m / 20ms)
      float sanitized_alt = outlier_filter_apply(&filter_baro_outlier, baro.altitude_m);

      // 2. Loc trung vi 5 mau (Median Filter)
      float median_alt = median_filter_apply(&filter_baro_median, sanitized_alt);

      // 3. Loc thong thap bac 2 tai 10 Hz
      baro_filtered_alt = biquad2p_filter_apply(&filter_baro, median_alt);

      // 4. Bat co bao cho IMU_task cap nhat EKF
      baro_has_new_data = true;
    }
    vTaskDelay(20); // 50 Hz ODR cua BMP388
  }
  /* USER CODE END Barometers_task */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
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
