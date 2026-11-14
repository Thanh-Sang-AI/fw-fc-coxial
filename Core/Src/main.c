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
#include "drivers/MTF-01P/mtf01p.h"
#include "protocol/mavlink/mavlink.h"
#include "math/filter/filter.h"
#include "task.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Struct 12 trang thai dong hoc drone [x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
typedef struct {
  float x, y, z;                         // Vi tri (m), Z duong huong len
  float vx, vy, vz;                      // Van toc dai (m/s), Vz duong huong len
  float roll, pitch, yaw;                // Goc Euler (deg)
  float roll_rate, pitch_rate, yaw_rate; // Toc do goc (deg/s)
} drone_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD                 0.0174532925f // He so doi tu deg sang rad (pi / 180)
#define IMU_ACCEL_DEADBAND      0.04f         // 2-sigma theo datasheet ICM-45686 (70 ug/sqrt(Hz) qua LPF 25Hz ~ 4mg)
#define TOF_ALT_NOISE           0.012f        // Sai so chuan ToF (1.2 cm) -> tin ToF rat cao so voi IMU

// Tham so EKF mat phang ngang X/Y:
#define FLOW_ACC_NOISE          0.35f         // Nhieu gia toc IMU (m/s^2)
#define FLOW_ACC_B_NOISE        0.003f        // Nhieu bias gia toc (m/s^3)
#define FLOW_VEL_NOISE          0.10f         // Sai so do luong van toc quang hoc co dinh (0.10 m/s)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;
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
/* Definitions for Op_Flo_Task */
osThreadId_t Op_Flo_TaskHandle;
const osThreadAttr_t Op_Flo_Task_attributes = {
  .name = "Op_Flo_Task",
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
float             gyro_bias[3] = {0.0f};

BMP388_t            baro;
bool                baro_status = false;

// Pipeline loc Barometer va EKF do cao doc lap (Chuan PX4 NED: Z-down noi bo, Z-up xuat ra baro_state)
median_filter_t     filter_baro_median;
biquad2p_filter_t   filter_baro;
altitude_ekf_t      baro_ekf;
volatile float      baro_filtered_alt = 0.0f;

// Struct trang thai do cao doc lap cua Barometer
typedef struct {
  float z;   // Do cao uoc luong tu Barometer EKF (m, Z-up)
  float vz;  // Van toc thang dung tu Barometer EKF (m/s, Vz-up)
} baro_state_t;
volatile baro_state_t baro_state = {0};

// Gia toc tuyen tinh chia se tu IMU_task (500 Hz) cho Op_Flo_task va Barometers_task
volatile float      imu_ax_linear    = 0.0f;
volatile float      imu_ay_linear    = 0.0f;
volatile float      imu_az_ned_mps2  = 0.0f;

mtf01p_t            opflow;

// Bo loc Median (chong gai ToF), Biquad LPF ToF va EKF do cao ToF
median_filter_t     filter_tof_median;
biquad2p_filter_t   filter_opflow_z;
altitude_ekf_t      alt_ekf;

// Bo loc Median 3 mau va Biquad LPF bac 2 cho Optical Flow (triet tieu gai va rang cua)
median_filter_t     filter_flow_med_x;
median_filter_t     filter_flow_med_y;
biquad2p_filter_t   filter_flow_lpf_x;
biquad2p_filter_t   filter_flow_lpf_y;

horizontal_ekf_t    ekf_x;
horizontal_ekf_t    ekf_y;
volatile float      flow_meas_vx      = 0.0f;
volatile float      flow_meas_vy      = 0.0f;
volatile float      flow_filt_x       = 0.0f;  // Optical Flow X sau Median + LPF bac 2 (rad/s)
volatile float      flow_filt_y       = 0.0f;  // Optical Flow Y sau Median + LPF bac 2 (rad/s)
volatile float      flow_diff_x       = 0.0f;  // (flow_filt_y - gyro_pitch_rad) khong nhan h (rad/s)
volatile float      flow_diff_y       = 0.0f;  // (-flow_filt_x + gyro_roll_rad) khong nhan h (rad/s)
volatile float      gyro_pitch_rad    = 0.0f;  // Gyro Pitch (rad/s)
volatile float      gyro_roll_rad     = 0.0f;  // Gyro Roll (rad/s)

volatile float      tof_filtered_alt = 0.0f;

volatile drone_state_t drone_state   = {0}; // 12 trang thai chung cho toan bo drone

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART7_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART5_Init(void);
void Led_blink(void *argument);
void IMU_task(void *argument);
void Barometers_task(void *argument);
void Op_Flo_task(void *argument);

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
  MX_UART5_Init();
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
  mtf01p_init(&opflow, &huart5);

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

  /* creation of Op_Flo_Task */
  Op_Flo_TaskHandle = osThreadNew(Op_Flo_task, NULL, &Op_Flo_Task_attributes);

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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
	else if (huart->Instance == UART5)
	{
		mtf01p_uart_rx_callback(&opflow);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART7)
	{
		mavlink_uart_error_callback(&mav);
	}
	else if (huart->Instance == UART5)
	{
		mtf01p_uart_error_callback(&opflow);
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
    float dt = (float)(now_cycle - last_cycle) / (float)SystemCoreClock;
    last_cycle = now_cycle;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;

    if (imu_status == HAL_OK) {
      icm45686_read(&imu);

      // 1. Anh xa truc (ROTATION_ROLL_180_YAW_90) + Loc Gyro (92 Hz) & Accel (25 Hz)
      drone_state.roll_rate  = biquad2p_filter_apply(&filter_gyro[0], -imu.gyro_dps[1] - gyro_bias[0]);
      drone_state.pitch_rate = biquad2p_filter_apply(&filter_gyro[1],  imu.gyro_dps[0] - gyro_bias[1]);
      drone_state.yaw_rate   = biquad2p_filter_apply(&filter_gyro[2], -imu.gyro_dps[2] - gyro_bias[2]);

      float ax = biquad2p_filter_apply(&filter_accel[0], -imu.accel_g[1]);
      float ay = biquad2p_filter_apply(&filter_accel[1], -imu.accel_g[0]);
      float az = biquad2p_filter_apply(&filter_accel[2], -imu.accel_g[2]);

      // 2. Kalman Roll/Pitch & tich phan Yaw
      drone_state.roll  = kalman_angle_update(&kalman_roll,
                                              atan2f(ay, az) * 57.29578f,
                                              drone_state.roll_rate, dt);
      drone_state.pitch = kalman_angle_update(&kalman_pitch,
                                              atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f,
                                              drone_state.pitch_rate, dt);

      if (fabsf(drone_state.yaw_rate) > 0.05f) {
        drone_state.yaw += drone_state.yaw_rate * dt;
        if (drone_state.yaw >  180.0f) drone_state.yaw -= 360.0f;
        if (drone_state.yaw < -180.0f) drone_state.yaw += 360.0f;
      }

      // 3. Tinh gia toc tuyen tinh chia se cho Op_Flo_task (100 Hz) va Barometers_task (50 Hz)
      float sin_p = sinf(drone_state.pitch * DEG2RAD);
      float cos_p = cosf(drone_state.pitch * DEG2RAD);
      float sin_r = sinf(drone_state.roll  * DEG2RAD);
      float cos_r = cosf(drone_state.roll  * DEG2RAD);

      imu_ax_linear   = (ax + sin_p) * 9.80665f;
      imu_ay_linear   = (ay - sin_r * cos_p) * 9.80665f;
      imu_az_ned_mps2 = (1.0f - ((ax * sin_p) + (ay * sin_r * cos_p) + (az * cos_r * cos_p))) * 9.80665f;
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
  median_filter_init(&filter_baro_median, 3);
  biquad2p_filter_init(&filter_baro, 10.0f, 50.0f);
  altitude_ekf_init(&baro_ekf, 2.0f, 0.01f, 0.25f);

  baro_status = BMP388_Init(&baro, &hi2c2);
  if (baro_status) {
    BMP388_CalibrateGround(&baro, 50);
    median_filter_reset(&filter_baro_median, baro.altitude_m);
    biquad2p_filter_reset(&filter_baro, baro.altitude_m);
    altitude_ekf_reset(&baro_ekf, -baro.altitude_m); // PX4 NED: Z = -altitude
  }

  /* Infinite loop */
  for(;;)
  {
    if (baro_status) {
      BMP388_ReadData(&baro);

      // Loc trung vi 3 mau
      baro_filtered_alt = median_filter_apply(&filter_baro_median, baro.altitude_m);

      // Kalman EKF do cao doc lap cho Barometer @ 50 Hz (dt = 0.02s)
      float az = imu_az_ned_mps2;
      if (fabsf(az - baro_ekf.accel_bias) < IMU_ACCEL_DEADBAND) {
        az = baro_ekf.accel_bias;
      }
      altitude_ekf_predict(&baro_ekf, az, 0.02f);
      altitude_ekf_update_baro(&baro_ekf, -baro_filtered_alt);

      // Luu vao struct rieng baro_state, khong ghi vao 12 trang thai drone_state
      baro_state.z  = -baro_ekf.z;
      baro_state.vz = -baro_ekf.vz;
    }
    vTaskDelay(20); // 50 Hz ODR cua BMP388
  }
  /* USER CODE END Barometers_task */
}

/* USER CODE BEGIN Header_Op_Flo_task */
/**
* @brief Function implementing the Op_Flo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Op_Flo_task */
void Op_Flo_task(void *argument)
{
  /* USER CODE BEGIN Op_Flo_task */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t last_cycle = DWT->CYCCNT;

  // Khoi tao bo loc trung vi (3 mau) va LPF bac 2 cho ToF (Cutoff 8 Hz @ 100 Hz Fs)
  median_filter_init(&filter_tof_median, 3);
  biquad2p_filter_init(&filter_opflow_z, 8.0f, 100.0f);
  median_filter_reset(&filter_tof_median, (float)opflow.raw_distance_cm * 0.01f);
  biquad2p_filter_reset(&filter_opflow_z, (float)opflow.raw_distance_cm * 0.01f);

  // Khoi tao bo loc Median (3 mau) va LPF bac 2 (Cutoff 20 Hz @ 100 Hz Fs) cho Optical Flow
  median_filter_init(&filter_flow_med_x, 3);
  median_filter_init(&filter_flow_med_y, 3);
  biquad2p_filter_init(&filter_flow_lpf_x, 20.0f, 100.0f);
  biquad2p_filter_init(&filter_flow_lpf_y, 20.0f, 100.0f);
  median_filter_reset(&filter_flow_med_x, 0.0f);
  median_filter_reset(&filter_flow_med_y, 0.0f);
  biquad2p_filter_reset(&filter_flow_lpf_x, 0.0f);
  biquad2p_filter_reset(&filter_flow_lpf_y, 0.0f);

  // Khoi tao EKF do cao ToF va EKF mat phang ngang X/Y @ 100 Hz
  altitude_ekf_init(&alt_ekf, 5.0f, 0.02f, 2.5f);
  horizontal_ekf_init(&ekf_x, FLOW_ACC_NOISE, FLOW_ACC_B_NOISE, FLOW_VEL_NOISE);
  horizontal_ekf_init(&ekf_y, FLOW_ACC_NOISE, FLOW_ACC_B_NOISE, FLOW_VEL_NOISE);

  /* Infinite loop @ 100 Hz */
  for(;;)
  {
    uint32_t now_cycle = DWT->CYCCNT;
    float dt = (float)(now_cycle - last_cycle) / (float)SystemCoreClock;
    last_cycle = now_cycle;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    // 0. Doc toan bo byte tu buffer trung gian va giai ma frame MAVLink
    mtf01p_update(&opflow);

    // 1. Doc raw do cao tu ToF (cm -> m)
    float raw_h = (float)opflow.raw_distance_cm * 0.01f;

    // 2. Loc trung vi 3 mau (triet tieu gai nhon) -> Loc thong thap LPF bac 2
    float med_h = median_filter_apply(&filter_tof_median, raw_h);
    float lpf_h = biquad2p_filter_apply(&filter_opflow_z, med_h);

    // 2b. Bu goc nghieng do cao: chieu thang dung xuong mat dat (Z-height thuc te)
    float tilt = cosf(drone_state.roll * DEG2RAD) * cosf(drone_state.pitch * DEG2RAD);
    float h    = lpf_h * (tilt > 0.5f ? tilt : 0.5f);

    // 2c. Loc Median 3 mau (diet gai) va LPF bac 2 (diet rang cua) cho raw optical flow
    float med_x = median_filter_apply(&filter_flow_med_x, opflow.raw_flow_x);
    float med_y = median_filter_apply(&filter_flow_med_y, opflow.raw_flow_y);
    flow_filt_x = biquad2p_filter_apply(&filter_flow_lpf_x, med_x);
    flow_filt_y = biquad2p_filter_apply(&filter_flow_lpf_y, med_y);

    // 3. Bu toc do goc Gyro cho Optical Flow
    gyro_pitch_rad = drone_state.pitch_rate * DEG2RAD;
    gyro_roll_rad  = drone_state.roll_rate  * DEG2RAD;

    flow_diff_x =  flow_filt_y - gyro_pitch_rad;
    flow_diff_y = -flow_filt_x + gyro_roll_rad;

    // 4. Van toc quang hoc do duoc (m/s)
    flow_meas_vx = flow_diff_x * h;
    flow_meas_vy = flow_diff_y * h;
    tof_filtered_alt = h;

    // 5. Chay Kalman EKF cho X, Y va Z truc tiep tai nhip 100 Hz
    float ax = imu_ax_linear;
    float ay = imu_ay_linear;
    float az = imu_az_ned_mps2;

    if (fabsf(ax - ekf_x.accel_bias) < IMU_ACCEL_DEADBAND) ax = ekf_x.accel_bias;
    if (fabsf(ay - ekf_y.accel_bias) < IMU_ACCEL_DEADBAND) ay = ekf_y.accel_bias;
    if (fabsf(az - alt_ekf.accel_bias) < IMU_ACCEL_DEADBAND) {
      az = alt_ekf.accel_bias;
      alt_ekf.vz *= 0.99f;
      if (fabsf(alt_ekf.vz) < 0.001f) alt_ekf.vz = 0.0f;
    }

    horizontal_ekf_predict(&ekf_x, ax, dt);
    horizontal_ekf_predict(&ekf_y, ay, dt);
    altitude_ekf_predict(&alt_ekf, az, dt);

    if (opflow.raw_distance_cm > 0) {
      horizontal_ekf_update_vel(&ekf_x, flow_meas_vx);
      horizontal_ekf_update_vel(&ekf_y, flow_meas_vy);
      altitude_ekf_update_tof(&alt_ekf, -h, TOF_ALT_NOISE);
    }

    // 6. Luu truc tiep vao bo 12 trang thai chung cua drone
    taskENTER_CRITICAL();
    drone_state.x  = ekf_x.pos;
    drone_state.y  = ekf_y.pos;
    drone_state.vx = ekf_x.vel;
    drone_state.vy = ekf_y.vel;
    drone_state.z  = -alt_ekf.z;
    drone_state.vz = -alt_ekf.vz;
    taskEXIT_CRITICAL();

    vTaskDelay(10); // 100 Hz
  }
  /* USER CODE END Op_Flo_task */
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
