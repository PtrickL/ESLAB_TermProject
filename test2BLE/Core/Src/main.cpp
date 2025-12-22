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
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>        // 為 sin()、M_PI
#include "stm32l475e_iot01_accelero.h"
//#include <stdio.h>
#include "stm32l475e_iot01_gyro.h"
#include <inttypes.h>
#include <stdbool.h>
#include "stm32l475e_iot01_magneto.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

using namespace ei;

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
CRC_HandleTypeDef hcrc;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/********************** 門板開關偵測邏輯 **************************/
#define DOOR_AXIS          	0        // 使用 X 軸 (你的門板旋轉軸)
#define DOOR_SIGN              -1.0f    // 如果開門為負角速度 → -1

#define VEL_OPEN_THR        	6.0f    // 檢測開門速度 (deg/s)
#define VEL_CLOSE_THR          -6.0f    // 檢測關門速度 (deg/s)
#define VEL_STOP_THR        	1.0f    // 靜止速度 (deg/s)
#define TWIST_THR 	    	5.0f   // deg/s，可自行調
#define KNOCK_THR 		300.0f   // 依你實測調整 (mg)

#define MARGIN		    	3.0f

#define ANGLE_OPEN_THR    	(angle_open_cal)
#define ANGLE_CLOSE_THR   	(angle_close_cal)

#define DEBOUNCE_COUNT       	10      // 約 1 秒 (10Hz sample rate)

#define ACC_SENS_2G 	  	(0.000061f)  // 0.061 mg/LSB

#define ACC_TAPS 		5
#define GYRO_TAPS 		7

#define AUDIO_REC 		8000
#define AUDIO_IPT 		6400

#define knockThres 		0.95
#define speechThres 		0.75
#define kFlagThres 		2
#define sFlagThres 		1

#define DOOR_KNOCKED		4
#define DOOR_SPEECH		5
#define ENV_NOISE		6



int16_t accelXYZ[3];
// === Bias ===
float acc_bias[3]  = {0};
float gyro_bias[3] = {0};

float32_t acc_fir_coeffs[ACC_TAPS]   = { 0.2f,0.2f,0.2f,0.2f,0.2f };
float32_t gyro_fir_coeffs[GYRO_TAPS] = { 1.0f/7,1.0f/7,1.0f/7,1.0f/7,1.0f/7,1.0f/7,1.0f/7 };

arm_fir_instance_f32 acc_fir[3];
arm_fir_instance_f32 gyro_fir[3];

float acc_state[3][ACC_TAPS];
float gyro_state[3][GYRO_TAPS];

float acc_corr[3], acc_filt[3];
float gyro_corr[3], gyro_filt[3];

float acc_mag = 0;

float angle_open_cal  = 0.0f;
float angle_close_cal = 0.0f;

float fused_angle = 0.0f;
unsigned int last_tick = 0;

volatile uint8_t calib_step = 0;
volatile uint8_t calib_btn_pressed = 0;

volatile uint8_t door_state_global = 0;
volatile uint8_t event_knock_detected = 0;
volatile uint8_t event_handle_twist = 0;
volatile uint8_t fFlag = 0;

int32_t RecBuf[AUDIO_REC];

int32_t scount = 0;
int32_t stick = 0;
uint16_t ii = 0;
static uint16_t audioOffset = AUDIO_REC-AUDIO_IPT;

signal_t ei_signal;

osSemaphoreId_t accelSemHandle;
osMessageQueueId_t accelQueueHandle;

typedef struct {
	int16_t data[3];
}accelData_t;

typedef enum {
	DOOR_CLOSED = 0,
	DOOR_OPENING,
	DOOR_OPEN,
	DOOR_CLOSING
} DoorState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/*int __io_putchar( int ch )
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;

}*/
void vprint(const char *fmt, va_list argp)
{
  char string[200];
  if(0 < vsprintf(string, fmt, argp)) // build string
  {
    HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
  }
}

void ei_printf(const char *format, ...) {
  va_list myargs;
  va_start(myargs, format);
  vprint(format, myargs);
  va_end(myargs);
}

int get_feature_data(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (float)(RecBuf[audioOffset+offset+i] >> 11);
  }
  return 0;
}

void init_fir_filters(void)
{
  for (int axis = 0; axis < 3; axis++)
  {
    arm_fir_init_f32(&acc_fir[axis], ACC_TAPS, acc_fir_coeffs, acc_state[axis], 1);
    arm_fir_init_f32(&gyro_fir[axis], GYRO_TAPS, gyro_fir_coeffs, gyro_state[axis], 1);
  }
}

void calibrate_bias(int samples)
{
  float acc_sum[3] = {0};
  float gyro_sum[3] = {0};

  for (int n = 0; n < samples; n++)
  {
    int16_t xyz[3];
    float gxyz[3];

    BSP_ACCELERO_AccGetXYZ(xyz);
    BSP_GYRO_GetXYZ(gxyz);


    acc_sum[0] += xyz[0];
    acc_sum[1] += xyz[1];
    acc_sum[2] += xyz[2];

    gyro_sum[0] += gxyz[0];
    gyro_sum[1] += gxyz[1];
    gyro_sum[2] += gxyz[2];

    HAL_Delay(5);
  }

  for (int i = 0; i < 3; i++)
  {
    acc_bias[i]  = acc_sum[i]  / samples;
    gyro_bias[i] = gyro_sum[i] / samples;
  }
}
void remove_bias(float ax, float ay, float az,
                 float gx, float gy, float gz)
{
    acc_corr[0] = ax - acc_bias[0];
    acc_corr[1] = ay - acc_bias[1];
    acc_corr[2] = az - acc_bias[2];

    gyro_corr[0] = gx - gyro_bias[0];
    gyro_corr[1] = gy - gyro_bias[1];
    gyro_corr[2] = gz - gyro_bias[2];
}

void compute_acc_magnitude(void)
{
    acc_mag = sqrtf(acc_filt[0]*acc_filt[0] +
                    acc_filt[1]*acc_filt[1] +
                    acc_filt[2]*acc_filt[2]);
}
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
  ei_signal.total_length = AUDIO_IPT;
  ei_signal.get_data = &get_feature_data;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  BSP_MAGNETO_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);      // 啟動 PWM
 // printf("PWM 10Hz started\r\n");
  HAL_Delay(100);

  // Try sending raw data to every likely port


  HAL_TIM_Base_Start_IT(&htim2);
  init_fir_filters();
  calibrate_bias(200);
  for (int i=0; i<5; i++) {
    MX_BlueNRG_MS_Process();   // 先跑幾次 stack
    HAL_Delay(10);
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  accelSemHandle =  osSemaphoreNew( 1, 1, NULL);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  accelQueueHandle = osMessageQueueNew(16, sizeof(accelData_t), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 125;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c2.Init.Timing = 0x00000E14;
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
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin
                          |ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin SPSGRF_915_SDN_Pin
                           ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin
                          |ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);


  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_EXTI13_Pin)
  {
    calib_btn_pressed = 1;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    osSemaphoreRelease(accelSemHandle);
  }
}


// --- END OF MISSING GLUE CODE ---
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
  fFlag = 1;
  stick = HAL_GetTick();
  scount += 1;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  printf("Starting Default in 5 Seconds\r\n");
  int32_t tt, countSmp, cooldownTimer;
  int16_t kFlag = 0, sFlag = 0, dFlag = 0;
  int16_t xyz[3];
  float gxyz[3];

  osDelay(2000);
  if(HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuf, AUDIO_REC)!=HAL_OK){
    printf("DMA Failed to Start\r\n");
  }
  osDelay(3000);

  while(1){
    if(fFlag==1){
      tt = stick;
      countSmp = scount;
      fFlag = 0;
      break;
    }
    osDelay(10);
  }
  while(1){
    if(fFlag==1){
      tt = stick-tt;
      countSmp = scount-countSmp;
      fFlag = 0;
      break;
    }
    osDelay(10);
  }

//  printf("Cycles Sampled = %d \r\n", countSmp);
//  printf("Average Time = %d \r\n", tt/countSmp);
//  printf("Classification Starting in 3 Seconds \r\n");
//  osDelay(3000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  for(;;)
  {
    MX_BlueNRG_MS_Process();
    if(osSemaphoreAcquire(accelSemHandle,osWaitForever) == osOK)
    {
      BSP_ACCELERO_AccGetXYZ(xyz);
      BSP_GYRO_GetXYZ(gxyz);

      float ax = (float)xyz[0];
      float ay = (float)xyz[1];
      float az = (float)xyz[2];

      float gx = gxyz[0];
      float gy = gxyz[1];
      float gz = gxyz[2];

      // 2. 去 bias
      remove_bias(ax, ay, az, gx, gy, gz);

      // 3. FIR 濾波
      for (int axis = 0; axis < 3; axis++) {
	  arm_fir_f32(&acc_fir[axis],  &acc_corr[axis],  &acc_filt[axis], 1);
	  arm_fir_f32(&gyro_fir[axis], &gyro_corr[axis], &gyro_filt[axis], 1);
      }

      // 4. magnitude
      compute_acc_magnitude();
      acc_mag = acc_mag *ACC_SENS_2G*1000;


      static int debounce = 0;
      static DoorState state = DOOR_OPEN;

      float gyro_rate = gyro_filt[DOOR_AXIS] * DOOR_SIGN / 1000.0f;   // deg/s

      /* ----------- 磁力計 heading 計算 ----------- */
      int16_t mxyz[3];
      static float heading_ref = 0.0f;   // 門全關時的 heading 基準
      static int ref_set = 0;

      // 讀取磁力計
      BSP_MAGNETO_GetXYZ(mxyz);

      float my = (float)mxyz[1];
      float mz = (float)mxyz[2];

      float heading_rad = atan2f(mz, my);
      float heading_deg = heading_rad * 180.0f / M_PI;

      // 第一次在「門關」狀態時，把此 heading 當作 0 度基準
      if (!ref_set && calib_step==0){
	heading_ref = heading_deg;
	ref_set = 1;
      }

      unsigned int current_tick = HAL_GetTick();
      float dt = (current_tick - last_tick) / 1000.0f;
      last_tick = current_tick;
      if (dt > 0.1f) dt = 0.1f; // Safety cap

      float mag_angle = heading_deg - heading_ref;
      float gyro_dps = gyro_rate;
      fused_angle += gyro_dps*dt;
      float error = mag_angle - fused_angle;

      if (error > 180.0f)  error -= 360.0f;
      if (error < -180.0f) error += 360.0f;

      fused_angle += 0.3f * error;
      float door_angle = fused_angle;

      if (door_angle > 180.0f)  door_angle -= 360.0f;
      if (door_angle < -180.0f) door_angle += 360.0f;

      if (calib_btn_pressed) {
	calib_btn_pressed = 0;   // consume event

	if (calib_step == 0) {
	  angle_close_cal = door_angle;
	  calib_step = 1;
	  printf("[CALIB] Close raw = %.2f  final = %.2f\r\n",
		 door_angle, angle_close_cal);
	}
	else if (calib_step == 1) {
	  angle_open_cal = door_angle;
	  calib_step = 2;
	  printf("[CALIB] Open raw = %.2f  final = %.2f\r\n",
		   door_angle, angle_open_cal);
	  printf("[CALIB] Calibration finished!\r\n");
	}
      }

//      printf("MAG: my=%.1f mz=%.1f  door_angle=%.2f deg\r\n", my, mz, door_angle);


      if (calib_step < 2) {
	// 校正階段 → 不做開關門判斷
	printf("[CALIB] Current angle = %.2f deg\r\n", door_angle);
	continue;   // skip state machine
      }

      switch(state)
      {
      case DOOR_CLOSED:
      {
//	if (acc_mag > KNOCK_THR) {
//	   event_knock_detected = 1;
//	   printf("[EVENT] Knock detected! acc_mag=%.1f\r\n", acc_mag);
//	}
	if(abs(door_angle-ANGLE_CLOSE_THR) > MARGIN/2){
	  door_angle = ANGLE_CLOSE_THR;
	}

	if (gyro_rate > VEL_OPEN_THR) {
	  state = DOOR_OPENING;
	  door_state_global = DOOR_OPENING;
	  event_handle_twist = 0;
	  debounce = 0;
	  dFlag = 0;
	  printf("[EVENT] Door opening...\r\n");
	}

	float gz_dps = gyro_filt[2] / 1000.0f;
	if (fabsf(gz_dps)>TWIST_THR && event_handle_twist==0 && gz_dps>0) {
	  event_handle_twist = 1;
	  printf("[EVENT] Handle twist detected! gz=%.1f dps\r\n", gz_dps);
	  continue;
	}else if (fabsf(gz_dps)<TWIST_THR && event_handle_twist==1) {
	  event_handle_twist = 0;
	}

	break;
      }
      case DOOR_OPENING:
      {
	if (fabsf(gyro_rate) < VEL_STOP_THR) {
	  debounce++;
	  if (debounce>=DEBOUNCE_COUNT && abs(door_angle-ANGLE_OPEN_THR)<MARGIN) {
	    state = DOOR_OPEN;
	    door_state_global = DOOR_OPEN;
	    printf("[EVENT] Door fully open (angle=%.1f)\r\n", door_angle);
	  }
	}
	else debounce = 0;

	if (gyro_rate < VEL_CLOSE_THR) {
	  state = DOOR_CLOSING;
	  door_state_global = DOOR_CLOSING;
	  debounce = 0;
	  printf("[EVENT] Door closing (from mid / open)\r\n");
	}
	break;
      }
      case DOOR_OPEN:
      {
	if (gyro_rate < VEL_CLOSE_THR) {
	  state = DOOR_CLOSING;
	  door_state_global = DOOR_CLOSING;
	  debounce = 0;
	  printf("[EVENT] Door closing...\r\n");
	}
	break;
      }
      case DOOR_CLOSING:
      {
	if (fabsf(gyro_rate) < VEL_STOP_THR) {
	  debounce++;
	  if (debounce>=DEBOUNCE_COUNT && abs(door_angle-ANGLE_CLOSE_THR)<MARGIN) {
	    state = DOOR_CLOSED;
	    door_state_global = DOOR_CLOSED;
	    cooldownTimer = HAL_GetTick() + 1500;
	    dFlag = 1;
	    printf("[EVENT] Door closed (angle=%.1f)\r\n", door_angle);
	  }
	}
	else debounce = 0;

	if (gyro_rate > VEL_OPEN_THR) {
	  state = DOOR_OPENING;
	  door_state_global = DOOR_OPENING;
	  debounce = 0;
	  printf("[EVENT] Door opening again\r\n");
	}
	break;
      }
      }
//      printf("mx=%d, my=%d, mz=%d, angle=%.2f\n", mxyz[0], mxyz[1], mxyz[2], door_angle);
//      printf("gyro=%.2f, angle=%.2f, state=%d\r\n", gyro_rate, door_angle, state);
    }
    if(fFlag){
      if(HAL_GetTick()<cooldownTimer || event_handle_twist){
        fFlag = 0;
      }else if(dFlag){
	fFlag = 0;
	ei_impulse_result_t result = { 0 };
	EI_IMPULSE_ERROR res = run_classifier(&ei_signal, &result, false);

	if(result.classification[0].value>=knockThres){
	  if(++kFlag >= kFlagThres){
	    kFlag = 0;
	    door_state_global = DOOR_KNOCKED;
	    ei_printf("[EVENT] Door Knocked,    Predictions: [");
	    ei_printf_float(result.classification[0].value);
	    ei_printf(", ");
	    ei_printf_float(result.classification[1].value);
	    ei_printf(", ");
	    ei_printf_float(result.classification[2].value);
	    ei_printf("]\r\n");
	  }
	}else if(result.classification[2].value>=speechThres){
	  if(++sFlag >= sFlagThres){
	    sFlag = 0;
	    door_state_global = DOOR_SPEECH;
	    ei_printf("[EVENT] Speech Detected, Predictions: [");
	    ei_printf_float(result.classification[0].value);
	    ei_printf(", ");
	    ei_printf_float(result.classification[1].value);
	    ei_printf(", ");
	    ei_printf_float(result.classification[2].value);
	    ei_printf("]\r\n");
	  }
	}else{
	  door_state_global = ENV_NOISE;
	}
  //      ei_printf("Predictions: ");

	// print the predictions
  //      ei_printf("[");
  //      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
  //	ei_printf_float(result.classification[ix].value);
  //#if EI_CLASSIFIER_HAS_ANOMALY == 1
  //	ei_printf(", ");
  //#else
  //	if (ix != EI_CLASSIFIER_LABEL_COUNT - 1){
  //	  ei_printf(", ");
  //	}
  //#endif
  //      }
  //#if EI_CLASSIFIER_HAS_ANOMALY == 1
  //      ei_printf_float(result.anomaly);
  //#endif
  //      ei_printf("]\r\n");
      }
    }
    osDelay(2);
  }
  /* USER CODE END 5 */
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
