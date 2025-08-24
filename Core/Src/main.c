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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rclc/sleep.h"  // Micro-ROS sleep header
//#include <rmw_uros/options.h>
#include <rcutils/allocator.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/subscription.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <geometry_msgs/msg/twist.h>

//#include <geometry_msgs/msg/vector3.h>
//#include <geometry_msgs/msg/quaternion.h>


#include "I2C.h"
#include "BNO055.h"
#include "IMU.h"
#include "RC522.h"
#include "PressureSensor.h"
#include "PWM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//extern USBD_HandleTypeDef hUsbDeviceFS;

rcl_publisher_t imu_publisher;
rcl_publisher_t pressure_publisher;
rcl_publisher_t magneto_publisher;
rcl_publisher_t heading_publisher;

rcl_subscription_t cmd_vel_subscriber;

rcl_ret_t spin_rs;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__FluidPressure pressure_msg;
sensor_msgs__msg__MagneticField magneto_msg;
std_msgs__msg__Float32 heading_msg;

geometry_msgs__msg__Twist cmd_vel_msg;

//osMutexId_t microrosMsgMutexHandle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for Thread_MicroROS */
osThreadId_t Thread_MicroROSHandle;
const osThreadAttr_t Thread_MicroROS_attributes = {
  .name = "Thread_MicroROS",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Thread_Sensors */
osThreadId_t Thread_SensorsHandle;
const osThreadAttr_t Thread_Sensors_attributes = {
  .name = "Thread_Sensors",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Thread_Thruster */
osThreadId_t Thread_ThrusterHandle;
const osThreadAttr_t Thread_Thruster_attributes = {
  .name = "Thread_Thruster",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Microros_DataMutex */
osMutexId_t Microros_DataMutexHandle;
const osMutexAttr_t Microros_DataMutex_attributes = {
  .name = "Microros_DataMutex"
};
/* Definitions for Microros_cmd_vel_Mutex */
osMutexId_t Microros_cmd_vel_MutexHandle;
const osMutexAttr_t Microros_cmd_vel_Mutex_attributes = {
  .name = "Microros_cmd_vel_Mutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void TaskMicroROS(void *argument);
void TaskSensors(void *argument);
void TaskThruster(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void rclc_sleep_ms(unsigned int milliseconds)
{
    osDelay(milliseconds);  // FreeRTOS delay
}

uint8_t status, str[16], sNum[5];
char* msg1 = "Reading from card\r\n";
char* msg2 = "Reading from tag\r\n";
uint16_t pwm_signal = 1500;
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
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Microros_DataMutex */
  Microros_DataMutexHandle = osMutexNew(&Microros_DataMutex_attributes);

  /* creation of Microros_cmd_vel_Mutex */
  Microros_cmd_vel_MutexHandle = osMutexNew(&Microros_cmd_vel_Mutex_attributes);

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
  /* creation of Thread_MicroROS */
  Thread_MicroROSHandle = osThreadNew(TaskMicroROS, NULL, &Thread_MicroROS_attributes);

  /* creation of Thread_Sensors */
  Thread_SensorsHandle = osThreadNew(TaskSensors, NULL, &Thread_Sensors_attributes);

  /* creation of Thread_Thruster */
  Thread_ThrusterHandle = osThreadNew(TaskThruster, NULL, &Thread_Thruster_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c2.Init.Timing = 0x2010091A;
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
  I2C_Init(&hi2c2, I2CNO_2);
  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x2010091A;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  I2C_Init(&hi2c3, I2CNO_3);
  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);





void HAL_Delay(uint32_t Delay)
{
  osDelay(Delay);
}

uint16_t map_thrust_to_pwm(float thrust)
{
    if (thrust > 1.0f) thrust = 1.0f;
    if (thrust < -1.0f) thrust = -1.0f;

    uint16_t pwm_neutral = 1500;
    uint16_t pwm_max = 1900;
    uint16_t pwm_min = 1100;

    if (thrust >= 0.0f)
    {
        // Pozitif thrust: nötr → max
        return (uint16_t)(pwm_neutral + thrust * (pwm_max - pwm_neutral));
    }
    else
    {
        // Negatif thrust: nötr → min
        return (uint16_t)(pwm_neutral + thrust * (pwm_neutral - pwm_min));
    }
}

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear_x  = msg->linear.x;   // ileri/geri
    float linear_y  = msg->linear.y;   // sağ/sol
    float angular_z = msg->angular.z;  // yaw dönmesi

    // Basit thrust dağılımı (ROV’un pervane yönlerine göre katsayıyla ayarlanmalı)
    float thrust_SA =  linear_x - linear_y - angular_z;  // Sağ Ön
    float thrust_SB =  linear_x + linear_y - angular_z;  // Sağ Arka
    float thrust_PA =  linear_x + linear_y + angular_z;  // Sol Ön
    float thrust_PB =  linear_x - linear_y + angular_z;  // Sol Arka

    // Thrust → PWM dönüşümü (örnek, motor karakteristiğine göre ayarlanmalı)
    THRUSTER_Vert_R.PWM_Raw = thrust_SA;
    THRUSTER_Vert_L.PWM_Raw = thrust_SB;
    THRUSTER_Horz_R.PWM_Raw = thrust_PA;
    THRUSTER_Horz_L.PWM_Raw = thrust_PB;

    THRUSTER_Vert_R.dataNumb++;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_TaskMicroROS */
/**
  * @brief  Function implementing the Thread_MicroROS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskMicroROS */
void TaskMicroROS(void *argument)
{
  /* USER CODE BEGIN 5 */

	static char frame_id_imu[] = "imu_link";
	static char frame_id_pressure[] = "pressure_link";
	static char frame_id_magneto[] = "magneto_link";
	static char frame_id_heading[] = "heading_link";

	rcl_allocator_t allocator;
	rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t cmd_vel_executor;

	rmw_uros_set_custom_transport(true, (void *) &huart2, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);

	allocator = rcutils_get_zero_initialized_allocator();

	allocator.allocate = microros_allocate;
	allocator.deallocate = microros_deallocate;
	allocator.reallocate = microros_reallocate;
	allocator.zero_allocate = microros_zero_allocate;

	rcutils_set_default_allocator(&allocator);

	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "my_node", "", &support);

	//microros sub and pub inits

	rclc_subscription_init_best_effort(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

	rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data");

	rclc_publisher_init_best_effort(&pressure_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure), "pressure_data");

	rclc_publisher_init_best_effort(&magneto_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "magneto_data");

	rclc_publisher_init_best_effort(&heading_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "compass_heading");

	//microros executor inits
	rclc_executor_init(&cmd_vel_executor, &support.context, 1, &allocator);

	rclc_executor_add_subscription(&cmd_vel_executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ALWAYS);

	memset(&imu_msg, 0, sizeof(imu_msg));
	memset(&pressure_msg, 0, sizeof(pressure_msg));
	memset(&magneto_msg, 0, sizeof(magneto_msg));
    memset(&heading_msg, 0, sizeof(heading_msg));

	for(;;){

		imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
		imu_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
		imu_msg.header.frame_id.data = frame_id_imu;
		imu_msg.header.frame_id.size = strlen(frame_id_imu);
		imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

		pressure_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
		pressure_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
		pressure_msg.header.frame_id.data = frame_id_pressure;
		pressure_msg.header.frame_id.size = strlen(frame_id_pressure);
		pressure_msg.header.frame_id.capacity = pressure_msg.header.frame_id.size + 1;

		magneto_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
		magneto_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
		magneto_msg.header.frame_id.data = frame_id_magneto;
		magneto_msg.header.frame_id.size = strlen(frame_id_magneto);
		magneto_msg.header.frame_id.capacity = magneto_msg.header.frame_id.size + 1;

		//publish data with mutex


		//if(osMutexAcquire(Microros_cmd_vel_MutexHandle, 10) == osOK){

		    spin_rs = rclc_executor_spin_some(&cmd_vel_executor, RCL_MS_TO_NS(20));

		//	osMutexRelease(Microros_cmd_vel_MutexHandle);
		//}


/*		if(osMutexAcquire(Microros_DataMutexHandle, 10) == osOK){

			rcl_publish(&imu_publisher, &imu_msg, NULL);

			rcl_publish(&pressure_publisher, &pressure_msg, NULL);

			rcl_publish(&magneto_publisher, &magneto_msg, NULL);

			rcl_publish(&heading_publisher, &heading_msg, NULL);

			osMutexRelease(Microros_DataMutexHandle);
		}*/

		osDelay(20);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TaskSensors */
/**
* @brief Function implementing the Thread_Sensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskSensors */
void TaskSensors(void *argument)
{
  /* USER CODE BEGIN TaskSensors */

	IMU_Init(&IMU_1, 1, I2CNO_2, BNO055_I2C_ADRESS, 0, 0);
	PressureSensor_Init(&PressureSensor_1, 1, I2CNO_3, 0x40);
	osDelay(100);

  /* Infinite loop */
  for(;;)
  {

	IMU_Execute(&IMU_1, 1);
//	PressureSensor_Execute(&PressureSensor_1, 1);

	if(osMutexAcquire(Microros_DataMutexHandle, 10) == osOK){

	imu_msg.linear_acceleration.x = IMU_1.CalculatedData.LinAcc.X_Axis;
	imu_msg.linear_acceleration.y = IMU_1.CalculatedData.LinAcc.Y_Axis;
	imu_msg.linear_acceleration.z = IMU_1.CalculatedData.LinAcc.Z_Axis;

	imu_msg.angular_velocity.x = IMU_1.CalculatedData.Gyroscope.X_Axis;
	imu_msg.angular_velocity.y = IMU_1.CalculatedData.Gyroscope.Y_Axis;
	imu_msg.angular_velocity.z = IMU_1.CalculatedData.Gyroscope.Z_Axis;

	imu_msg.orientation.w = IMU_1.CalculatedData.Qua.W_Axis;
	imu_msg.orientation.x = IMU_1.CalculatedData.Qua.X_Axis;
	imu_msg.orientation.y = IMU_1.CalculatedData.Qua.Y_Axis;
	imu_msg.orientation.z = IMU_1.CalculatedData.Qua.Z_Axis;

	magneto_msg.magnetic_field.x = IMU_1.CalculatedData.Magnetometer.X_Axis;
	magneto_msg.magnetic_field.y = IMU_1.CalculatedData.Magnetometer.Y_Axis;
	magneto_msg.magnetic_field.z = IMU_1.CalculatedData.Magnetometer.Z_Axis;

	pressure_msg.fluid_pressure = PressureSensor_1.FilteredPressureDataPascal;

	heading_msg.data = IMU_1.CalculatedData.Heading;

	osMutexRelease(Microros_DataMutexHandle);
	}

	osDelay(10);
  }
  /* USER CODE END TaskSensors */
}

/* USER CODE BEGIN Header_TaskThruster */
/**
* @brief Function implementing the Thread_Thruster thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskThruster */
void TaskThruster(void *argument)
{
  /* USER CODE BEGIN TaskThruster */
	PWM_Init(&THRUSTER_Vert_R, &htim4, TIM_CHANNEL_1, 1100, 1900, 1500);
	PWM_Init(&THRUSTER_Vert_L, &htim4, TIM_CHANNEL_2, 1100, 1900, 1500);
	PWM_Init(&THRUSTER_Horz_R, &htim4, TIM_CHANNEL_3, 1100, 1900, 1500);
	PWM_Init(&THRUSTER_Horz_L, &htim4, TIM_CHANNEL_4, 1100, 1900, 1500);
	osDelay(100);
  /* Infinite loop */
  for(;;)
  {

	//  if(osMutexAcquire(Microros_cmd_vel_MutexHandle, 10) == osOK){

		//  PWM_SetPulse(&THRUSTER_Vert_R);
		 // PWM_SetPulse(&THRUSTER_Vert_L);
		 // PWM_SetPulse(&THRUSTER_Horz_R);
		 // PWM_SetPulse(&THRUSTER_Horz_L);

	//	  osMutexRelease(Microros_cmd_vel_MutexHandle);
	//  }

    osDelay(10);
  }
  /* USER CODE END TaskThruster */
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
  if (htim->Instance == TIM1)
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
