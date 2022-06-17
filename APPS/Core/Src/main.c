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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FEB_logger.h"
#include "FEB_CAN.h"
#include "FEB_ADC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct {
  uint16_t torque;
  uint8_t enabled;

  uint8_t other_faults;

  uint8_t prestop_called;

  uint16_t setpoint;
  float k_p;
  float k_i;
  float period;
  uint16_t error_sum;
  uint16_t error_sum_counter;
  uint16_t error_sum_thresh;
  uint16_t windup_thresh;

  float motor_speed;

  float throttle_thresh; //  In Watts
  float abs_thresh; //  In Watts
} RMSControl;

struct {
  uint8_t enabled;

  float motor_speed;
  float current; // In amps times 10
  float voltage; // In volts times 10
} RMSStatus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODULE_NAME     "APPS"

#define BRAKE_PEDAL_RESET       0.513
#define BRAKE_PEDAL_FULL        0.602

#define ACC_PEDAL_0_RESET       1.128
#define ACC_PEDAL_0_FULL        0.305
#define ACC_PEDAL_1_RESET       1.579
#define ACC_PEDAL_1_FULL        1.294

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

CAN_HandleTypeDef *RMS_CANx;

void FEB_RMS_init(CAN_HandleTypeDef *CANx);
void FEB_RMS_updateTorque();
void FEB_RMS_disable();
void FEB_RMS_enable();
void FEB_RMS_setTorque(uint16_t torque);

float FEB_APPS_getBrakePedal();
float FEB_APPS_getAccPedal();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  // listen to only frame with id==0x200
  FEB_CAN_initFilter(&hcan1, 0x200, 0xFFF);


  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    while (1)
    FEB_log(MODULE_NAME, "ERROR", "CAN1 initialization error");
  }

  FEB_RMS_init(&hcan1);

  FEB_RMS_setTorque(0);

// START OF debug only
  for (uint16_t i=0; i<50; i+=1) {
    FEB_RMS_setTorque(0);
    HAL_Delay(100);
  }

//  FEB_RMS_enable();
// END OF debug only

  FEB_log(MODULE_NAME, "INFO", "RMS Enabled");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    float brake_pedal = FEB_APPS_getBrakePedal();
    float acc_pedal = FEB_APPS_getAccPedal();

    float brake_pressure[2];
    brake_pressure[0] = FEB_ADC_sampleChannel(&hadc1, ADC_CHANNEL_0);
    brake_pressure[1] = FEB_ADC_sampleChannel(&hadc1, ADC_CHANNEL_1);

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0 || HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) > 0) {

      CAN_RxHeaderTypeDef header;
      uint8_t buffer[8];

      HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, buffer);

      if (header.StdId == 0x200 && !buffer[0]) {
        FEB_RMS_enable();

        FEB_log(MODULE_NAME, "INFO", "RMS Enabled");
      }
      if (header.StdId == 0x200 && buffer[0]) {
        FEB_RMS_disable();

        FEB_log(MODULE_NAME, "INFO", "RMS Disabled");
      }
    }

    float pedal_state[2];
    pedal_state[0] = brake_pedal;
    pedal_state[1] = acc_pedal;
    FEB_CAN_transmit(&hcan1, 0x201, (uint8_t *)pedal_state, 8, 0);


    uint8_t brake_light_control = 0b01000000;
    FEB_CAN_transmit(&hcan1, 0x300, &brake_light_control, 1, 0);

    // if we are braking, we dont need torque...
    // coefficient in Nm
    uint16_t torque = acc_pedal * 50;
//    uint16_t torque = (brake_pedal < 0.1) ? acc_pedal * 50 : 0;

    FEB_RMS_setTorque(torque);

    char str[128];
    sprintf(str, "<APPS> [INFO] BRAKE(0x100): %f \tACC(0x101): %f \ttorque: %d\r\n", brake_pedal, acc_pedal, torque);
    FEB_log(MODULE_NAME, "INFO", str);

    HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 8;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void FEB_RMS_init(CAN_HandleTypeDef *CANx) {
  RMS_CANx = CANx;

  RMSControl.enabled = 0;
  RMSControl.torque = 0;
  RMSControl.prestop_called = 0;

  RMSControl.motor_speed = 0;
  RMSControl.setpoint = 500;

  RMSControl.k_p = 0.16;
  RMSControl.k_i = 0.05;
  RMSControl.period = 0.01;
  RMSControl.windup_thresh = 3000;

  RMSControl.error_sum = 0;
  RMSControl.error_sum_counter = 0;
  RMSControl.error_sum_thresh = 1000;


  RMSControl.throttle_thresh = 10. * 1000;
  RMSControl.abs_thresh = 15. * 1000;

  RMSControl.other_faults = -1;

  uint16_t can_id = 0x411;  // hex(1041)
  uint32_t delay_between_message = 5;
  uint8_t is_blocking = 1;

  uint8_t msg_off_data[8] = {0x34, 0, 1, 0, 0, 0, 0, 0};
  FEB_CAN_transmit(RMS_CANx, can_id, msg_off_data, 8, is_blocking);
  HAL_Delay(delay_between_message);

  // Set voltage 1 freqency
  uint8_t v1_freq_data[8] = {0x21, 2, 0, 10, 0, 0, 0, 0};
  FEB_CAN_transmit(RMS_CANx, can_id, v1_freq_data, 8, is_blocking);
  HAL_Delay(delay_between_message);

  // Turn off voltage 2
  uint8_t v2_off_data[8] = { 0x22, 0, 0, 0, 0, 0, 0 };
  FEB_CAN_transmit(RMS_CANx, can_id, v2_off_data, 8, is_blocking);
  HAL_Delay(delay_between_message);

  // Turn off voltage 3
  uint8_t v3_off_data[8] = { 0x23, 0, 0, 0, 0, 0, 0, 0 };
  FEB_CAN_transmit(RMS_CANx, can_id, v3_off_data, 8, is_blocking);
  HAL_Delay(delay_between_message);

  // Lower energy frequency
  uint8_t lower_energy_data[8] = { 0x27, 2, 0, 0x64, 0, 0, 0, 0 };
  FEB_CAN_transmit(RMS_CANx, can_id, lower_energy_data, 8, is_blocking);
  HAL_Delay(delay_between_message);

  // Store
  uint8_t store_data[8] = { 0x32, 0, 0, 0, 0, 0, 0, 0 };
  FEB_CAN_transmit(RMS_CANx, can_id, store_data, 8, is_blocking);
  HAL_Delay(delay_between_message);

  // Turn on messages
  uint8_t msg_on_data[8] = { 0x34, 1, 1, 0, 0, 0, 0, 0 };
  FEB_CAN_transmit(RMS_CANx, can_id, msg_on_data, 8, is_blocking);
  HAL_Delay(delay_between_message);



  for (int i=0; i<20; i+=1) {
    FEB_RMS_setTorque(0);
    HAL_Delay(100);
  }
}

void FEB_RMS_updateTorque() {
  uint8_t buffer[8] = {RMSControl.torque & 0xFF, RMSControl.torque >> 8, 0, 0, 0, RMSControl.enabled, 0, 0};
  FEB_CAN_transmit(RMS_CANx, 0x0C0, buffer, 8, 0);
}

void FEB_RMS_disable() {
  uint8_t pump_control = 0b01010000;
  FEB_CAN_transmit(&hcan1, 0x305, &pump_control, 1, 0);

  RMSControl.enabled = 0;
  FEB_RMS_updateTorque();
}

void FEB_RMS_enable() {
  uint8_t pump_control = 0b01010001;
  FEB_CAN_transmit(&hcan1, 0x305, &pump_control, 1, 0);

  RMSControl.enabled = 1;
  FEB_RMS_updateTorque();
}

void FEB_RMS_setTorque(uint16_t torque) {
  RMSControl.torque = torque * 10;
  FEB_RMS_updateTorque();
}

float FEB_APPS_getBrakePedal() {
  float brake_pedal = FEB_ADC_sampleChannel(&hadc1, ADC_CHANNEL_13);
//  char str[128];
//  sprintf(str, "%f\t", brake_pedal);
//  FEB_log("", "", str);
  float brake_normalized = (brake_pedal - BRAKE_PEDAL_RESET) / (BRAKE_PEDAL_FULL - BRAKE_PEDAL_RESET);

  /* clamp */
  brake_normalized = brake_normalized > 1 ? 1 : brake_normalized;
  brake_normalized = brake_normalized < 0.05 ? 0 : brake_normalized;

  return brake_normalized;
}

float FEB_APPS_getAccPedal() {
  float acc_pedal_0 = FEB_ADC_sampleChannel(&hadc1, ADC_CHANNEL_10);
  float acc_pedal_1 = FEB_ADC_sampleChannel(&hadc1, ADC_CHANNEL_11);
//  char str[128];
//  sprintf(str, "%f\t%f\t", acc_pedal_0, acc_pedal_1);
//  FEB_log("", "", str);

  float acc_0_normalized = (acc_pedal_0 - ACC_PEDAL_0_RESET) / (ACC_PEDAL_0_FULL - ACC_PEDAL_0_RESET);
  float acc_1_normalized = (acc_pedal_1 - ACC_PEDAL_1_RESET) / (ACC_PEDAL_1_FULL - ACC_PEDAL_1_RESET);

  float acc_normalized = 0.5 * (acc_0_normalized + acc_1_normalized);

  /* clamp */
  acc_normalized = acc_normalized > 1 ? 1 : acc_normalized;
  acc_normalized = acc_normalized < 0.05 ? 0 : acc_normalized;

  return acc_normalized;
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
