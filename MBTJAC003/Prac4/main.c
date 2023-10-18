/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 128     // Number of samples in LUT
#define TIM2CLK 8000000  // STM Clock frequency
#define F_SIGNAL 1000 // Frequency of output analog signal
#define DEBOUNCE_DELAY 10  // Debounce interval in milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t Sin_LUT[NS] = {5.120000000000000000e+02,
		5.370000000000000000e+02,
		5.630000000000000000e+02,
		5.880000000000000000e+02,
		6.130000000000000000e+02,
		6.370000000000000000e+02,
		6.620000000000000000e+02,
		6.860000000000000000e+02,
		7.090000000000000000e+02,
		7.330000000000000000e+02,
		7.550000000000000000e+02,
		7.770000000000000000e+02,
		7.980000000000000000e+02,
		8.190000000000000000e+02,
		8.390000000000000000e+02,
		8.580000000000000000e+02,
		8.760000000000000000e+02,
		8.940000000000000000e+02,
		9.100000000000000000e+02,
		9.250000000000000000e+02,
		9.400000000000000000e+02,
		9.530000000000000000e+02,
		9.660000000000000000e+02,
		9.770000000000000000e+02,
		9.870000000000000000e+02,
		9.960000000000000000e+02,
		1.003000000000000000e+03,
		1.010000000000000000e+03,
		1.015000000000000000e+03,
		1.019000000000000000e+03,
		1.022000000000000000e+03,
		1.024000000000000000e+03,
		1.024000000000000000e+03,
		1.023000000000000000e+03,
		1.021000000000000000e+03,
		1.017000000000000000e+03,
		1.013000000000000000e+03,
		1.007000000000000000e+03,
		1.000000000000000000e+03,
		9.910000000000000000e+02,
		9.820000000000000000e+02,
		9.710000000000000000e+02,
		9.600000000000000000e+02,
		9.470000000000000000e+02,
		9.330000000000000000e+02,
		9.180000000000000000e+02,
		9.020000000000000000e+02,
		8.850000000000000000e+02,
		8.670000000000000000e+02,
		8.490000000000000000e+02,
		8.290000000000000000e+02,
		8.090000000000000000e+02,
		7.880000000000000000e+02,
		7.660000000000000000e+02,
		7.440000000000000000e+02,
		7.210000000000000000e+02,
		6.980000000000000000e+02,
		6.740000000000000000e+02,
		6.500000000000000000e+02,
		6.250000000000000000e+02,
		6.000000000000000000e+02,
		5.750000000000000000e+02,
		5.500000000000000000e+02,
		5.250000000000000000e+02,
		4.990000000000000000e+02,
		4.740000000000000000e+02,
		4.490000000000000000e+02,
		4.240000000000000000e+02,
		3.990000000000000000e+02,
		3.740000000000000000e+02,
		3.500000000000000000e+02,
		3.260000000000000000e+02,
		3.030000000000000000e+02,
		2.800000000000000000e+02,
		2.580000000000000000e+02,
		2.360000000000000000e+02,
		2.150000000000000000e+02,
		1.950000000000000000e+02,
		1.750000000000000000e+02,
		1.570000000000000000e+02,
		1.390000000000000000e+02,
		1.220000000000000000e+02,
		1.060000000000000000e+02,
		9.100000000000000000e+01,
		7.700000000000000000e+01,
		6.400000000000000000e+01,
		5.300000000000000000e+01,
		4.200000000000000000e+01,
		3.300000000000000000e+01,
		2.400000000000000000e+01,
		1.700000000000000000e+01,
		1.100000000000000000e+01,
		7.000000000000000000e+00,
		3.000000000000000000e+00,
		1.000000000000000000e+00,
		0.000000000000000000e+00,
		0.000000000000000000e+00,
		2.000000000000000000e+00,
		5.000000000000000000e+00,
		9.000000000000000000e+00,
		1.400000000000000000e+01,
		2.100000000000000000e+01,
		2.800000000000000000e+01,
		3.700000000000000000e+01,
		4.700000000000000000e+01,
		5.800000000000000000e+01,
		7.100000000000000000e+01,
		8.400000000000000000e+01,
		9.900000000000000000e+01,
		1.140000000000000000e+02,
		1.300000000000000000e+02,
		1.480000000000000000e+02,
		1.660000000000000000e+02,
		1.850000000000000000e+02,
		2.050000000000000000e+02,
		2.260000000000000000e+02,
		2.470000000000000000e+02,
		2.690000000000000000e+02,
		2.910000000000000000e+02,
		3.150000000000000000e+02,
		3.380000000000000000e+02,
		3.620000000000000000e+02,
		3.870000000000000000e+02,
		4.110000000000000000e+02,
		4.360000000000000000e+02,
		4.610000000000000000e+02,
		4.870000000000000000e+02,
		5.120000000000000000e+02
};

uint32_t saw_LUT[NS] = {0.000000000000000000e+00,
		8.000000000000000000e+00,
		1.600000000000000000e+01,
		2.400000000000000000e+01,
		3.200000000000000000e+01,
		4.000000000000000000e+01,
		4.800000000000000000e+01,
		5.600000000000000000e+01,
		6.400000000000000000e+01,
		7.200000000000000000e+01,
		8.100000000000000000e+01,
		8.900000000000000000e+01,
		9.700000000000000000e+01,
		1.050000000000000000e+02,
		1.130000000000000000e+02,
		1.210000000000000000e+02,
		1.290000000000000000e+02,
		1.370000000000000000e+02,
		1.450000000000000000e+02,
		1.530000000000000000e+02,
		1.610000000000000000e+02,
		1.690000000000000000e+02,
		1.770000000000000000e+02,
		1.850000000000000000e+02,
		1.930000000000000000e+02,
		2.010000000000000000e+02,
		2.090000000000000000e+02,
		2.170000000000000000e+02,
		2.260000000000000000e+02,
		2.340000000000000000e+02,
		2.420000000000000000e+02,
		2.500000000000000000e+02,
		2.580000000000000000e+02,
		2.660000000000000000e+02,
		2.740000000000000000e+02,
		2.820000000000000000e+02,
		2.900000000000000000e+02,
		2.980000000000000000e+02,
		3.060000000000000000e+02,
		3.140000000000000000e+02,
		3.220000000000000000e+02,
		3.300000000000000000e+02,
		3.380000000000000000e+02,
		3.460000000000000000e+02,
		3.540000000000000000e+02,
		3.620000000000000000e+02,
		3.710000000000000000e+02,
		3.790000000000000000e+02,
		3.870000000000000000e+02,
		3.950000000000000000e+02,
		4.030000000000000000e+02,
		4.110000000000000000e+02,
		4.190000000000000000e+02,
		4.270000000000000000e+02,
		4.350000000000000000e+02,
		4.430000000000000000e+02,
		4.510000000000000000e+02,
		4.590000000000000000e+02,
		4.670000000000000000e+02,
		4.750000000000000000e+02,
		4.830000000000000000e+02,
		4.910000000000000000e+02,
		4.990000000000000000e+02,
		5.070000000000000000e+02,
		5.160000000000000000e+02,
		5.240000000000000000e+02,
		5.320000000000000000e+02,
		5.400000000000000000e+02,
		5.480000000000000000e+02,
		5.560000000000000000e+02,
		5.640000000000000000e+02,
		5.720000000000000000e+02,
		5.800000000000000000e+02,
		5.880000000000000000e+02,
		5.960000000000000000e+02,
		6.040000000000000000e+02,
		6.120000000000000000e+02,
		6.200000000000000000e+02,
		6.280000000000000000e+02,
		6.360000000000000000e+02,
		6.440000000000000000e+02,
		6.520000000000000000e+02,
		6.610000000000000000e+02,
		6.690000000000000000e+02,
		6.770000000000000000e+02,
		6.850000000000000000e+02,
		6.930000000000000000e+02,
		7.010000000000000000e+02,
		7.090000000000000000e+02,
		7.170000000000000000e+02,
		7.250000000000000000e+02,
		7.330000000000000000e+02,
		7.410000000000000000e+02,
		7.490000000000000000e+02,
		7.570000000000000000e+02,
		7.650000000000000000e+02,
		7.730000000000000000e+02,
		7.810000000000000000e+02,
		7.890000000000000000e+02,
		7.970000000000000000e+02,
		8.060000000000000000e+02,
		8.140000000000000000e+02,
		8.220000000000000000e+02,
		8.300000000000000000e+02,
		8.380000000000000000e+02,
		8.460000000000000000e+02,
		8.540000000000000000e+02,
		8.620000000000000000e+02,
		8.700000000000000000e+02,
		8.780000000000000000e+02,
		8.860000000000000000e+02,
		8.940000000000000000e+02,
		9.020000000000000000e+02,
		9.100000000000000000e+02,
		9.180000000000000000e+02,
		9.260000000000000000e+02,
		9.340000000000000000e+02,
		9.420000000000000000e+02,
		9.510000000000000000e+02,
		9.590000000000000000e+02,
		9.670000000000000000e+02,
		9.750000000000000000e+02,
		9.830000000000000000e+02,
		9.910000000000000000e+02,
		9.990000000000000000e+02,
		1.007000000000000000e+03,
		1.015000000000000000e+03,
		1.023000000000000000e+03
};

uint32_t triangle_LUT[NS] = {0.000000000000000000e+00,
		1.625396825396825307e+01,
		3.250793650793650613e+01,
		4.876190476190475920e+01,
		6.501587301587301226e+01,
		8.126984126984126533e+01,
		9.752380952380951840e+01,
		1.137777777777777715e+02,
		1.300317460317460245e+02,
		1.462857142857142776e+02,
		1.625396825396825307e+02,
		1.787936507936507837e+02,
		1.950476190476190368e+02,
		2.113015873015872899e+02,
		2.275555555555555429e+02,
		2.438095238095237960e+02,
		2.600634920634920491e+02,
		2.763174603174603021e+02,
		2.925714285714285552e+02,
		3.088253968253968083e+02,
		3.250793650793650613e+02,
		3.413333333333333144e+02,
		3.575873015873015675e+02,
		3.738412698412698205e+02,
		3.900952380952380736e+02,
		4.063492063492063266e+02,
		4.226031746031745797e+02,
		4.388571428571428328e+02,
		4.551111111111110858e+02,
		4.713650793650793389e+02,
		4.876190476190475920e+02,
		5.038730158730158450e+02,
		5.201269841269840981e+02,
		5.363809523809522943e+02,
		5.526349206349206042e+02,
		5.688888888888889142e+02,
		5.851428571428571104e+02,
		6.013968253968253066e+02,
		6.176507936507936165e+02,
		6.339047619047619264e+02,
		6.501587301587301226e+02,
		6.664126984126983189e+02,
		6.826666666666666288e+02,
		6.989206349206349387e+02,
		7.151746031746031349e+02,
		7.314285714285713311e+02,
		7.476825396825396410e+02,
		7.639365079365079509e+02,
		7.801904761904761472e+02,
		7.964444444444443434e+02,
		8.126984126984126533e+02,
		8.289523809523809632e+02,
		8.452063492063491594e+02,
		8.614603174603173557e+02,
		8.777142857142856656e+02,
		8.939682539682539755e+02,
		9.102222222222221717e+02,
		9.264761904761903679e+02,
		9.427301587301586778e+02,
		9.589841269841269877e+02,
		9.752380952380951840e+02,
		9.914920634920633802e+02,
		1.007746031746031690e+03,
		1.024000000000000000e+03,
		1.024000000000000000e+03,
		1.007746031746031804e+03,
		9.914920634920634939e+02,
		9.752380952380951840e+02,
		9.589841269841269877e+02,
		9.427301587301587915e+02,
		9.264761904761904816e+02,
		9.102222222222221717e+02,
		8.939682539682539755e+02,
		8.777142857142857792e+02,
		8.614603174603174693e+02,
		8.452063492063491594e+02,
		8.289523809523809632e+02,
		8.126984126984127670e+02,
		7.964444444444444571e+02,
		7.801904761904761472e+02,
		7.639365079365079509e+02,
		7.476825396825397547e+02,
		7.314285714285714448e+02,
		7.151746031746031349e+02,
		6.989206349206349387e+02,
		6.826666666666667425e+02,
		6.664126984126984325e+02,
		6.501587301587301226e+02,
		6.339047619047619264e+02,
		6.176507936507937302e+02,
		6.013968253968254203e+02,
		5.851428571428571104e+02,
		5.688888888888889142e+02,
		5.526349206349207179e+02,
		5.363809523809524080e+02,
		5.201269841269840981e+02,
		5.038730158730159019e+02,
		4.876190476190477057e+02,
		4.713650793650793958e+02,
		4.551111111111110858e+02,
		4.388571428571428896e+02,
		4.226031746031746934e+02,
		4.063492063492063835e+02,
		3.900952380952380736e+02,
		3.738412698412698774e+02,
		3.575873015873016811e+02,
		3.413333333333333712e+02,
		3.250793650793650613e+02,
		3.088253968253968651e+02,
		2.925714285714286689e+02,
		2.763174603174603590e+02,
		2.600634920634920491e+02,
		2.438095238095238528e+02,
		2.275555555555556566e+02,
		2.113015873015873467e+02,
		1.950476190476190368e+02,
		1.787936507936508406e+02,
		1.625396825396826443e+02,
		1.462857142857143344e+02,
		1.300317460317460245e+02,
		1.137777777777778283e+02,
		9.752380952380963208e+01,
		8.126984126984132217e+01,
		6.501587301587301226e+01,
		4.876190476190481604e+01,
		3.250793650793661982e+01,
		1.625396825396830991e+01,
		0.000000000000000000e+00
};

// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = (uint32_t)(TIM2CLK / (F_SIGNAL)); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle
char current_waveform = 1;
static uint32_t last_button_press = 0;  // Variable to track the last button press time

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);


  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, (uint32_t)&TIM3->CCR3, NS);


  // TODO: Write current waveform to LCD ("Sine")
  delay(3000);
  lcd_command(CLEAR);
  lcd_putstring("Sine");

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


  /* USER CODE END 2 */

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Debounce using HAL_GetTick()

	uint32_t current_time = HAL_GetTick();  // Get the current time

	if (current_time - last_button_press < DEBOUNCE_DELAY) {
		// Ignore button press (debouncing)
		return;
	}


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes
	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort_IT(&hdma_tim2_ch1);

	// TODO: Change to the next waveform LUT
	switch (current_waveform) {
		case 1:
			current_waveform = 2;
			// Set the new source address to the sawtooth wave LUT
			HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)saw_LUT, (uint32_t)&TIM3->CCR3, NS);
			lcd_command(CLEAR);
			lcd_putstring("Sawtooth");
			break;

		case 2:
			current_waveform = 3;
			// Set the new source address to the triangular wave LUT
			HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)triangle_LUT, (uint32_t)&TIM3->CCR3, NS);
			lcd_command(CLEAR);
			lcd_putstring("Triangular");
			break;

		case 3:
			current_waveform = 1;
			// Set the new source address to the sine wave LUT
			HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, (uint32_t)&TIM3->CCR3, NS);
			lcd_command(CLEAR);
			lcd_putstring("Sine");
			break;

		default:
			break;
	}

	// Re-enable DMA transfer
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


	// Update the last button press time
	last_button_press = current_time;



	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
