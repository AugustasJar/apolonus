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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_f1.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define rad2deg  57.29567015
#define MAX_ANGLE 0.24
#define MIN_PWM 2
#define Kp 400
#define Ki 400
#define Kd 0.01
#define PWM_COUNTER 65535
#define PWM_CONV PWM_COUNTER/100
#define MOTOR_DEADZONE 0.01
#define Kp_pos 100
#define Ki_pos 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void motor(double L, double R);
double abs2(double x);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
encoder encoderA,encoderB;

int16_t encoderA_pos = 0;
double right_motor_pos = 0, left_motor_pos = 0;
double encoder_step = 0.00872664625997;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// to fix usbc_dc

//510 memset(pdev->pClassData,0,sizeof(USBD_CDC_HandleTypeDef)); // THIS LINE WAS ADDED


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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  char  buff2[128];
  uint32_t t1,t2;

  imu_t mpu;
  uint16_t pwm;

  double control = 0;
  double dt = 10;
  const double th_bias = -.025;
  double angle_error = 0;
  double prev_angle_error = 0;
  double speed = 0;
  double last_pos = 0;
  double prev_speed = 0;
  double ss = 0;
  double sign;
  double integrator = 0;
  double dth = 0;
  double U1, U2;
  double delta = 0;
  double prev_dth = 0;
  double prev_delta = 0;
  const double F = 85;
     const double F1 = 0.3*sqrt(F);
     const double F2 = F*10;
     const double A = 2.8;
     const double K_th = 15;
     const double K_dth = 0.01;
     const double K_speed = 0;
     const double K_pos = 0;
  unsigned int reached_eq = 0;

 double Ppos = 0.05;
double Ipos = 0.0001;
double Dpos = 0.000000002;
double integrator2 = 0.00000;
double set_point = 0;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  //counter period 65535


  //initializing encoder variables
  encoder_init(&encoderA,GPIOB,right_motor_A_Pin,GPIOB,right_motor_B_Pin,encoder_step);
  encoder_init(&encoderB,GPIOB,left_motor_A_Pin,GPIOB,left_motor_B_Pin,encoder_step);


  //wait for the mpu to init.
  while (imu_init(&hi2c1) == 1) {
		  HAL_Delay(10);
		  sprintf(buff2,"waiting for the imu to boot, \r\n");
  }
  imu_tune(&hi2c1,&mpu);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  imu_read(&hi2c1,&mpu);
  angle_error = mpu.th + th_bias;
  prev_angle_error = angle_error;

  motor(30,30);
  HAL_Delay(100);
  motor(0,0);

  while (1)
  {
	  last_pos = encoderA.pos;
	  t1 = HAL_GetTick();
	  HAL_Delay(1);
	  imu_read(&hi2c1,&mpu);

	  imu_complimentary(&mpu,dt);

	  dth = (angle_error - prev_angle_error)/dt;
	  speed = (encoderA.pos - last_pos)/dt;
	  if (abs2(angle_error) < 0.05) {
		  if (abs2(integrator2) < 0.015) {
			  integrator2 = integrator2 + Ipos*encoderA.pos*dt;
		  }
		  set_point = Ppos*encoderA.pos + integrator2 + Dpos*speed;
	  }


	  angle_error = mpu.th + th_bias - set_point;


	  // apply control only if the system is upright
	  if (abs2(angle_error) < 0.3 && reached_eq == 1) {
		  delta = K_th*angle_error + K_dth*dth + K_speed*speed + K_pos*encoderA.pos;
		  ss = delta + (A*( delta - prev_delta))/dt;
		  U1 = sqrt(abs2(ss))*F1;
		  sign = ss > 0 ? 1 : -1;
		  if (abs2(integrator) < 85) {
			  integrator = integrator + F2*sign*dt;
		  }
		  control =  U1*sign + integrator;
		  control = control;

		  motor(control,control);
	  }
	  else {
		  reached_eq = abs2(angle_error) < 0.01 ? 1 : 0;
		  encoderA.pos = 0;
		  integrator = 0;
		  control = 0;
		  motor(0,0);
	  }
	  prev_delta = delta;
	  prev_angle_error = angle_error;
	  prev_speed = speed;

	  t2 = HAL_GetTick();
	  dt = ((double)(t2 - t1))/1000;
	  sprintf(buff2,"%f,%f,%f,%f,%f,%f,AAB\r\n",angle_error,encoderA.pos,dth,speed,control,set_point);
	  CDC_Transmit_FS((uint8_t*) buff2, strlen(buff2));

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, left_motor_dir_Pin|right_motor_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : left_motor_dir_Pin right_motor_dir_Pin */
  GPIO_InitStruct.Pin = left_motor_dir_Pin|right_motor_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : left_motor_B_Pin right_motor_A_Pin right_motor_B_Pin left_motor_A_Pin */
  GPIO_InitStruct.Pin = left_motor_B_Pin|right_motor_A_Pin|right_motor_B_Pin|left_motor_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == right_motor_A_Pin) {
    	encoder_update(&encoderA);
    }
    if (GPIO_Pin == left_motor_A_Pin){
    	encoder_update(&encoderB);
    }

}
// takes input from -100 to 100.

void motor(double L, double R) {

	// constrain the inputs
	L = L > 100 ? 100 : L;
	R = R > 100 ? 100 : R;
	L = L < -100 ? -100 : L;
	R = R < -100 ? -100 : R;

	//change the direction pin.
	if (L < 0) {
		HAL_GPIO_WritePin(GPIOA, left_motor_dir_Pin, GPIO_PIN_SET);
		L = -1*L;
	}
	else {
		HAL_GPIO_WritePin(GPIOA, left_motor_dir_Pin, GPIO_PIN_RESET);
	}
	if (R > 0) {
		HAL_GPIO_WritePin(GPIOA, right_motor_dir_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOA, right_motor_dir_Pin, GPIO_PIN_RESET);
		R = -1*R;
	}

	//deadzone compensation
	L = L > 2 && L < MOTOR_DEADZONE  ? MOTOR_DEADZONE : L;
	R = R > 2 && R < MOTOR_DEADZONE  ? MOTOR_DEADZONE : R;

	//convert to pwm tick count
	L = L*PWM_CONV;
	R = R*PWM_CONV;

	//write to timers.
	TIM2->CCR2 = R;
	TIM2->CCR1 = L;
}

double abs2(double x) {
	x = x < 0 ? -x : x;
	return x;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	motor(0,0);
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
