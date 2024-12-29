/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979
#define dt 0.005

#define max_volt 24.0
#define min_volt 0.0

#define Kp 2.0
#define Kd 0.05

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t UART_flag = 0;

uint8_t Timer_flag = 0;
uint32_t Time = 0;
uint8_t direction = 0;
uint8_t select_mode_flag = 1;
uint8_t play_flag = 0;
uint8_t set_flag = 0;
uint8_t turn_motor_flag = 0;

uint8_t mode_ch[2];
int pulse_count = 0;
int cur_location_pulse = 0;
float input_location = 0.0;
char directional_sign = ' ';
float T_setting_range = 0.0;

float a = 0.0;
float c = 0.0;
float d = 0.0;
float e = 0.0;

float Tf = 0.0;
float T1 = 0.0;
float T2 = 0.0;

float Af = 0.0;
float A1 = 0.0;
float A2 = 0.0;

float Vmax = 0.0;
float Vm = 0.0;
float Yd = 0.0;

double Time_sample = 0.0;
double cur_position = 0.0;
double target_position = 0.0;
float end_timing = 0.0;

float error = 0.0;
float old_error = 0.0;
float u = 0.0;
int set_duty = 0;

float Tf_sample = 0.0;
float T1_sample = 0.0;
float T2_sample = 0.0;

uint16_t i = 0;
uint16_t j =0;

char num_str[5];
float actual_location_degree = 0.0;
float previous_degree = 0.0;

//double *cur_position_mem;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
    if( HAL_UART_Transmit(&huart1, ptr, len, len) == HAL_OK ) return len;
    else return 0;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		Time++;
		if((Time % 5) == 0) {
			Timer_flag = 1;
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) {
		UART_flag = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	direction = HAL_GPIO_ReadPin(read_direction_GPIO_Port, read_direction_Pin);

	pulse_count++;

	if(GPIO_Pin == encoder_a_Pin && direction == 1) {
		cur_location_pulse++;
	}
	if(GPIO_Pin == encoder_a_Pin && direction == 0) {
		cur_location_pulse--;
	}

	if(GPIO_Pin == user_key_Pin) {
		play_flag = 1;
	}
}

void select_operation_mode(void) {
	  uint8_t tx_data[] = "Select a mode: ";
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_data, sizeof(tx_data));

	  uint8_t rx_buffer[3] = {0,0,'\n'};
	  HAL_UART_Receive_IT(&huart1, &rx_buffer, 2);
	  while(!UART_flag);
	  HAL_UART_Transmit_IT(&huart1, &rx_buffer, 3);
	  HAL_Delay(1000);

	  mode_ch[0] = rx_buffer[0];
	  mode_ch[1] = rx_buffer[1];

	  UART_flag = 0;
}

void input_condition(void) {
	  uint8_t tx_data[] = "Enter the target angle: ";
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_data, sizeof(tx_data));

	  uint8_t rx_buffer[6] = {0,0,0,0,0,'\n'};
	  HAL_UART_Receive_IT(&huart1, &rx_buffer, 5);
	  while(!UART_flag);
	  HAL_UART_Transmit_IT(&huart1, &rx_buffer, 6);
	  HAL_Delay(1000);

	  directional_sign = rx_buffer[0];
	  memcpy(num_str, &rx_buffer[1], 4);
	  num_str[4] = '\0';

	  input_location = atof(num_str);

	  UART_flag = 0;
}

void input_Tf(void) {
	  uint8_t tx_data[] = "Enter a Tf: ";
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_data, sizeof(tx_data));

	  uint8_t rx_buffer[5] = {0,0,0,0,'\n'};
	  HAL_UART_Receive_IT(&huart1, &rx_buffer, 4);
	  while(!UART_flag);
	  HAL_UART_Transmit_IT(&huart1, &rx_buffer, 5);
	  HAL_Delay(1000);

	  Tf = atof(rx_buffer);
	  UART_flag = 0;
}

void input_T1(void) {
	  uint8_t tx_data[] = "Enter a T1: ";
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_data, sizeof(tx_data));

	  uint8_t rx_buffer[5] = {0,0,0,0,'\n'};
	  HAL_UART_Receive_IT(&huart1, &rx_buffer, 4);
	  while(!UART_flag);
	  HAL_UART_Transmit_IT(&huart1, &rx_buffer, 5);
	  HAL_Delay(1000);

	  T1 = atof(rx_buffer);
	  UART_flag = 0;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(play_flag == 1) {
		  if(select_mode_flag == 1) {
			  actual_location_degree = (cur_location_pulse * 3.0) / 275.0;

			  select_operation_mode();
			  set_flag = 1;
			  select_mode_flag = 0;
		  }

		  if(mode_ch[0] == 'r' && mode_ch[1] == 'e') {
			  if(set_flag == 1) {
				  input_condition();
				  Af = (input_location * 11.0 * 2.0 * PI) / 60.0;

				  Vmax = (4490.0 / 60.0) * 2.0 * PI;
				  T_setting_range = Af / Vmax;

				  printf("[Tf - T1 > %f]\r\n",T_setting_range);
				  input_Tf();
				  input_T1();
				  T2 = Tf - T1;

				  Vm = Af / (Tf - T1);

				  A1 = (Vm * T1) / 2.0;
				  A2 = ((Vm * T1) / 2.0) + ((Tf - (2.0 * T1)) * Vm);

				  a = A1 / (T1 * T1);
				  c = (A2 - A1) / (T2 - T1);
				  d = (A1 * T2 - A2 * T1) / (T2 -T1);
				  e = (A2 - Af) / ((T2 - Tf) * (T2-Tf));

				  T1_sample = T1 * 1000.0;
				  T2_sample = T2 * 1000.0;
				  Tf_sample = Tf * 1000.0;

				  end_timing = (Tf * 1000.0) / 5.0;
				  //cur_position_mem = (float*)malloc(sizeof(float) * end_timing);

				  if(Vmax > Vm) {
					  printf("Condition pass!\r\n");
					  turn_motor_flag = 1;
					  set_flag = 0;

					  HAL_TIM_Base_Start_IT(&htim2);
					  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					  htim3.Instance -> CCR3 = 0;

					  if(directional_sign == '+') {
						  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);
					  }
					  else if(directional_sign == '-') {
						  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_SET);
					  }
				  }
				  else if (Vmax < Vm) {
					  printf("Insufficient conditions!\r\n");
					  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);

		 	          select_mode_flag = 1;
		 	          play_flag = 0;
				  }
			  }
			  if(turn_motor_flag == 1 && Timer_flag == 1) {
				  Time_sample = (float)Time / 1000.0;

				  if(Time >= 0 && Time < T1_sample) {
					  Yd = a * Time_sample * Time_sample;
				  }
				  else if(Time >= T1_sample && Time < T2_sample) {
					  Yd = (c * Time_sample) + d;
				  }
				  else if(Time >= T2_sample && Time < Tf_sample) {
					  Yd = (e * (Time_sample - Tf) * (Time_sample - Tf)) + Af;
				  }

				  target_position = Yd;
				  cur_position = (pulse_count * 2.0 * PI) / 500.0;
				  //cur_position_mem[i] = cur_position;

				  error = target_position - cur_position;
				  u = Kp * error + Kd * (error - old_error);
				  old_error = error;

				  if(u > max_volt) u = max_volt;
				  if(u < min_volt) u = min_volt;

				  set_duty = (uint16_t)(1000.0 * u / 24.0);

				  htim3.Instance -> CCR3 = set_duty;

				  i++;
				  Timer_flag = 0;
			  }
			  if(turn_motor_flag == 1 && i >= end_timing) {
				  HAL_Delay(500);
				  previous_degree = actual_location_degree;
				  actual_location_degree = (cur_location_pulse * 3.0) / 275.0;
				  printf("mode: %c%c\r\nprevious: %f\r\nactual: %f\r\n",mode_ch[0], mode_ch[1], previous_degree, actual_location_degree);

				  /* ----------- Reset ----------- */
				  select_mode_flag = 1;
				  turn_motor_flag = 0;
				  play_flag = 0;
				  end_timing = 0.0;
				  mode_ch[0] = ' ';
		 		  mode_ch[1] = ' ';
		 		  i = 0;
		 		  Yd = 0.0;
		 		  Time = 0;
		 		  error = 0;
		 		  old_error = 0;
		 		  pulse_count = 0;

		 		  HAL_TIM_Base_Stop_IT(&htim2);
		 		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		 		 //free(cur_position_mem);
		 		  printf("end!\r\n\n");
			  }
		  }
		  else if(mode_ch[0] == 'a' && mode_ch[1] == 'b') {
			  if(set_flag == 1) {
				  input_condition();

				  if(directional_sign == '+') {
					  if (input_location > actual_location_degree) {
						  directional_sign = '+';
						  Af = ((input_location - actual_location_degree) * 11.0 * 2.0 * PI) / 60.0;
					  }
					  else if (input_location < actual_location_degree) {
						  directional_sign = '-';
						  Af = ((actual_location_degree - input_location) * 11.0 * 2.0 * PI) / 60.0;
					  }
				  }
				  else if(directional_sign == '-') {
					  if ((-input_location) > actual_location_degree) {
						  directional_sign = '+';
						  Af = (((-input_location) - actual_location_degree) * 11.0 * 2.0 * PI) / 60.0;
					  }
					  else if ((-input_location) < actual_location_degree) {
						  directional_sign = '-';
						  Af = ((actual_location_degree - (-input_location)) * 11.0 * 2.0 * PI) / 60.0;
					  }
				  }

				  Vmax = (4490.0 / 60.0) * 2.0 * PI;
				  T_setting_range = Af / Vmax;

				  printf("[Tf - T1 > %f]\r\n",T_setting_range);
				  input_Tf();
				  input_T1();
				  T2 = Tf - T1;

				  Vm = Af / (Tf - T1);

				  A1 = (Vm * T1) / 2.0;
				  A2 = ((Vm * T1) / 2.0) + ((Tf - (2.0 * T1)) * Vm);

				  a = A1 / (T1 * T1);
				  c = (A2 - A1) / (T2 - T1);
				  d = (A1 * T2 - A2 * T1) / (T2 -T1);
				  e = (A2 - Af) / ((T2 - Tf) * (T2-Tf));

				  T1_sample = T1 * 1000.0;
				  T2_sample = T2 * 1000.0;
				  Tf_sample = Tf * 1000.0;

				  end_timing = (Tf * 1000.0) / 5.0;
				  //cur_position_mem = (float*)malloc(sizeof(float) * end_timing);

				  if(Vmax > Vm) {
					  printf("Condition pass!\r\n");
					  turn_motor_flag = 1;
					  set_flag = 0;

					  HAL_TIM_Base_Start_IT(&htim2);
					  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					  htim3.Instance -> CCR3 = 0;

					  if(directional_sign == '+') {
						  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);
					  }
					  else if(directional_sign == '-') {
						  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_SET);
					  }
				  }
				  else if (Vmax < Vm) {
					  printf("Insufficient conditions!\r\n");
		 	          HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
		 	          HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);

		 	          select_mode_flag = 1;
		 	          play_flag = 0;
				  }
			  }
			  if(turn_motor_flag == 1 && Timer_flag == 1) {
				  Time_sample = (float)Time / 1000.0;

				  if(Time >= 0 && Time < T1_sample) {
					  Yd = a * Time_sample * Time_sample;
				  }
				  else if(Time >= T1_sample && Time < T2_sample) {
					  Yd = (c * Time_sample) + d;
				  }
				  else if(Time >= T2_sample && Time < Tf_sample) {
					  Yd = (e * (Time_sample - Tf) * (Time_sample - Tf)) + Af;
				  }

				  target_position = Yd;
				  cur_position = (pulse_count * 2.0 * PI) / 500.0;
				  //cur_position_mem[i] = cur_position;

				  error = target_position - cur_position;
				  u = Kp * error + Kd * (error - old_error);
				  old_error = error;

				  if(u > max_volt) u = max_volt;
				  if(u < min_volt) u = min_volt;

				  set_duty = (uint16_t)(1000.0 * u / 24.0);

				  htim3.Instance -> CCR3 = set_duty;

				  i++;
				  Timer_flag = 0;
			  }
			  if(turn_motor_flag == 1 && i >= end_timing) {
				  HAL_Delay(500);
				  previous_degree = actual_location_degree;
				  actual_location_degree = (cur_location_pulse * 3.0) / 275.0;
				  printf("mode: %c%c\r\nprevious: %f\r\nactual: %f\r\n",mode_ch[0], mode_ch[1], previous_degree, actual_location_degree);

				  /* ----------- Reset ----------- */
				  select_mode_flag = 1;
				  turn_motor_flag = 0;
				  play_flag = 0;
				  end_timing = 0.0;
				  mode_ch[0] = ' ';
				  mode_ch[1] = ' ';
				  i = 0;
				  Yd = 0.0;
				  Time = 0;
				  error = 0;
				  old_error = 0;
				  pulse_count= 0;

				  HAL_TIM_Base_Stop_IT(&htim2);
				  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				  //free(cur_position_mem);
				  printf("end!\r\n\n");
			  }
		  }
		  else if(mode_ch[0] == 's' && mode_ch[1] == 'p') {
			  if(set_flag == 1) {

				  if (actual_location_degree > 0) {
					  Af = (actual_location_degree * 11.0 * 2.0 * PI) / 60.0;
					  directional_sign = '-';
				  }
				  else if (actual_location_degree < 0) {
					  Af = -(actual_location_degree * 11.0 * 2.0 * PI) / 60.0;
					  directional_sign = '+';
				  }
				  T_setting_range = Af / Vmax;

				  printf("[Tf - T1 > %f]\r\n",T_setting_range);
				  input_Tf();
				  input_T1();
				  T2 = Tf - T1;

				  Vm = Af / (Tf - T1);

				  A1 = (Vm * T1) / 2.0;
				  A2 = ((Vm * T1) / 2.0) + ((Tf - (2.0 * T1)) * Vm);

				  a = A1 / (T1 * T1);
				  c = (A2 - A1) / (T2 - T1);
				  d = (A1 * T2 - A2 * T1) / (T2 -T1);
				  e = (A2 - Af) / ((T2 - Tf) * (T2-Tf));

				  T1_sample = T1 * 1000.0;
				  T2_sample = T2 * 1000.0;
				  Tf_sample = Tf * 1000.0;

				  end_timing = (Tf * 1000.0) / 5.0;
				  //cur_position_mem = (float*)malloc(sizeof(float) * end_timing);

				  if(Vmax > Vm) {
					  printf("condition pass!\r\n");
					  turn_motor_flag = 1;
					  set_flag = 0;

					  HAL_TIM_Base_Start_IT(&htim2);
					  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					  htim3.Instance -> CCR3 = 0;

					  if(directional_sign == '+') {
						  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);
					  }
					  else if(directional_sign == '-') {
						  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_SET);
					  }
				  }
				  else if (Vmax < Vm) {
					  printf("Insufficient conditions!\r\n");
					  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);

					  select_mode_flag = 1;
					  play_flag = 0;
				  }
			  }
			  if(turn_motor_flag == 1 && Timer_flag == 1) {
				  Time_sample = (float)Time / 1000.0;

				  if(Time >= 0 && Time < T1_sample) {
					  Yd = a * Time_sample * Time_sample;
				  }
				  else if(Time >= T1_sample && Time < T2_sample) {
					  Yd = (c * Time_sample) + d;
				  }
				  else if(Time >= T2_sample && Time < Tf_sample) {
					  Yd = (e * (Time_sample - Tf) * (Time_sample - Tf)) + Af;
				  }

				  target_position = Yd;
				  cur_position = (pulse_count * 2.0 * PI) / 500.0;
				  //cur_position_mem[i] = cur_position;

				  error = target_position - cur_position;
				  u = Kp * error + Kd * (error - old_error);
				  old_error = error;

				  if(u > max_volt) u = max_volt;
				  if(u < min_volt) u = min_volt;

				  set_duty = (uint16_t)(1000.0 * u / 24.0);

				  htim3.Instance -> CCR3 = set_duty;

				  i++;
				  Timer_flag = 0;
			  }
			  if(turn_motor_flag == 1 && i >= end_timing) {
				  HAL_Delay(500);
				  previous_degree = actual_location_degree;
				  actual_location_degree = (cur_location_pulse * 3.0) / 275.0;
				  printf("mode: %c%c\r\previous: %f\r\nactual: %f\r\n",mode_ch[0], mode_ch[1], previous_degree, actual_location_degree);

				  /* ----------- Reset ----------- */
				  select_mode_flag = 1;
				  turn_motor_flag = 0;
				  play_flag = 0;
				  end_timing = 0.0;
				  mode_ch[0] = ' ';
				  mode_ch[1] = ' ';
				  i = 0;
				  Yd = 0.0;
				  Time = 0;
				  error = 0;
				  old_error = 0;
				  pulse_count = 0;

				  HAL_TIM_Base_Stop_IT(&htim2);
				  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				  //free(cur_position_mem);
				  printf("end!\r\n\n");
			  }
		  }
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 18-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, in1_Pin|in2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : user_key_Pin */
  GPIO_InitStruct.Pin = user_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(user_key_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : in1_Pin in2_Pin */
  GPIO_InitStruct.Pin = in1_Pin|in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : encoder_a_Pin */
  GPIO_InitStruct.Pin = encoder_a_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(encoder_a_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : read_direction_Pin */
  GPIO_InitStruct.Pin = read_direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(read_direction_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
