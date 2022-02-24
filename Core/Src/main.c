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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include <string.h>



//light sensor
#include "bh1750_config.h"
#include "bh1750.h"

//lcd config
#include "lcd_i2c.h"
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

/* USER CODE BEGIN PV */

float error;
float voltage=0;
float check_voltage = 0;
float set_point=490;

float prev_error;
float I, Integral, prev_Integral;
float pwm_i;
float pwm_p;
float error_p;

// HUART
char text[10];
char input[4] = "";

char text1[20];
char text2[20];
char text3[20];

int length1, length2, length3;
uint8_t length;
//char buff[3];

// Light
float lightLUXint;


//PWM
float pwm_duty_f;
float pwm_duty =0;

// LCD variables;
struct lcd_disp disp;
char LCDdisplay1[17];
char LCDdisplay2[17];

int new_set;

// rotary encoder
uint32_t encoder_coutner = 0;
uint32_t temp = 0;
int16_t count;
int16_t position = 0;


//PID variables

float kp =0.01;
float ki =0.001;
float kd =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int v1,v2,v3;
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
	HAL_UART_Receive_IT(&huart3, &input, 4);
	char idx =input[0];

	if(idx=='S'){

		v1 = (int)(input[1]-'0');
		v2 = (int)(input[2]-'0');
		v3 = (int)(input[3]-'0');
		new_set = v1*100+v2*10+v3;
		set_point=new_set;

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	 if(idx == 'g'){
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		int disp_out;
		int disp_set;
		int disp_error;

		disp_out = (int)lightLUXint;
		disp_set = (int)set_point;
		disp_error=(int)error_p;

		length1 = snprintf(text1, 20, "SET:%d lux \r\n", disp_out);
		length2 = snprintf(text2, 20, "OUT:%d lux \r\n", disp_set);
		length3 = snprintf(text3, 20, "ERR:%d lux \r\n", disp_error);
		if(input[3]=='U'){
			HAL_UART_Transmit(&huart3, (uint8_t*)text1, length1, 100);
		}
		else if(input[3]=='Y'){
			HAL_UART_Transmit(&huart3, (uint8_t*)text2, length2, 100);
		}
		else if(input[3]=='E'){
			HAL_UART_Transmit(&huart3, (uint8_t*)text3, length3, 100);
		}
	}

}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		lightLUXint = BH1750_ReadLux(&hbh1750_1);
		error=set_point-lightLUXint;
		error_p=error/1000*100;
		// PID //

		// I
		//	      Integral, prev_Integral;
		I = prev_Integral+error+prev_error;
		pwm_i=I*ki;

		prev_Integral=I;
		prev_error = error;

		//P
		pwm_p=kp*error;

		pwm_duty=pwm_i+pwm_p;

		if(pwm_duty>999)
			pwm_duty=999;
		else if(pwm_duty<0)
			pwm_duty=0;


		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_duty);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	encoder_coutner=__HAL_TIM_GET_COUNTER(htim);
	count = (int16_t)count;
	position = count/4;
	set_point=encoder_coutner;
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	BH1750_Init(&hbh1750_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart3, &input, 4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);


	//encoder
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);


	// LCD configuration
	disp.addr = (0x3F << 1);
	disp.bl = true;
	lcd_init(&disp);

	int disp_out;
	int disp_set;
	int disp_error;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		disp_out = (int)lightLUXint;
		disp_set = (int)set_point;
		disp_error=(int)error_p;

		sprintf(LCDdisplay1, "OUT:%d lux", disp_out);
		sprintf(LCDdisplay2, "SET:%d lux",disp_set);

		sprintf((char *)disp.f_line, LCDdisplay1);
		sprintf((char *)disp.s_line, LCDdisplay2);

		lcd_display(&disp);

		length = snprintf(text, 20, "OUT:%d lux \r\n", disp_out);

		HAL_Delay(500);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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

