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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "iks4a1_motion_sensors.h"
#include "custom_bus.h"
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

int pwm_base = 900;   // Bazowa prędkość (0-1000)

float Kp = 25.0;      // Wzmocnienie proporcjonalne
float Ki = 0.2;       // Wzmocnienie całkujące
float calka = 0;      // Skumulowany błąd

int16_t prev_licznik1 = 0;
int16_t prev_licznik2 = 0;
int32_t prev_czas = 0;

char robot_state = '0';		// '0'-prosto, '1'-prawo, '2'-lewo, '3'-stop

volatile int16_t current_cnt1, current_cnt2;
volatile int16_t speed1, speed2;
volatile float uchyb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FUNKCJA STEROWANIA PWM
// PC6=CH1, PC7=CH2 (Silnik 1), PC8=CH3, PC9=CH4 (Silnik 2)
void pwm(int s1_m1, int s1_m2, int s2_m1, int s2_m2) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, s1_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, s1_m2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, s2_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, s2_m2);
}

// FUNKCJA RUSZANIA Z MIEJSCA
void ruszanie(){
    for (int i = 600; i <= pwm_base; i += 10) {
        pwm(i, 0, i, 0);
        HAL_Delay(20);
    }
}

// FUNKCJA OGRANICZAJĄCA WARTOŚCI DO DANEGO PRZEDZIALU
int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
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
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // USTAWIANIE OKRESU
      __HAL_TIM_SET_AUTORELOAD(&htim8, 999);

      // START PWM DLA OBU SILNIKÓW (4 kanały na TIM8)
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

      // START SPRZĘTOWEGO ZLICZANIA ENKODERÓW
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

      pwm(0,0,0,0); // silniki stoją na starcie

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (HAL_GetTick() - prev_czas >= 50) // Pomiar co 50ms
	        {
	            prev_czas = HAL_GetTick();

	            // POBIERANIE DANYCH Z ENKODERÓW
	            current_cnt1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	            current_cnt2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

	            // OBLICZENIE PRĘDKOŚCI
	            speed1 = current_cnt1 - prev_licznik1; // Rzutowanie danych na int16_t - aby poprawnie liczyć różnicę przy przepełnieniu licznika
	            speed2 = -(current_cnt2 - prev_licznik2);

	            prev_licznik1 = current_cnt1;
	            prev_licznik2 = current_cnt2;

	            // LOGIKA STEROWANIA

				if (robot_state == '0') // JAZDA PROSTO z regulacją PI
				{
					float uchyb = (float)speed1 - (float)speed2;
					calka += uchyb;

					// Anty-windup (limitowanie całki)
					if (calka > 500) calka = 500;
					if (calka < -500) calka = -500;

					int korekcja = (int)(uchyb * Kp + calka * Ki);

					int p1 = clamp(pwm_base - korekcja, 0, 1000);
					int p2 = clamp(pwm_base + korekcja, 0, 1000);

					pwm(p1, 0, p2, 0);
				}
				else if (robot_state == '1') // PRAWO
				{
					pwm(800, 0, 0, 800);
				}
				else if (robot_state == '2') // LEWO
				{
					pwm(0, 800, 800, 0);
				}
				else if (robot_state == '3') // STOP
				{
					pwm(0,0,0,0);
				}
				else if (robot_state == '4') // JAZDA DO TYLU
				{
					pwm(0, 800, 0, 800);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
