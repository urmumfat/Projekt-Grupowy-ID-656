/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body z obsługa przerwań UART
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h> // Potrzebne do memcpy
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)  // Wyłączenie wyrównywania bajtów, aby struktura pasowała do ramki
typedef struct {
    uint8_t header;    // 0xAA
    float v_lin;       // Prędkość liniowa z ROS
    float v_ang;       // Prędkość kątowa z ROS
    uint8_t footer;    // 0x55
} TelemetryFrame_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Zmienne do obsługi komunikacji w przerwaniach
uint8_t rx_byte;                    // Pojedynczy bajt odbierany w danej chwili
uint8_t rx_buffer[10];              // Bufor roboczy na ramkę (1+4+4+1 bajtów)
uint8_t rx_index = 0;               // Licznik odebranych bajtów
volatile uint8_t new_data_flag = 0; // Flaga informująca o poprawnej ramce

TelemetryFrame_t rxFrame;           // Struktura z danymi docelowymi
char msgBuffer[100];                // Bufor diagnostyczny dla Putty
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Przekierowanie printf do UART2 (diagnostyka)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init(); // Port diagnostyczny (ST-Link)
  MX_USART3_UART_Init(); // Port do komunikacji z ROS 2

  /* USER CODE BEGIN 2 */
  printf("System operacyjny robota uruchomiony...\r\n");

  // Uruchomienie nasłuchiwania w trybie przerwań (IT)
  // Odbieramy 1 bajt i czekamy na wywołanie Callbacku
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Sprawdzamy, czy w tle odebrano nową komendę z ROSa
      if (new_data_flag == 1)
      {
          new_data_flag = 0; // Reset flagi

          // DIAGNOSTYKA: Wypisanie odebranych danych na UART2
          int len = sprintf(msgBuffer, "ROS Command -> V_lin: %.2f, V_ang: %.2f\r\n",
                            rxFrame.v_lin, rxFrame.v_ang);
          HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, len, 10);

          /* TUTAJ MIEJSCE NA TWOJĄ KINEMATYKĘ I PID:
             v_left  = rxFrame.v_lin - (rxFrame.v_ang * ROZSTAW / 2.0);
             v_right = rxFrame.v_lin + (rxFrame.v_ang * ROZSTAW / 2.0);
          */
      }

      // Miejsce na inne zadania, np. odczyt enkoderów, które nie będą blokowane przez UART
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
// Callback wywoływany przez HAL po odebraniu każdego bajtu przez USART3
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // Maszyna stanów do synchronizacji ramki
        if (rx_index == 0) // Czekamy na bajt startu
        {
            if (rx_byte == 0xAA)
            {
                rx_buffer[rx_index++] = rx_byte;
            }
        }
        else // Odbieramy resztę danych
        {
            rx_buffer[rx_index++] = rx_byte;

            // Jeśli odebraliśmy 10 bajtów (pełna ramka)
            if (rx_index == 10)
            {
                // Sprawdzenie bajtu stopu
                if (rx_buffer[9] == 0x55)
                {
                    // Kopiowanie bufora do struktury (rzutowanie pamięci)
                    memcpy(&rxFrame, rx_buffer, 10);
                    new_data_flag = 1; // Powiadom pętlę główną
                }
                rx_index = 0; // Reset indeksu i szukanie kolejnego 0xAA
            }
        }

        // Ponowne uzbrojenie przerwania na kolejny bajt
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

// Obsługa błędów UART (np. przepełnienie bufora - Overrun Error)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // Wyczyszczenie flag błędów sprzętowych
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        // Restart mechanizmu odbioru
        rx_index = 0;
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // ... (Twoja wygenerowana konfiguracja zegara zostaje bez zmian) ...
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
