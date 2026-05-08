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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("System operacyjny robota uruchomiony...\r\n");

    rx_index = 0; // Upewniamy się, że stan maszyny to 0
    // Nasłuchujemy 1 bajtu startu, zapisując bezpośrednio do bufora
    HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (new_data_flag == 1)
	        {
	            new_data_flag = 0; // Reset flagi

	            // Wypisanie 10 surowych bajtów w formacie HEX
	            int len = sprintf(msgBuffer, "Odebrano 10 bajtow HEX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
	                              rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3],
	                              rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7],
	                              rx_buffer[8], rx_buffer[9]);

	            HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, len, 100);
	        }          /* TUTAJ MIEJSCE NA TWOJĄ KINEMATYKĘ I PID:
             v_left  = rxFrame.v_lin - (rxFrame.v_ang * ROZSTAW / 2.0);
             v_right = rxFrame.v_lin + (rxFrame.v_ang * ROZSTAW / 2.0);
          */


      // Miejsce na inne zadania, np. odczyt enkoderów, które nie będą blokowane przez UART
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
// Callback wywoływany przez HAL po odebraniu każdego bajtu przez USART3
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_index == 0) // STAN 0: Odebrano 1 bajt. Sprawdzamy, czy to bajt startu.
        {
            if (rx_buffer[0] == 0xAA)
            {
                rx_index = 1; // Znaleziono start. Zmieniamy stan maszyny.

                // GENIALNY TRIK: Zlecamy pobranie kolejnych 9 bajtów od razu!
                // Przerwanie wywoła się ponownie dopiero, gdy wpadnie cała reszta ramki.
                HAL_UART_Receive_IT(&huart3, &rx_buffer[1], 9);
            }
            else
            {
                // To nie jest 0xAA (np. jakieś śmieci). Szukamy dalej 1 bajtu.
                HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
            }
        }
        else // STAN 1: Właśnie odebrano paczkę 9 bajtów (czyli mamy już komplet 10)
        {
            // Sprawdzamy bajt stopu (jest na indeksie 9, bo bufor ma rozmiar 0-9)
            if (rx_buffer[9] == 0x55)
            {
                // Ramka w 100% poprawna, kopiujemy do struktury floatów
                memcpy(&rxFrame, rx_buffer, 10);
                new_data_flag = 1; // Powiadomienie dla pętli głównej
            }

            // Koniec cyklu ramki. Resetujemy maszynę i szukamy nowego bajtu startu.
            rx_index = 0;
            HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
        }
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // Wyczyszczenie wszystkich flag błędów (w tym Overrun)
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        // W razie błędu awaryjnie wracamy do poszukiwania bajtu startu
        rx_index = 0;
        HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
    }
}/* USER CODE END 4 */
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
