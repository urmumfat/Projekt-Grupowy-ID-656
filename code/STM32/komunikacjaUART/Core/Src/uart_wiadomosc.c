#include "uart_wiadomosc.h"


void UART_SendMotorData(UART_HandleTypeDef *huart, char side, int value) {
    char msg[16]; // Bufor na wiadomość

    // Formatowanie ciągu znaków: litera, liczba, nowa linia (\r\n dla czytelności w terminalu)
    // Wynik: "L123\r\n"
    int len = snprintf(msg, sizeof(msg), "%c%d\r\n", side, value);

    // Wysyłanie przez HAL z timeoutem 100ms
    if (len > 0) {
        HAL_UART_Transmit(huart, (uint8_t*)msg, len, 100);
    }
}
