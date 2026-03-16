/*
 * uart_wiadomosc.h
 *
 *  Created on: Feb 25, 2026
 *      Author: adamj
 */

#ifndef INC_UART_WIADOMOSC_H_
#define INC_UART_WIADOMOSC_H_


#include "stm32l4xx_it.h" // Zmień na swoją serię, np. stm32l4xx_hal.h jeśli używasz innej
#include <stdio.h>
#include <string.h>

#include "main.h"

void UART_SendMotorData(UART_HandleTypeDef *huart, char side, int value);

#endif /* INC_UART_WIADOMOSC_H_ */
