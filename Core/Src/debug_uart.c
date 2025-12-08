/*
 * debug_uart.c
 *
 *  Created on: Jul 1, 2024
 *      Author: renaud
 */

// To use this function :
// - activate uart (choose 2 or 3 according to Nucleo Board)
// - include this file to your project
// - create a new COM console (on the right COM)


// This definition allow to redirect printf to uart

#include "main.h"
extern UART_HandleTypeDef huart2;


#define PUTCHAR_PROTOTYPE int __io_putchar(int c)

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART and Loop until the end of transmission */
	/* change to have the good uart ! */
	HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 0xFFFF);

	return c;
}

