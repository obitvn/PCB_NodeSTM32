/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $

 https://habr.com/en/post/279747/ conver to stm32f1 by chutienthinh.bk 0981762826
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
//static UART_HandleTypeDef huart;
extern UART_HandleTypeDef huart1;


/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	// remove pi DIR because using HC-11 RF
	if (xRxEnable) { // если переключаемся в режим приема
        // переключаем драйвер в режим приема (см. принципы работы rs485)
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // разрешаем прерывание, реагирует если буфер приема не пуст
    } else {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE); // запрещаем прерывание
    }
    if (xTxEnable) { // если переключаемся в режим передачи
        // переключаем драйвер в режим передачи (см. принципы работы rs485)
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE); // разрешаем прерывание, реагирует если буфер передачи пуст
    } else {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE); // запрещаем прерывание
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	 return (HAL_OK == HAL_UART_Transmit(&huart1, (uint8_t*)&ucByte, 1, 10));
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	 *pucByte = (uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);  
  return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */






