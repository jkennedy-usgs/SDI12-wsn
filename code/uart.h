 /**********************************************************************
 *  API for MaxStream Xbee ZigBee modules, version 1.0                 *
 *  Compatible with 802.15.4 firmware, version 1.0.A.3 and above.      *
 *  Developed for Atmel ATMega162 @ 3,6864 MHz.                        *
 *                                                                     *
 *  Part of thesis project "ZigBee in Industry" by Andreas Wettergren. *
 *  Available for download at                                          *
 *  http://urn.kb.se/resolve?urn=urn:nbn:se:liu:diva-10061             *
 **********************************************************************/

/* Copyright © 2007 Binar Elektronik AB
 * Code written by Andreas Wettergren - andreas(at)wettergren.se
 * This file is part of Xbee API version 1.0.
 *
 * Xbee API version 1.0 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Xbee API version 1.0 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Xbee API version 1.0. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef UART_H
#define UART_H

/*
 *************************************
 *  Includes                         *
 *************************************
 */
#include <inttypes.h>
#include <stdbool.h>


/*
 *************************************
 *  Functions                        *
 *************************************
 */

/*
 * Description: Initializes UART0 and UART1
 * Input: None
 * Output: None
 */
void uart_init();

/*
 *************************************
 *  USART1                           *
 *************************************
 */

/*
 * Description: Transmits uint8_t over UART1
 * Input: uint8_t - Data to be transmitted
 * Output: None
 */
void UART1_Transmit(uint8_t data);
uint8_t UART1_Receive();
/*
 * Description: Transmits uint16_t over UART1
 * Input: uint16_t - Data to be transmitted
 * Output: None
 */
void UART1_Transmit_16bit(uint16_t data);
void UART1_Transmit_32bit(uint32_t data);

/*
 * Description: Transmits a NULL-terminated string over UART1
 *              The string can be of arbitrary length.
 * Input: char* - A char pointer to the string
 * Output: None
 */
void UART1_Transmit_string(char *string);


/*
 * Description: Receives uint8_t from UART1
 * Input: None
 * Output: uint8_t - Received data
 */
//unsigned char UART1_Receive(void);

#endif
