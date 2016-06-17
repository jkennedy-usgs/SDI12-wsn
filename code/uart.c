 /* UART library for XBee-SDI12 bridge
 
  Originally based on:
 */
 
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

#include <avr/io.h>
#include <string.h>
#include <stdbool.h>
#include "uart.h"

void uart_init()
{
  unsigned int ubrr = 103; // UBRR = 103 -> CPU_clk = 16 MHz, Baudrate 9600

	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); 
/* Set frame format: 8data, 2stop bit */
    UCSR1C = (0<<USBS0)|(3<<UCSZ00);
}

/*
 *************************************
 *  USART1                           *
 *************************************
 */

void UART1_Transmit(uint8_t data )
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (1<<UDRE1)) )
    ;
  /* Put data into buffer, sends the data */
  UDR1 = data;
}

void UART1_Transmit_16bit(uint16_t data)
{
/* Wait for empty transmit buffer */
while ( !( UCSR1A & (1<<UDRE1)) )
;
/* Put MSB of data into buffer, sends the data */
UDR1 = (char)((data & 0xFF00)>>8);

/* Wait for empty transmit buffer */
while ( !( UCSR1A & (1<<UDRE1)) )
;

/* Put LSB of data into buffer, sends the data */

UDR1 = (data & 0x00FF);
}

void UART1_Transmit_32bit(uint32_t data)
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (1<<UDRE1)) )
    ;
  /* Put MSB_high of data into buffer, sends the data */
  UDR1 = (uint8_t)((data & 0xFF000000)>>24);

  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (1<<UDRE1)) )
    ;
  /* Put MSB_low of data into buffer, sends the data */
  UDR1 = (uint8_t)((data & 0x00FF0000)>>16);

  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (1<<UDRE1)) )
    ;
  /* Put LSB_high of data into buffer, sends the data */
  UDR1 = (uint8_t)((data & 0x0000FF00)>>8);

  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (1<<UDRE1)) )
    ;
  
  /* Put LSB_low of data into buffer, sends the data */
  UDR1 = (uint8_t)(data & 0x000000FF);
}

uint8_t UART1_Receive( void )
{
	 /* Wait for data to be received */

	TIMSK0 = (1<<TOIE0);

	while ( !(UCSR1A & (1<<RXC1)) )  {

		
	}
	return UDR1;
	TIMSK0 &= ~(1<<TOIE0);
	
}

void UART1_Transmit_string(char *string)
{
  	if(string == NULL)
    return;

  	uint8_t i = 0;
  	while(string[i] != '\0')
  	{
    	UART1_Transmit(string[i]);
    	i++;
  	}
}

