//*****************************************************************************
//	XBee API module for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//	This module issues and receives commands from remote nodes, and from the
//	 local node on the bridge unit. Functions are called from wireless_xbee.c.
//	 Most of this library is not specific to the SDI-12 bridge project and
//	 should be adaptable to other projects utlizing the XBee API.
//*****************************************************************************

#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <stdio.h>
#include "xbee_API.h"
#include "uart.h"
#include "RingBuff.h"
#include "nodes.h"
#include "main.h"

uint8_t API_status = 0x00;

/*
 * Struct used to store information between API function calls
 */

typedef struct
{
  uint8_t Frame_ID;
  char AT_cmd[2];
  char AT_cmd_value[4];
} _API;

volatile _API API_pkt;

/*
 * Initialization functions
 */

void xbee_node_discover()
{
	API_pkt.AT_cmd[0] = 'N';
  	API_pkt.AT_cmd[1] = 'D';
	local_AT_command_request(4);
}

void xbee_set_pullups(uint32_t SL, uint32_t SH, uint16_t pullups)
{
  	API_pkt.AT_cmd[0] = 'P';
  	API_pkt.AT_cmd[1] = 'R';
  	uint8_t r1 = pullups >> 8;
  	uint8_t r2 = (uint8_t)pullups;
  	API_pkt.AT_cmd_value[0] = r1;
  	API_pkt.AT_cmd_value[1] = r2;
	remote_AT_command_request( SL, SH, 0x11, ACK );
}

void xbee_start_network_sleep(uint32_t SL, uint32_t SH)
{
	API_pkt.AT_cmd[0] = 'S';
	API_pkt.AT_cmd[1] = 'M';
	API_pkt.AT_cmd_value[0] = 8;
	remote_AT_command_request( SL, SH, 0x10, ACK);
}

void xbee_start_sleep_coord()
{
	API_pkt.AT_cmd[0] = 'S';
	API_pkt.AT_cmd[1] = 'M';
	API_pkt.AT_cmd_value[0] = 7;
	local_AT_command_request(5);
}

void xbee_set_sleep_coord(bool send_status_messages)
{
	API_pkt.AT_cmd[0] = 'S';
	API_pkt.AT_cmd[1] = 'O';
	if ( send_status_messages )
		API_pkt.AT_cmd_value[0] = 5;
	else
		API_pkt.AT_cmd_value[0] = 1;

	local_AT_command_request(5);
}

void xbee_set_sleep_time(uint16_t sleep_time)
{
	API_pkt.AT_cmd[0] = 'S';
	API_pkt.AT_cmd[1] = 'P';
	API_pkt.AT_cmd_value[0] = sleep_time >> 8;
  	API_pkt.AT_cmd_value[1] = (uint8_t)sleep_time;
	local_AT_command_request(6);
}

void xbee_set_wake_time(uint16_t wake_time)
{
	API_pkt.AT_cmd[0] = 'S';
	API_pkt.AT_cmd[1] = 'T';
	API_pkt.AT_cmd_value[0] = wake_time >> 8;
  	API_pkt.AT_cmd_value[1] = (uint8_t)wake_time;
	local_AT_command_request(6);
}

/*
 * ADC sampling functions
 */

uint8_t xbee_set_DIO(uint32_t SL, uint32_t SH, uint8_t pin, uint8_t pin_state, bool ack)
{
  	API_pkt.AT_cmd[0] = 'D';
  	API_pkt.AT_cmd[1] = pin;
  	API_pkt.AT_cmd_value[0] = pin_state;
  	API_pkt.AT_cmd_value[1] = 0x00;
	remote_AT_command_request(SL,SH,0x10, ack );
	return API_pkt.Frame_ID;
}

uint8_t xbee_sample_DIO(uint32_t SL, uint32_t SH)
{
  	API_pkt.AT_cmd[0] = 'I';
  	API_pkt.AT_cmd[1] = 'S';
  	remote_AT_command_request( SL, SH, 0x0F, ACK);
  	return API_pkt.Frame_ID;
}

uint16_t xbee_sample_batt(uint32_t SL, uint32_t SH)		//Needs work!
{
	API_pkt.AT_cmd[0] = '%';
  	API_pkt.AT_cmd[1] = 'V';
	uint16_t battery = 0;
	remote_AT_command_request( SL, SH, 0x0F, ACK) ;
		//battery = (API_pkt.AT_response[0] << 8) + API_pkt.AT_response[1];
	//else
		//handle_API_error();
	return 0; //battery;
}

/*
 * Private functions
 */

static uint16_t sum_of_bytes(uint32_t data)
{
	uint16_t sum;

	sum =  (uint8_t)((data & 0xFF000000)>>24);
	sum += (uint8_t)((data & 0x00FF0000)>>16);
 	sum += (uint8_t)((data & 0x0000FF00)>>8);
 	sum += (uint8_t)(data & 0x000000FF);

  	return sum;
}

static void increment_pkt()
{
	API_pkt.Frame_ID++;
	if(API_pkt.Frame_ID == 0)                   // Frame_ID = 0 means no response, don't want to send that
    	API_pkt.Frame_ID = 1;
}

/*
 * UART transmit functions
 */

static void remote_AT_command_request(uint32_t SL, uint32_t SH, uint8_t packet_length, bool ack)
{
  	uint8_t i, checksum, pkt_identifier, pkt_ID;

  	if ( ack ) {
		increment_pkt();
		pkt_ID = API_pkt.Frame_ID;
	}
	else
		pkt_ID = 0;									// No response expected

	pkt_identifier = 0x17;
	UART1_Transmit(API_start_delimiter);    		// Start delimiter
	UART1_Transmit_16bit(packet_length);    		// Varies based on command
	UART1_Transmit(pkt_identifier);         		// API Identifier-> TX Request with 64 bits serial number
	UART1_Transmit(pkt_ID);				           	// Frame ID (for ACK)
	UART1_Transmit_32bit(SH);               		// Serial Number High
	UART1_Transmit_32bit(SL);               		// Serial Number Low
	UART1_Transmit(0xFF);					  		// Destination network address (broadcast)
	UART1_Transmit(0xFE);					  		// Destination network address (broadcast)
	UART1_Transmit(0x02);					  		// Set AT command (ignored if query)
	UART1_Transmit(API_pkt.AT_cmd[0]);          	// First char of AT command
	UART1_Transmit(API_pkt.AT_cmd[1]);          	// Second char of AT command

	checksum = (0xFF - (uint8_t)(pkt_identifier + pkt_ID + sum_of_bytes(SH) + sum_of_bytes(SL)
  					+ 0xFE + 0xFF + 0x02 + API_pkt.AT_cmd[0] + API_pkt.AT_cmd[1] + API_pkt.AT_cmd_value[0]
					+ API_pkt.AT_cmd_value[1]) ) ;

	// If packet_length is greater than 15, command parameters are present
  	for ( i = 0; i < (packet_length - 15); i++ )
		UART1_Transmit(API_pkt.AT_cmd_value[i]);

	// Clear AT_cmd_value
  	for (i = 0; i < 4; i++)
  		API_pkt.AT_cmd_value[i] = 0;

	UART1_Transmit(checksum);           // Sends CRC to module
}

static void local_AT_command_request(uint8_t packet_length)
{
	increment_pkt();
	uint8_t i, checksum, pkt_identifier;
  	pkt_identifier = 0x08;               		// 0x08->AT Command

	UART1_Transmit(API_start_delimiter); 		// Start delimiter
	UART1_Transmit_16bit(packet_length);   		// Length
	UART1_Transmit(pkt_identifier);      		// API Identifier->AT Command
	UART1_Transmit(API_pkt.Frame_ID);        	// Frame ID (for ack)
	UART1_Transmit(API_pkt.AT_cmd[0]);       	// First char of AT command
	UART1_Transmit(API_pkt.AT_cmd[1]);       	// Second char of AT command

	for ( i = 0; i < (packet_length - 4); i++ )
  		UART1_Transmit(API_pkt.AT_cmd_value[i]);

	checksum = (0xFF - (uint8_t)(	pkt_identifier + API_pkt.Frame_ID +
  									API_pkt.AT_cmd[0] + API_pkt.AT_cmd[1] +
									API_pkt.AT_cmd_value[0] + API_pkt.AT_cmd_value[1]) ) ;
	for (i = 0; i < 4; i++)
  		API_pkt.AT_cmd_value[i] = 0;

	UART1_Transmit(checksum);           // Sends CRC to module
}


