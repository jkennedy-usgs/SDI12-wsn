//*****************************************************************************
//	Wireless module for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//	This module abstracts Xbee-specific functions from the main.c file. The
//	 intent is that if WSN protocols other than the DigiMesh API are used, this
//	 module and the xbee_API module can be substituted as appropriate (for
//	 example, wireless_ZigBee.c).
//*****************************************************************************

#include <util/delay.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include "wireless_xbee.h"
#include "main.h"
#include "RingBuff.h"
#include "nodes.h"
#include "xbee_API.h"
#include "dogm.h"

/*
 * Error handling
 * API_status bit explanations
 *   0x00 -> All OK
 *   0x02 -> Frame ID error
 *   0x03 -> CRC Error
 *   0x04 -> Bad call
 *   0x05 -> Start frame error
 *   0x06 -> UART Timeout
 *   0xFF -> Unknown error
 *
 */

//We have to keep track of the probe state, because the response from the
// remote node to the power on/power off command is identical.
bool probes_on = false;

uint8_t frameID;

//Convert hardware DIP switch on remote node to SDI-12 address
static uint8_t DIP_to_ID( uint8_t DIP_setting )
{
	uint8_t one, two, four, eight, new_id;

	// These values are based on the node carrier board layout; bit is set when input is low
	one    = ((~(DIP_setting & 0x02)) & 0x02 )>>1;
	two    = ((~(DIP_setting & 0x10)) & 0x10 )>>3;
	four   = ((~(DIP_setting & 0x80)) & 0x80 )>>5;
	eight  = ((~(DIP_setting & 0x40)) & 0x40 )>>3;
	new_id = (one|two|four|eight);
	return new_id;
}

void wireless_init_sleep()
{
	xbee_set_sleep_time( SETUP_SLEEP_TIME );
	xbee_set_wake_time( SETUP_WAKE_TIME );

	// Because node sampling is initiated by receipt of a "network woke up" message, turn these messages off during setup
	xbee_set_sleep_coord( NO_SLEEP_MESSAGES );
	xbee_start_sleep_coord();
}

void wireless_sample_battery(uint8_t node_number)
{
	char lcd_string[10];

	uint16_t battery = xbee_sample_batt(nodes[node_number].SL,nodes[node_number].SH);
	uint8_t volts_1 = battery/1000;

	// print battery voltage to screen
	itoa(volts_1, lcd_string, 10);
	dogm_puts(lcd_string);
	dogm_puts(".");
	uint16_t volts_tenths = battery%1000;
	itoa(volts_tenths, lcd_string, 10);
	dogm_puts(lcd_string);
	dogm_puts("V");
}

void wireless_turn_on_probes(uint8_t node_number)
{
	probes_on = true;
	frameID = xbee_set_DIO(nodes[node_number].SL,nodes[node_number].SH, PROBE1_PIN, PIN_HIGH, NO_ACK); //This frameID is invalid - there will be no ack - but have to get some return from function
	frameID = xbee_set_DIO(nodes[node_number].SL,nodes[node_number].SH, PROBE2_PIN, PIN_HIGH, ACK);
}

void wireless_turn_off_probes(uint8_t node_number)
{
	probes_on = false;
	frameID = xbee_set_DIO(nodes[node_number].SL,nodes[node_number].SH, PROBE1_PIN, PIN_LOW, NO_ACK);
	frameID = xbee_set_DIO(nodes[node_number].SL,nodes[node_number].SH, PROBE2_PIN, PIN_LOW, ACK);
}

void wireless_initialize_IO(uint32_t SL, uint32_t SH)
{
	xbee_set_DIO(SL, SH, PROBE_1_INPUT_PIN, ANALOG_INPUT, ACK);
	xbee_set_DIO(SL, SH, PROBE_2_INPUT_PIN, ANALOG_INPUT, ACK);
	xbee_set_DIO(SL, SH, DIP_PIN1, DIGITAL_INPUT, ACK);
	xbee_set_DIO(SL, SH, DIP_PIN2, DIGITAL_INPUT, ACK);
	xbee_set_DIO(SL, SH, DIP_PIN4, DIGITAL_INPUT, ACK);
	xbee_set_DIO(SL, SH, DIP_PIN8, DIGITAL_INPUT, ACK);
	xbee_set_pullups(SL, SH, PULLUP_BITS);
	// add command to write these settings to non-volatile memory
}

void wireless_sample_DIO(uint32_t SL, uint32_t SH)
{
	frameID = xbee_sample_DIO( SL, SH );
}

void wireless_start_sleep()
{
	xbee_start_sleep_coord();
	xbee_set_sleep_time( SLEEP_TIME );
	xbee_set_wake_time( WAKE_TIME );
	xbee_set_sleep_coord( SEND_SLEEP_MESSAGES );
}

void wireless_start_network_sleep(uint32_t SL, uint32_t SH)
{
	xbee_start_network_sleep( SL, SH );
}

void wireless_node_discover()
{
	xbee_node_discover();
}

uint8_t wireless_parse_message( bool init_state )  {

	uint8_t network_status, frameID, res, delimiter, len, frame_type, add, tmp, return_state, DIO;
	uint16_t cmd, ADC1, ADC2;
	uint32_t add_H, add_L;
	char lcd_string[5];

	delimiter = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
	len = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
	len = BUFF_GetBuffByte(BUFF_REMOVE_DATA);			// overwrite first length byte, it will be zero
	frame_type = BUFF_GetBuffByte(BUFF_REMOVE_DATA);

	switch ( frame_type )  {

		// This is a packet from the local Xbee.
		// 11/10/2010: Only time it's a valid response is during node discovery
		case AT_COMMAND_RESPONSE:

			frameID = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
			cmd	 = ( BUFF_GetBuffByte(BUFF_REMOVE_DATA) << 8 )
				 | ( BUFF_GetBuffByte(BUFF_REMOVE_DATA));

			// packets received in response to node discovery
			if ( cmd == ND_RESPONSE && BUFF_GetBuffByte(BUFF_REMOVE_DATA) == 0x00)  {

				// remove reserved bytes from buffer
				res = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
				res = BUFF_GetBuffByte(BUFF_REMOVE_DATA);

				add_H  = ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) << 24 );
				add_H |= ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) << 16 );
				add_H |= ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) << 8  );
				add_H |= ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) );
				add_L  = ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) << 24 );
				add_L |= ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) << 16 );
				add_L |= ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) << 8  );
				add_L |= ( (uint32_t)(BUFF_GetBuffByte(BUFF_REMOVE_DATA)) );

				temp_nodes[number_of_nd_nodes].SH = add_H;
				temp_nodes[number_of_nd_nodes].SL = add_L;
				number_of_nd_nodes++;
				dogm_putc(number_of_nd_nodes+48);
				return_state = kWSN_StatNodeDiscovery;
			}
			else		// other local packets?
				return_state = kWSN_StatDoneSampling;
		break;

		//These occur during intialization, when a DIO sample is received.
		case REMOTE_AT_COMMAND_RESPONSE:

			frameID = BUFF_GetBuffByte(BUFF_REMOVE_DATA);

			// Next bytes are the address of the originating node.
			for ( add = 0; add < 8; add++ )  {
				tmp = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
			}

			res = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
			res = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
			cmd	 = ( BUFF_GetBuffByte(BUFF_REMOVE_DATA) << 8 )
				 | ( BUFF_GetBuffByte(BUFF_REMOVE_DATA));

			if ( BUFF_GetBuffByte(BUFF_REMOVE_DATA) == SUCCESSFUL_CMD )  {

				switch ( cmd )  {

					//Possible DIO samples:
					//  -if unitialized, it's DIP setting, otherwise it's
					//  -sensor data
					case DIO_sample:

						// clear next five bytes
						for ( add = 0; add < 5; add++ )  {
							tmp = BUFF_GetBuffByte(BUFF_REMOVE_DATA);
						}

						DIO 	=  BUFF_GetBuffByte(BUFF_REMOVE_DATA);
						ADC1 	= (BUFF_GetBuffByte(BUFF_REMOVE_DATA) << 8) + BUFF_GetBuffByte(BUFF_REMOVE_DATA);
						ADC2	= (BUFF_GetBuffByte(BUFF_REMOVE_DATA) << 8) + BUFF_GetBuffByte(BUFF_REMOVE_DATA);

						uint8_t ID = DIP_to_ID(DIO);

						if ( !init_state )  {		//message is a response with DIP settings
							nodes[ID].DIP_setting = ID;
							node_ids[number_of_nodes] = ID;

							// print to LCD
							dogm_gotoxy(10,1);
							itoa(ID,lcd_string,10);
							dogm_puts(lcd_string);
							_delay_ms(500);

							// take addresses from temporary array and put in nodes array. Array index is the SDI-12 address, set by DIP switch
							nodes[ID].SL = temp_nodes[number_of_nodes].SL;
							nodes[ID].SH = temp_nodes[number_of_nodes].SH;
							return_state = UNINITIALIZED;
							init_status = ADDR_INITIALIZED;
						}

						else {						//message has sensor data
							ADC_sample.ADC1 = ADC1;
							ADC_sample.ADC2 = ADC2;
							ADC_sample.node = ID;
							return_state = kWSN_StatSampleReady;
						}
					break;

					case WIRELESS_SLEEP_STARTED:
						return_state = UNINITIALIZED;
						init_status = IO_UNINITIALIZED;
						number_of_nodes++;
					break;

					case PROBE1_ONorOFF:
						return_state = kWSN_StatWaitingForMessage;
					break;

					case PROBE2_ONorOFF:				// same incoming packet for both, have to keep track of which command was sent
						if ( probes_on ) 				// if the last command was to turn on the probes on, sample them
							return_state = kWSN_StatProbesOn;
						else  							// last command turned probes off
							return_state = kWSN_StatProbesOff;
					break;

					case PULLUPS_SET:
						return_state = UNINITIALIZED;
						init_status = ADDR_UNINITIALIZED;
					break;

					default:
						return_state = kWSN_StatPacketError;
				}
			}
			else {										//bad response
				//log error
				return_state = kWSN_StatPacketError;
			}
			if ( !init_state )  {
				return_state = UNINITIALIZED;
			}
		break;

		//Occur when network wakes up or sleeps
		case MODEM_STATUS:

			network_status = BUFF_GetBuffByte(BUFF_REMOVE_DATA);

			if ( network_status == NETWORK_WOKE_UP )  {
					return_state = kWSN_StatBeforeSampling;
			}
			else if ( network_status == NETWORK_ASLEEP )  {
				return_state = kWSN_StatAsleep;
			}
			else
				return_state = kWSN_StatPacketError;

			if ( !init_state )  {
				return_state = UNINITIALIZED;
			}

		break;

		default:
			return_state = kWSN_StatPacketError;
	}

	return return_state;
}
