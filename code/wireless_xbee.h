//*****************************************************************************
//	Header file for wireless module for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//	This module abstracts Xbee-specific functions from the main.c file. The
//	 intent is that if WSN protocols other than the DigiMesh API are used, this
//	 module and the xbee_API module can be substituted as appropriate (for
//	 example, wireless_ZigBee.c).
//*****************************************************************************

#ifndef WIRELESS_XBEE_H
#define WIRELESS_XBEE_H

//XBee-specific 
#define PROBE1_ONorOFF				0x4438
#define PROBE2_ONorOFF				0x4439
#define PULLUPS_SET					0x5052
#define API_start_delimiter 		0x7E
#define ANALOG_INPUT 				0x02
#define DIGITAL_INPUT				0x03

//State definitions for initialization routine
#define IO_UNINITIALIZED 	 		0x01
#define ADDR_UNINITIALIZED 			0x02
#define ADDR_INITIALIZED			0x03
#define INIT_WAITING 				0x04

//Pin settings specific to SDI-12 node unit PCB:
#define PROBE1_PIN					'8'
#define PROBE2_PIN 					'9'
#define PROBE_1_INPUT_PIN			'2'
#define PROBE_2_INPUT_PIN 			'3'
#define DIP_PIN1					'1'
#define DIP_PIN2					'4'
#define DIP_PIN4					'7'
#define DIP_PIN8					'6'
#define PULLUP_BITS 				0x2029 					// 2029: Pullups on DIO 1,4,7,6

void wireless_turn_on_probes(uint8_t node_number);

void wireless_turn_off_probes(uint8_t node_number);

void wireless_initialize_IO(uint32_t SL, uint32_t SH);

void wireless_sample_DIO(uint32_t SL, uint32_t SH);

void wireless_start_sleep();

uint8_t wireless_parse_message(bool initialized);

void wireless_start_network_sleep(uint32_t SL, uint32_t SH);

void wireless_node_discover();

void wireless_sample_battery(uint8_t node_number);

#endif
