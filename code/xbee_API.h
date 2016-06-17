//*****************************************************************************
//	Header file for XBee API module for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//*****************************************************************************

#ifndef XBEE_API_H
#define XBEE_API_H

#include <stdbool.h>

#define API_start_delimiter 		0x7E
#define API_PACKET_INDICATOR 		0x7E
#define MODEM_STATUS 				0x8A
#define NETWORK_WOKE_UP 			0x0B
#define NETWORK_ASLEEP 				0x0C
#define REMOTE_RESPONSE 			0x97
#define SUCCESSFUL_CMD 				0x00
#define AT_COMMAND_RESPONSE 		0x88
#define REMOTE_AT_COMMAND_RESPONSE	0x97
#define ND_RESPONSE 				0x4E44
#define DIO_sample					0x4953

#define WIRELESS_SLEEP_STARTED		0x534D
#define PIN_HIGH 					0x05
#define PIN_LOW 					0x04

#define ERR_FRAME_ID				0x02
#define ERR_CHECKSUM				0x03
#define ERR_BAD_CALL				0x04
#define ERR_START_FRAME				0x05
#define ERR_UART_TIMEOUT			0x06

#define ACK							1
#define NO_ACK						0

void xbee_set_sleep_time(uint16_t sleep_time);
void xbee_set_wake_time(uint16_t wake_time);
void xbee_set_sleep_coord(bool send_status_messages);
void xbee_start_sleep_coord();
void xbee_start_network_sleep(uint32_t SL, uint32_t SH);
void xbee_node_discover();
uint16_t xbee_sample_batt(uint32_t SL, uint32_t SH);
void xbee_clear_error_flags();

/*
 * Description: Send command to remote XBee node, such as set or sample I/O, read parameter
 * Input: Address of remote node SH and SL, length of packet to send, whether a response is expected or not
 * Output: boolean with response status
 */
static void remote_AT_command_request(uint32_t SL, uint32_t SH, uint8_t packet_length, bool ack);

/*
 * Description: Set pin to proper state: ADC input, or digital high/low.
 * Input: Address of remote node SH and SL, pin to set, pin state.
 * Output: boolean with response status
 */
uint8_t xbee_set_DIO(uint32_t SL, uint32_t SH, uint8_t pin, uint8_t pin_state, bool ack);

/*
 * Description: Set pullup value. Pullups should be disabled on digital outputs and ADC channels.
 * Input: Address of remote node SH and SL, length of packet to send, 16 bit pullup setting. .
 * Output: boolean with response status
 */
void xbee_set_pullups(uint32_t SL, uint32_t SH, uint16_t pullups);

/*
 * Description: Samples all enabled digital and analog channels of remote XBee.
 * Input: Address of remote node SH and SL.
 * Output: struct with DIO, ADC1, ADC2 readings from remote XBee
 */
uint8_t xbee_sample_DIO(uint32_t SL, uint32_t SH);

/*
 * Description: Increments API_pkt.frame_ID, or resets to 1 if it has been set to zero by an error.
 * Input: none
 * Output: none
 */
static void increment_pkt();

/*
 * Description: Send command to local XBee node, the one attached to SDI-12 port.
 * Input: Length of packet to send. Varies with the command.
 * Output: none
 */
static void local_AT_command_request(uint8_t length);

#endif
