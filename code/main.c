//*****************************************************************************
//	Main program routine for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//	This module initializes the wireless sensor network, and runs the WSN state
//	 machine.
//*****************************************************************************

/************************************
TODO:
 1) Xbee sleep should start when first SDI-12 command is received (?)
 2) Implement watchdog timer
*************************************/

/******************************************************************************
 Initialization process:
 1) "Node Discovery" (ND) command is sent to XBee network. Each node responds at a random
 	time with a	packet containing it's address. This address is stored in temp_nodes[]. The
	array index is sequential: first is zero, second is one, etc.
 2) For each node in temp_nodes, do these steps in order:
 	-Initialize IO on the Xbee with appropriate inputs and pullups.
	-Sample Xbee IO. This returns the SDI-12 address from the SIP switch. The SDI-12 address
	is used as the array index of nodes[].
	-Set Xbee sleep mode with specified sleep and wake times.
 3) Start Xbee sleep mode.
 4) call SDI-12 initialization function.
******************************************************************************/

/******************************************************************************
 Typical program flow after initialization:
 1) Receive "wireless network is awake" message
 2) Display LCD message while network syncnronizes
 3)	Sample each node:
 	-turn on sensors (aka probes)
 	-warmup delay
 	-sample probes
 	-qaqc sample
 	-log sample
 	-turn off probes
 4) Display "Done sampling" on LCD
 5) Do nothing while waiting for "wireless network is asleep" message

All the while, network is responsive to incoming SDI-12 data requests.
******************************************************************************/

#include <util/delay.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/wdt.h>
#include "RingBuff.h"
#include "dogm.h"
#include "sdi12.h"
#include "nodes.h"
#include "main.h"
#include "uart.h"
#include "wireless_xbee.h"

// extern vars that keep track of node information.
uint8_t number_of_nodes;
uint8_t number_of_nd_nodes;
_temp_node 	temp_nodes[NODE_ARRAY_SIZE];
_node nodes[NODE_ARRAY_SIZE];
uint8_t node_ids[NODE_ARRAY_SIZE];
_ADC_sample ADC_sample;

// Keeps track of which node is being sampled, varies from 0 to number_of_nodes-1. It's NOT the SDI-12 address.
// The SDI-12 address is node_ids[current_node].
uint8_t current_node;

// Vars for Rx ISR
volatile bool next_byte_is_len1;
volatile bool next_byte_is_len2;
volatile uint16_t xbee_incoming_length;
volatile uint8_t current_byte;
volatile uint32_t checksum;

// Vars for timer
volatile uint16_t overflows;
uint16_t overflow_counter;
uint16_t seconds;
volatile bool timer_done;

// Vars for state machine
bool initialized;
volatile uint8_t init_status = IO_UNINITIALIZED;
bool newly_asleep = true;
volatile uint8_t state = kWSN_StatNodeDiscovery;

// functions
void start_timer(uint16_t counts);
void reset_timer();
void initialize();

int main()
{
	sdi12_msg_signal = 0xff;
	char lcd_string[10];
	DDRB = (1<<DDB0);
	initialize();

	while (1)  {

	//SDI-12 routines execute when sdi12_dotask() is called, OR on these interrupts:
	//	-TX
	//	-RX
	//	-Timer 1
	//	-Pin change on SDI-12 bus.
		sdi12_dotask();
		if ( sdi12_msg_signal != 0xff ) {
			//dogm_clear();

			sdi12_DataPtr = node_prep_SDI12_msg(sdi12_msg_signal);
			sdi12_msg_signal = 0xff;
			//dogm_puts(sdi12_DataPtr);
		}

	// Main WSN state machine
		switch ( state )  {

			//During normal program flow, this state exits when RX ISR sets state to kWSN_StatMessageWaiting
			case kWSN_StatWaitingForMessage:
				if ( timer_done )  {
					dogm_clear();
					dogm_puts( "No response!" );

					// Log error
					nodes[node_ids[current_node]].UART_timeouts++;
					start_timer(DISPLAY_DELAY_SHORT);
					state = kWSN_StatNextNode;
				}
			break;

			case kWSN_StatPacketError:
				// Log error
				nodes[node_ids[current_node]].Packet_errors++;
				dogm_puts( "Packet error!" );
				start_timer(DISPLAY_DELAY_SHORT);
				state = kWSN_StatNextNode;
			break;

			case kWSN_StatMessageWaiting:
				//Turn off timer, because a message was received. Timer isn't
				// used during initialization routine.
				if ( initialized ) {
					reset_timer();
				}
				state = wireless_parse_message( initialized );
			break;

			case kWSN_StatBeforeSampling:
				dogm_clear();
				dogm_puts("Network awake");
				start_timer( NETWORK_AWAKE_DELAY );
				state = kWSN_StatWarmup;
			break;

			case kWSN_StatWarmup:
				if ( timer_done )  {
					state = kWSN_StatSampling;
				}
			break;

			case kWSN_StatSampling:
				if ( current_node < number_of_nodes )  {	// 0-index, haven't sampled all the probes yet
					dogm_clear();
					itoa(node_ids[current_node], lcd_string, 10);
					dogm_puts(lcd_string);

					start_timer( UART_TIMEOUT );
					state = kWSN_StatWaitingForMessage;

					wireless_turn_on_probes(node_ids[current_node]);
				}
				else  {		// All probes have been sampled
					dogm_clear();
					dogm_puts("Done sampling");

					newly_asleep = true;
					state = kWSN_StatDoneSampling;
				}
			break;

			// Probes are on, so start warmup timer
			case kWSN_StatProbesOn:
				start_timer( SAMPLE_DELAY );
				state = kWSN_StatProbeWarmup;
			break;

			case kWSN_StatProbeWarmup:
				if ( timer_done )  {	//Warmup timer has expired
					start_timer( UART_TIMEOUT );
					state = kWSN_StatWaitingForMessage;
					wireless_sample_DIO( nodes[node_ids[current_node]].SL, nodes[node_ids[current_node]].SH );
				}
			break;

			case kWSN_StatSampleReady:
				if ( node_validate_sample(ADC_sample.ADC1) )  {
					nodes[ADC_sample.node].probe[0].data[nodes[ADC_sample.node].current_sample] = ADC_sample.ADC1;
					node_incr_data_count( ADC_sample.node, 0 );
				}
				else  {
					nodes[ADC_sample.node].probe[0].data[nodes[ADC_sample.node].current_sample] = 0;
					node_decr_data_count( ADC_sample.node, 0 );
				}

				if ( node_validate_sample(ADC_sample.ADC2) )  {
					nodes[ADC_sample.node].probe[1].data[nodes[ADC_sample.node].current_sample] = ADC_sample.ADC2;
					node_incr_data_count( ADC_sample.node, 1 );
				}
				else  {
					nodes[ADC_sample.node].probe[1].data[nodes[ADC_sample.node].current_sample] = 0;
					node_decr_data_count( ADC_sample.node, 1 );
				}

				dogm_gotoxy(2,0);
				//Plus one to convert from 0-indexed array to 1 through 16
				itoa(nodes[node_ids[current_node]].current_sample + 1, lcd_string, 10);
				dogm_puts(lcd_string);
				dogm_puts("of16 Avg");

				if( nodes[node_ids[current_node]].current_sample + 1 < 10 )
					dogm_puts(" ");

				// Display average values
				itoa(node_calculate_average(ADC_sample.node,0), lcd_string, 10);
				dogm_puts(lcd_string);
				itoa(node_calculate_average(ADC_sample.node,1), lcd_string, 10);
				dogm_gotoxy(12,1);
				dogm_puts(lcd_string);

				// Display sampled values
				dogm_gotoxy(0,1);
				itoa(ADC_sample.ADC1, lcd_string, 10);
				dogm_puts(lcd_string);
				dogm_puts(",");
				itoa(ADC_sample.ADC2, lcd_string, 10);
				dogm_puts(lcd_string);

				// Increment current_sample for the current_node
				node_incr_sample_idx(ADC_sample.node);

				start_timer( UART_TIMEOUT );
				state = kWSN_StatWaitingForMessage;
				wireless_turn_off_probes( node_ids[current_node] );
			break;

			case kWSN_StatProbesOff:
				start_timer( DISPLAY_DELAY );
				state = kWSN_StatNextNode;
			break;

			case kWSN_StatNextNode:
				if ( timer_done )  {
					current_node++;
					state = kWSN_StatSampling;
				}
			break;

			// Nothing to do
			case kWSN_StatDoneSampling:
			break;

			case kWSN_StatAsleep:
				if ( newly_asleep )  {
					seconds = SLEEP_SECONDS;
					start_timer( OVERFLOWS_PER_SECOND );
					dogm_clear();
					dogm_puts("Network asleep");
					dogm_gotoxy(0,1);
					dogm_puts("Awake in:");
					dogm_gotoxy(14,1);
					dogm_putc('s');
					current_node = 0;
					newly_asleep = false;
				}
				else if ( timer_done )  {
					start_timer( OVERFLOWS_PER_SECOND );
					seconds = seconds - 1;
					dogm_gotoxy(10,1);
					if ( seconds < 1000 && seconds >= 100 )
						dogm_putc('0');
					else if ( seconds < 100 && seconds >= 10 )
						dogm_puts("00");
					else if ( seconds < 10 )
						dogm_puts("000");
					itoa(seconds, lcd_string, 10);
					dogm_puts(lcd_string);
				}
			break;

			case kWSN_StatNodeDiscovery:
				if ( timer_done )  {
					if ( number_of_nd_nodes == 0 ) {
						dogm_clear();
						dogm_puts("No nodes found!");
						dogm_gotoxy(0,1);
						dogm_puts("restarting...");
						//wdt_enable(WDTO_120MS);
					}
					else  {
						dogm_clear();
						dogm_puts("ND Done!");
						_delay_ms(1000);
						dogm_clear();
						dogm_puts("Reading SDI-12");
						dogm_gotoxy(0,1);
						dogm_puts("Adresses:");
						overflows = 0;
						state = UNINITIALIZED;
						// start timer for assigning SDI-12 addresses - if it timeouts, restart
					}
				}
			break;

			// This is Xbee-specific
			case UNINITIALIZED:
				if ( number_of_nodes <  number_of_nd_nodes )  {
					switch ( init_status )  {

						//Message has been sent; expecting a response
						case INIT_WAITING:
						break;

						case IO_UNINITIALIZED:
							init_status = INIT_WAITING;
							wireless_initialize_IO(temp_nodes[number_of_nodes].SL,temp_nodes[number_of_nodes].SH);
						break;
						case ADDR_UNINITIALIZED:
							init_status = INIT_WAITING;
							wireless_sample_DIO(temp_nodes[number_of_nodes].SL,temp_nodes[number_of_nodes].SH);
						break;
						case ADDR_INITIALIZED:
							init_status = INIT_WAITING;
							wireless_start_network_sleep(temp_nodes[number_of_nodes].SL,temp_nodes[number_of_nodes].SH);
					}
				}
				else {
					dogm_clear();
					dogm_puts("Starting sleep");
					_delay_ms(500);
					initialized = true;
					wireless_start_sleep();
					sdi12_init();
					state = kWSN_StatDoneSampling;
				}
			break;
		}
	}
}

// This is XBee-specific
ISR(USART1_RX_vect)
{
	uint8_t ReceivedByte = UDR1;
	current_byte++;

	if ( next_byte_is_len2 )  {
		xbee_incoming_length = ReceivedByte;
		next_byte_is_len2 = false;
		checksum = 0;
	}
	else if ( next_byte_is_len1 )  {
		next_byte_is_len1 = false;
		next_byte_is_len2 = true;
	}
	else if ( ReceivedByte == API_start_delimiter )  {
		next_byte_is_len1 = true;
		xbee_incoming_length = 0;
		current_byte = 1;
		BUFF_InitialiseBuffer();
	}
	else
		checksum += ReceivedByte;

	BUFF_StoreBuffByte(ReceivedByte);

	if ( current_byte == xbee_incoming_length + 4 )  {
		if( (uint8_t) checksum == 0xFF )  {
  			state = kWSN_StatMessageWaiting;
		}
	}
}

void initialize()
{
	// read reset flags
	if ( MCUSR ) {
		uint8_t temp = MCUSR;
	}

	// clear reset flags
	MCUSR  &= ~((1<<JTRF)|(1<<WDRF)|(1<<BORF)|(1<<EXTRF)|(1<<PORF));
	wdt_disable();
	//WDTCSR |= (1<<WDCE);
	//WDTCSR &= ~(1<<WDE);
	/* Turn off WDT */
	//WDTCSR = 0x00;

	// setup timer prescaler (divide by 1024)
	TCCR0B = (1<<CS02) | (1<<CS00);

	// initialize ring buffer for UART1 Rx interrupt
	BUFF_InitialiseBuffer();

	uart_init();

	dogm_init();
	dogm_clear();
	dogm_puts("Starting up...");
	_delay_ms(2000);
	dogm_clear();
	dogm_puts("Node Discovery");
	dogm_gotoxy(0, 1);
	dogm_puts("Found:");

	// set timer0 for node discovery
	sei();
	start_timer(ND_PERIOD);

	// issue node_discover command - response is handled by RX1 interrupt
	wireless_node_discover();
}

void start_timer(uint16_t counts)
{
	overflow_counter = counts;
	overflows = 0;
	timer_done = false;
	TIMSK0 |= (1<<TOIE0);
}

void reset_timer()
{
	timer_done = false;
	overflows = 0;
	TIMSK0 &= ~(1<<TOIE0);
}

ISR(TIMER0_OVF_vect)
{
	overflows++;

	if (overflows >= overflow_counter) {
		timer_done = true;
		overflows = 0;
		TIMSK0 &= ~(1<<TOIE0);
	}
}

void wd_start(void)
{
wdt_reset();
MCUSR &= ~(1<<WDRF);
WDTCSR |= (1<<WDCE) | (1<<WDE); //needed by manual p 50
WDTCSR = 68; // = WDIE=1, WDP2=1, WDE=0 (no reset)
return;
}

ISR(BADISR_vect)
{
}

