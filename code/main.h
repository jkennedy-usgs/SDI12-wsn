#ifndef MAIN_H
#define MAIN_H

#include <inttypes.h>
#include <stdbool.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>


#define kWSN_StatMessageWaiting			1
#define kWSN_StatWaitingForMessage		2
#define kWSN_StatAsleep					3
#define kWSN_StatBeforeSampling			4
#define kWSN_StatWarmup					5
#define kWSN_StatSampling				6
#define kWSN_StatDoneSampling			7
#define kWSN_StatProbesOn				8
#define kWSN_StatProbeWarmup			9
#define kWSN_StatProbesOff				10
#define kWSN_StatSampleReady			12
#define kWSN_StatNextNode				13
#define kWSN_StatPacketError			14
#define kWSN_StatNodeDiscovery			15
#define UNINITIALIZED 					0



#define SAMPLE_DELAY					20						// delay between turning probes on and reading ADC
#define NETWORK_AWAKE_DELAY				100						// delay between "network woke up message" and starting to sample probes
#define DISPLAY_DELAY					200
#define DISPLAY_DELAY_SHORT				40
#define ND_PERIOD						1000

#define OVERFLOWS_PER_SECOND 			61
#define UART_TIMEOUT					200

#define NO_SLEEP_MESSAGES				false
#define SEND_SLEEP_MESSAGES				true

// Sleep times for initial setup.
#define SETUP_SLEEP_TIME 				0x0010
#define SETUP_WAKE_TIME 				0x1530
// Sleep times used during operation, after initial setup:
#define SLEEP_TIME 						1000					// 10's msec; 	0xFFFF~11min.,  03E8 = 10 sec, 1770 = 60 sec., 7530 = 300 sec.
#define WAKE_TIME 						25000					// msec; 		0x1388 = 5 sec, EA60 = 60 sec, 7530 = 30 sec.
#define SLEEP_SECONDS 					(SLEEP_TIME/100)		// for lcd display

// typedefs




typedef struct
{
	uint16_t	ADC1;
	uint16_t	ADC2;
	uint8_t		node;
} _ADC_sample;

// GLOBAL VARIABLES


extern volatile 	uint8_t init_status;
extern _ADC_sample	ADC_sample;


#endif
