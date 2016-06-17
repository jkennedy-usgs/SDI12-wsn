//*****************************************************************************
//	Node module for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//	This module contains code pertaining to individual nodes. It's not specific
//   to any particular wireless protocol, or to wireless at all. Details of
//	 each node are stored in the "nodes" array of node structs. Sensor data is
//	 stored in a simple ring buffer.
//*****************************************************************************

#include <string.h>
#include "main.h"
#include "wireless_xbee.h"
#include "nodes.h"

//char array that will hold the response message to the host data logger
char SDI12_string[35];

uint16_t SDI12counter = 0;

void node_incr_sample_idx( uint8_t node_ID )
{
	if ( nodes[node_ID].current_sample >= (DATA_BUFFER_SIZE - 1)  )
		nodes[node_ID].current_sample = 0;
	else
		nodes[node_ID].current_sample++;
}

void node_incr_data_count( uint8_t node_ID, uint8_t probe_ID )
{
	if ( nodes[node_ID].probe[probe_ID].num_good_samples < DATA_BUFFER_SIZE )
		nodes[node_ID].probe[probe_ID].num_good_samples++;
}

void node_decr_data_count( uint8_t node_ID, uint8_t probe_ID )
{
	if ( nodes[node_ID].probe[probe_ID].num_good_samples > 1)
		nodes[node_ID].probe[probe_ID].num_good_samples--;
}

bool node_validate_sample(uint16_t sample)
{
	//if ( sample != 0x03FF && sample != 0x0000 )
		return true;
	//else
	//	return false;
}

uint16_t node_calculate_average(uint8_t ID, uint8_t probe)
{
	uint16_t sum = 0;
	uint16_t avg = 0;

	for ( uint8_t avg_idx = 0; avg_idx < 16; avg_idx++ )  {
			sum += nodes[ID].probe[probe].data[avg_idx];
	}
	if ( nodes[ID].probe[probe].num_good_samples > 0 )
		avg = sum / nodes[ID].probe[probe].num_good_samples;
	else
		avg = 0;

	return avg;
}

char* node_prep_SDI12_msg(uint8_t node_ID)
{
	strcpy(SDI12_string, "d+");
	char avg[8];

	itoa(node_calculate_average(node_ID, 0),avg,10);
	strcat(SDI12_string, avg);
	strcat(SDI12_string, "+");
	itoa(node_calculate_average(node_ID, 1),avg,10);
	strcat(SDI12_string, avg);
	return SDI12_string;
}


