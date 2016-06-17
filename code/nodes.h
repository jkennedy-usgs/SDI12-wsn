//*****************************************************************************
//	Header file for node module for SDI-12 bridge project
//		Jeff Kennedy, USGS
//		January 8, 2011
//
//*****************************************************************************

#ifndef NODES_H
#define NODES_H

#define DATA_BUFFER_SIZE  16
#define NODE_ARRAY_SIZE   10

typedef struct
{
  	uint32_t SL;               // Serial number low
  	uint32_t SH;               // Serial number high
} _temp_node;

typedef struct
{
	uint16_t	data[DATA_BUFFER_SIZE];
	uint8_t		num_good_samples;
} _probe;

typedef struct
{
  	uint32_t 	SL;               			// Serial number low
  	uint32_t 	SH;               			// Serial number high
  	_probe	 	probe[2];
  	uint8_t 	current_sample;				// Index of current sample in data array
  	uint16_t 	UART_timeouts;				// Data quality check: number of UART timeouts
  	uint16_t 	Packet_errors;				// Data quality check: number of packet errors
  	uint16_t 	CRC_errors;					// Data quality check: number of checksum errors
  	uint8_t 	DIP_setting;				// DIP switch setting. Also equal to the SDI-12 address.
} _node;

extern _temp_node 	temp_nodes[NODE_ARRAY_SIZE];
extern _node 		nodes[NODE_ARRAY_SIZE];
extern uint8_t 		node_ids[NODE_ARRAY_SIZE];
extern uint8_t 		number_of_nodes;
extern uint8_t 		number_of_nd_nodes;

void node_incr_sample_idx(uint8_t ID);
char * node_prep_SDI12_msg(uint8_t ID);
uint16_t node_calculate_average(uint8_t ID, uint8_t probe);

#endif
