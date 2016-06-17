/*
                  Simple Ringbuffer Library
                  -------------------------
						  (c) Dean Camera, 2005
						-------------------------
						 dean_camera@hotmail.com

          
 This library allows you to implement a ringbuffer into your code
 (useful for buffering FIFO data while the processor is busy)
 via easy-to-use functions.
 
 By changing the typedefs in RingBuff.h, you can change both the
 datatype stored and the size of the buffer.
 
 An error variable is exposed via extern to your program; if the
 buffer is full when you try to store data bit 1 of the error
 variable is set, and if you try to read an empty buffer bit 0 is
 set. Both bits are cleared after a sucessful data store or read.
 The error masks are avaliable via the defines BUFF_ERR_OVERFLOW
 and BUFF_ERR_EMPTY.

 Before it can be used, you must execute the "InitialiseBuffer"
 routine. To store data, use the "StoreBuffByte" routine and for
 reading data you use the "StoreBuffByte" routine.
 
 The extern BuffElements variable holds the number of elements
 in the buffer. This can be polled to check if the buffer is
 empty or not.
 
 The macro "ClearBuffError()" is defined in the RingBuff.h file
 so you can clear the error variable manually if you wish.
 
                [{ Feedback Appreciated }]
*/


// Includes:
#include <avr/io.h>
#include "RingBuff.h"

// Global Variables:
volatile BuffType       *StoreLoc;
volatile BuffType       *RetrieveLoc;
volatile BuffType       RingBuffer[BuffLen];
volatile ElemType       BuffElements;
volatile unsigned char BuffError;

// Routines:
void BUFF_InitialiseBuffer(void)
{
	StoreLoc    = (BuffType*)&RingBuffer; // Set up the IN pointer to the start of the buffer
	RetrieveLoc = (BuffType*)&RingBuffer; // Set up the OUT pointer to the start of the buffer

	BuffElements = 0;           // Reset the buffer elements counter
}

void BUFF_StoreBuffByte(BuffType DataToStore)
{
	if(BuffElements == BuffLen) // Buffer full
	{
		BuffError |= BUFF_ERR_OVERFLOW;   // Set the "buffer full" error flag
		return;
	}
	else                       // Sucessfull operation
	{
		BuffError = 0;          // Clear the error variable
	}
		
	*StoreLoc = DataToStore;   // Store the data

	StoreLoc++;                // Increment the IN pointer to the next element
	BuffElements++;            // Increment the total elements variable

	if (StoreLoc == (BuffType*)&RingBuffer[BuffLen])
		StoreLoc = (BuffType*)&RingBuffer; // Wrap pointer if end of array reached
}	

BuffType BUFF_GetBuffByte(uint8_t Pop)
{
	if(!(BuffElements))        // No elements in the buffer
	{
		BuffError |=  BUFF_ERR_EMPTY;       // Set the "buffer empty" error flag
		return 0;
	}
	else                      // Sucessfull operation
	{
		BuffError = 0;         // Clear the error variable
	}

	BuffType RetrievedData = *RetrieveLoc; // Grab the stored byte into a temp variable

	if (Pop)
	{
		RetrieveLoc++;   // Increment the OUT pointer to the next element if flag set
		BuffElements--;  // Decrement the total elements variable
	}
	
	if (RetrieveLoc == (BuffType*)&RingBuffer[BuffLen])
		RetrieveLoc = (BuffType*)&RingBuffer; // Wrap pointer if end of array reached
		
	return RetrievedData;    // Return the retrieved data
}
