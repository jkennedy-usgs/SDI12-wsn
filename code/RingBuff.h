#ifndef RINGBUFF_H
  #define RINGBUFF_H
   
  // Configuration:
  #define BuffLen 60         // Replace "10" with desired maximum buffer elements
  typedef uint8_t BuffType; // Replace "uint8_t" with desired buffer storage type
  typedef uint8_t ElemType; // Replace "uint8_t" with the smallest datatype that can hold BuffLen

	// Macros:
	#define BUFF_ClearBuffError()  BuffError = 0;
	#define BUFF_ClearBuffer()     BuffElements = 0;
	
	#define BUFF_REMOVE_DATA  1
	#define BUFF_LEAVE_DATA   0
	
	#define BUFF_ERR_OVERFLOW (1 << 1)
	#define BUFF_ERR_EMPTY    (1 << 0)	
#endif

// Extern Variables:
extern volatile unsigned char BuffError;    // Bit 0 is set on empty read, bit 1 set on buffer full on write
extern volatile ElemType        BuffElements; // Holds the number of elements in the buffer

// Prototypes:

/* Call once at program startup. */
void     BUFF_InitialiseBuffer(void);

/* Use the data to be stored as the parameter. Must
be the type specified by the config typedef above. */
void     BUFF_StoreBuffByte(BuffType DataToStore);

/* Use BUFF_REMOVE_DATA as the parameter to remove the byte
from the buffer after it is read, otherwise BUFF_LEAVE_DATA
to leave it (subsequent calls to the routine will reread the
same byte. */
BuffType BUFF_GetBuffByte(uint8_t Pop);
