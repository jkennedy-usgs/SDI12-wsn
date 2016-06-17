//**************************************************************
//  Header for SDI12.c module for Wireless SDI-12
//  Author: 	James Wagner, Oregon Research Electronics
//  Date:		April 30, 2010
//	Version:	1.2
//  Processor:	AVR AtMega644P/V - adaptable to other processors
//				with Pin Change Interrupts
//
// This header file contains all of the sdi-12 PUBLIC interface functions and variables.
// simply: #include "sdi12.h" in any module that needs to access these functions or variables. 
// The interface to the system is handled by sdi12_interface.c System-accessible functions 
// that have been created for the use of of sdi-12 modules are in sdi12_sys.c.
//**************************************************************
#ifndef SDI12_H
 #define SDI12_H
  
 #define SDI12_DEBUG	//controls inclusion of debugging statements

//PUBLIC constant declaration
//NB: data will be sent as soon as it is available. So, it is only necessary to set the
//    following value longer than the worst case delay. So, if it really takes 100ms
// 	  and you set 5 seconds, it will NOT run any slower. Largest value is given by
//	  65535 = Nmax * F_CPU/1024; For F_CPU = 16000000, we have 65535 = Nmax * F_CPU/1024 = N*15625
//    Nmax = 65536/15625 = 4.19 so largest integer is 4.
 #define kSDI12_MeasureWait	1		//number of wait seconds, max 4, min 1; will NOT handle 0 case
 
 
//PUBLIC variable declarations
  uint8_t sdi12_msg_signal;			//signal to wireless: 0x00 = idle; otherwise address of data requested device
  uint8_t sdi12_action;				//control variable
  uint8_t extern number_of_nodes; 	//declared in main module
  uint8_t extern node_ids[]; 		//declared in main module
  char * volatile sdi12_DataPtr;	//pointer to data message

//API function declarations
  void sdi12_init( void );	 	//PUBLIC  initializes sdi12 interface
  void sdi12_enable( void );	//PUBLIC  enables the sdi12 interface after being disabled
  void sdi12_disable( void );	//PUBLIC  disables the sdi12 interface
  void sdi12_dotask( void ); 	//PUBLIC  must be called regularly from main() to manage sdi12

#endif /* !SDI12_H */
