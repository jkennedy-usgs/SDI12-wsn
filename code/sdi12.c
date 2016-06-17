//**************************************************************
//  Main module for Wireless SDI-12
//  Author: 	James Wagner, Oregon Research Electronic
//  Date:		June 09, 2010
//  Version:	V1.3 new version Sept 10, 2010
//  Processor:	AVR AtMega644P/V
//
// This module contains all of the sdi-12 interface functions.
//
//On timing, for most things, Timer1 Compare Register A is used.
// A prescale of 1024 gives one count in 256us @ 4MHz (about 1/4ms)
// or 64us at 16MHz. Since Timer1 is 16 bits, the maximum count at
// 16MMHz gives is 4.19s. The original development board has a 4MHz
//	crystal while the final unit has 16MHz.
//
//A detailed description of each state follows, listing the active
// periphrals and the events that cause a transition to another state.
// The flow chart is described on page B2 of "SDI-12, A Serial-Digital
// Interface Standard for Microprocessor-Based Sensors, Version 1.3".
// Descriptions in [ ] denote locations within the flow chart.
//**************************************************************//
//
//                        SDI12 STATES
//
//**************************************************************//
//
// For auxiliary state control variables, see AUXVAR  at the end
// of the state descriptions.
//
//**************************************************************//
//+kSDI12_StatIdle 		[top center circle]
//						Waiting for something to happen: might or might
//						not be in a low power mode, depending on the
//						wireless requirments. Waits for a fallilng Pin
//						Change Int. Timer1 is off. UART Rx & RxInt are
//						off. UART TxInt is off. Pin Change Interrupt is
//						on.
//
// +TO:kSDI12_StatTstBrk on negative Pin Change Interrupt. Positive Pin
// 						Change is ignored. Reset and turn on Timer1.
//						Set CompareA to 100ms. No change PCint.
//--------------------------------------------------------------------
//+kSDI12_StatTstBrk	[top center circle]
//						The leading edge of a break has been detected
//						and it is waiting for the END of a break
//						interval or a 100ms timeout. There cannot be any
//						negative Pin Changes since state was entered
//						on a negative Pin Change. If a CompareInt
//						occurs, it means that the 100ms timeout has
//						Timer1 is running with CompareA = 100ms and overflow
//						interrupt is off. Pin Change Interrupt is on.
//						All UART enables and ints are off.
//
// +TO:kSDI12_StatIdle	Pin Change Detected and the timer has
//						fewer than 12ms. Turn Timer off.
//
// +TO:kSDI12_StatIdle	On Timer CompareA Int. Turn Timer off.
//
// +TO:kSDI12_StatTstMrk Positive Pin Change Detected and the timer has
//						more than 12ms. Reset timer and change
//						CompareA interrupt 8.19ms. Pin change is on
//						sdi12_RxData = 0 for shared states to follow
//--------------------------------------------------------------------
//+kSDI12_StatTstMrk	[2nd row left circle]
//						This state tests for a mark of at least 8.33ms
//						after a break to verify a valid system "break".
//						Timer1 is running with CompareA interrupt on and
//						set for 8.19ms. Pin Change is on. Receive is off.
//
// +TO:kSDI12_StatWaitAct On Timer CompareA Int. Reset timer and set CompareA
//						100ms since the end of the break. The allowance
//						is 100ms from the end of the break to a received character.
//						Here, we are 8.33ms into this interval but we can only
//						detect when a character HAS been received, 8.33ms after
//						the interval.
//						Turn on UART receiver and UART receiver int.
//
// +TO:kSDI12_StatTstBrk On Pin Change Interrupt. It is to early to be the
//						end of a valid mark. It MUST be negative since
//						it was immediately preceded by a positive change.
//						So this MIGHT be a new break period. Reset the
//						timer and change CompareA interrupt to 100ms.
//
//--------------------------------------------------------------------
// +kSDI12_StatWaitAct	[2nd row middle circle]
//						Finally, a valid break (break + mark) has been
//						detected. One of several things might happen.
//						A character might be received or a new break
//						might occur or the 100ms timer CompareA might trip.
//						Only the UART is on so if a break happens,
//						it will be detected as a character 0x00 and an
//						invalid stop bit.
//
// +TO:kSDI12_StatWaitChar On UART receive detect and valid address or '?' with
//						valid parity & stop bit (if sdi12_RxData == 0) or
//						address same as sdi12_RxAddr (if sdi12_RxData != 0).
//						Clear receive buffer and put character in the buffer.
//						Rx and RxInt remain on. Timer remains on to insure
//						proper response in case of a line fault.
//
// +TO:kSDI12_StatIdle 	On UART receive detect and invalid address or receive
//						error. UART receive and interrupt off. Timer1
//						is off and Pin Change Int is on.
//
// +TO:kSDI12_StatIdle 	On Compare detect. Timeout has occurred. Probable
//						line faule. Turn UART receive and interrupt
//						off. Timer1 is off and Pin Change Int is on.
//
// +TO:kSDI12_StatTstBrk On UART receive detect with value 0x00 and stop bit
//						error. This COULD be a new break but 8.33ms has already
//						passed since the start so reset the timer to 32 to
//						account for the time already passed. Set CompareA
//						interrupt to 100ms. UART receive off. Pin
//						Change Int on.
//
//--------------------------------------------------------------------
//+SDI12_StatWaitChr	[4th row middle circle]
//						This is the state waiting for the 2nd & following characters
//						to be received.. It is necessary to detect 1.66ms (max) marks.
//						The start of the timing interval is the end of the
//						previous stop bit. The timer reset on entry to the
//						state and compare is set to 10ms (sum of 1.66ms and 8.33ms
//					    character duration). Pin Change is OFF. UART receive is on.
//						receive int. If the timer expires first, exit to idle.
//						If the RxInt happens first, continue until '/n' is received
//
// +TO:kSDI12_StatIdle	On timer expire.UART receive and interrupt
//						off. Timer1 is off and Pin Change Int is on.
//
// +TO:kSDI12_StatIdle	On RxInt, invalid char or 10th char. Timer1 is off
//						and Pin Change Int is on.
//
// +TO:SDI12_StatWaitChr On RxInt, valid character NOT '/n' and fewer than 9 chrs.
//						Reset timer and set compare for 10ms. UART receiver and
//						UART receiver int are on.
//
// +TO:kSDI12_StatSndMrk On RxInt, character IS '/n' and valid. Pin Change
//						off, Rx off, RxInt off. Timer on and compare is
//						set for 8.45ms. Put character into buffer.
//						Put TxPin into mark state.
//						Start background command processing.
//
//----------------------------------------------------------------------------
// +kSDI12_StatSndMrk	[6th row middle square]
//						In this state, the bridge is "sendng" an 8.45ms
//						mark AND is processing the received command.
//						Timer is on and compare = kSDI12_tim8_45long.
//						Transmit buffer is ON and output is mark.
//						Everything else is off.
//
// +TO:kSDI12_StatSndResp On timer compare. Send the command response.
//						Turn on external transmit enable.
//
//----------------------------------------------------------------------------
// +kSDI12_StatSndResp 	[7th row middle square]
//						Here, the UART is sending an ACK, or data (as in
//						'I' command). What it does at the end of the
//						message depends on the command. If the command
//						was an 'M' variant, it begins a delay after which
//						it will send a service request OR receive a 'D'.
//						Receive is off. Timer is off. Pin Change is off.
//						Send a response character.
//
// +TO:kSDI12_StatSndResp on tx done int and another charcter yet to send.
//
// +TO:kSDI12_StatWaitSRQ	On tx done int and response done and command is
//						"M". Turn off external transmit enable. Reset the
//						timer and set compare for 100ms. Reset sdi12_waitSRQ_cnt
//						Turn off UART receiver and UART receiver int. PCInt
//						must be on for break detect. This waits for an SRQ to
//						send or a timeout or a break.
//
// +TO:kSDI12_StatIdle	On tx done int and response done and command is anything
//						BUT "M". Turn off external transmit enable.
//						Reset & turn off timer. Turn off UART receiver
//						and UART receiver int. PCInt on.
//
//----------------------------------------------------------------------------
// +kSDI12_StatWaitSRQ  The timer is on, waiting for sdi1_DataPtr to become
//						non-zero or timeout or break. The timer interval is
//						100ms and each overflow causes sdi12_waitSRQ_cnt to
//						be incremented. It is incremented to
//						kSDI12_MeasureWait * 10. PCI is on for falling edges.
//
// +TO:kSDI12_StatWaitSRQ on timer int and sdi12_DataPtr == 0 and sdi12_waitSRQ_cnt <
//						kSDI12_MeasureWait * 10. Increment sdi12_waitSRQ_cnt
//						on each pass; otherwise, no changes.
//
// +TO:kSDI12_StatSendSRQ	on timer int and sdi12_DataPtr != 0. Turn off timer.
//						Turn on transmitter & buffer, load first char into UDRn
//						and set index to second char.
//
// +TO:kSDI12_StatABrk on negative Pin Change Interrupt. Positive Pin
// 						Change is ignored. Reset Timer1.
//						Set CompareA to 100ms. No change PCint.
//
// +TO:kSDI12_StatIdle	on timer int and  sdi12_waitSRQ_cnt == kSDI12_MeasureWait * 10
//						Turn on PCI for break detect.
//
//----------------------------------------------------------------------------
// kSDI12_StatABrk	Like kSDI12_StatTstBrk. The leading edge of an abort break
//						has been detected and it is waiting for the END of the
//						break interval or a 100ms timeout. There cannot be any
//						negative Pin Changes since state was entered
//						on a negative Pin Change. If a CompareInt
//						occurs, it means that the 100ms timeout has
//						Timer1 is running with CompareA = 100ms and overflow
//						interrupt is on. Pin Change Interrupt is on.
//						All UART enables and ints are off.In this state, a request to
//						generate an SRQ is ignored.
//
// +TO:kSDI12_StatIdle On timer expire with no rising edge. Turn off timer
//						and leave on PCI int.
//
// +TO:kSDI12_StatTstMrk On rising PCI int with Timer > 12ms represents valid
//						abort break detect. Set abort flag. Wait for remainder
//						of the "D" command (assuming that the just detected break
//						was the beginning of the D command. A mark has to come
//						next. The mark can be the same for abort and regular
//						data since the abort stat is carried in a testable flag.
//						Reset timer and CompareA interrupt 8.19ms. Pin change is on
//
// +TO:kSDI12_StatIdle On rising PCI int with Timer < 12ms represents invalid
//						abort break detect. clear abort flag. Return to idle.
//
//----------------------------------------------------------------------------
// +kSDI12_StatSendSRQ 	Sending service request message. PCInt is off. Timer
//						is off. Rx and RxInt are off.
//
// +TO:kSDI12_StatSendSRQ On TxComplete Int and next available transmit char
//						not null. Load the next character into UDRn.
//
// +TO:kSDI12_StatWaitDBrk On TxComplete Int and next available transmit char
//						is null. Disable external transmit enable. Disable
//						TxInt. Reset timer and set CompareA for 85ms.
//						Reset sdi12_waitSRQ_cnt.PCI on for break detect.
//						Receiver on for possible D command address but RxInt
//						is off.
//
//----------------------------------------------------------------------------
// +kSDI12_StatWaitDBrk	SRQ sent, waiting for a break, or first D-command
//						character or a 85ms timeout. The 85ms represents
//						the interval after an SRQ in which the host does
//						NOT need to send a break/mark pair but MAY. Tx off
//						& Rx on but RxInt is off. Timer1  with CompareA
//						set to 85ms is on. PCInt is on. If the break edge
//						is detected within the 85ms interval, it MIGHT be
//						either a real break start OR it might be the leading
//						edge of	a start bit.
//
//  TO:kSDI12_StatDTst	On PCI neg edge detect, signifying leading edge of
//						break OR start bit of D command address bit.
//	`					Timer remains on and Pin Change Int is on.
//						Receive remains on & RxInt off. Timer compare set to
//						200ms for failsafe timeout.
//
//  TO:kSDI12_StatIdle	On PCI pos edge detect, signifying invalid edge.
//						Timer off, Receive off, RxInt back on, PCI on.
//
//  TO:kSDI12_StatWaitDBrk2 On timeout. Timer on at 200ms failsafe. PCI is on.
//						Receiver is OFF and RxInt back on.
//						Wait for regular break/mark/command.
//
//----------------------------------------------------------------------------
// +kSDI12_StatDTst 	kSDI12_StatWaidDBrk has detected a transition within
//						the 85ms window after an SRQ when EITHER a break or
//						a D command without a break is allowable. This
//						means EITHER the start of a break, the start bit
//						of a character, or a fault condition. The timer
//						is on with an interval of 200ms as a failsafe.
//						If the PCI trips at less than 8.3ms, it is likely
//						a character (which is ultimately validated by the
//						receive filter). If it trips at more than 8.3ms but
//						less than 12ms, its not a valid break & too long
//						to be a character. If more than 12ms but less than
//						200ms, its a break. If the timer times out, it is
//						a fault. PCI is on, Rx is on, timer is on with 200ms timeout.
//
// +TO:kSDI12_StatDChr	On PCI edge detect, timer less than 8.3m so probable
//						character. Leave Rx on AND turn RxInt back on to catch
//						character. Leave timer on as failsafe (10ms). PCI off.
//
// +TO:kDSI12_StatIdle 	On PCI edge detect, timer more than 8.3ms but
//						less than 12ms (error). Timer off. Receive off.
//						RxInt goes back on. PCI only thing on.
//
// +TO:kSDI12_StatIdle 	On timeout. This means the "break" is too long and it is
//						probably a line fault. Timer off. RxInt on and PCI on.
//
// +TO:kSDI12_StatTstMrk  On PCI edge detect, timer more than 12ms (valid break).
//						Mark is next, so back to the regular reception routine.
//						RxInt back on. Reset timer and change CompareA interrupt
//						8.19ms. Pin change is on sdi12_RxData = 0 for shared
//						states to follow
//----------------------------------------------------------------------------
// +kSDI12_StatWaitDBrk2 kSDI12_StatWaidDBrk has timed out. This means
//					 	that any D command MUST now have a full break-
//						mark pair. Timer is on with a timeout of 200ms
//						as failsafe.
//
// +TO:kSDI12_StatDBrk	On PCI neg edge detect, signifying leading edge of
//						break or an error condition.
//	`					Timer remains on and Pin Change Int is on.
//						Timer compare set to 100ms for failsafe timeout.
//
// +TO:kSDI12_StatIdle On PCI po edge detect. Wrong edge. All off but PCI.
//
// +TO:kSDI12_StatIdle On timeout. Timer off. PCI only thing on.
//
//----------------------------------------------------------------------------
// +kSDI12_StatDBrk		Leading edge of post-SRQ break has been detected.
//						Waiting for end of break. Timer 100ms and PCI on.
//						Transmit and receive off. Break only valid if
//						duration is greater than 12ms.
//
// +TO:kSDI12_StatIdle	On timeout. PCInt on, Rx off, RxInt off, timer off.
//
// +TO::kSDI12_StatIdle  On PCI int with timer < 12ms. Don't care sense.
//						Too short. All off except PCI.
//
// +TO:kSDI12_StatTstMrk PCI int with Timer > 12ms represents valid
//						abort break detect. Must be rising because previous was
//						falling. CLEAR abort flag. Wait for remainder
//						of the "D" command (assuming that the just detected break
//						was the beginning of the D command. A mark has to come
//						next. The mark can be the same for abort and regular
//						data since the abort stat is carried in a testable flag.
//						Reset timer and CompareA interrupt 8.19ms. PCI is on
//
//-----------------------------------------------------------------------------
// kSDI12_StatDChr		Preliminaries of the first character of a D command have
//						been detected. Wait for the final UART character int
//						with a failsafe of 10ms.
//
// +TO:kSDI12_StatIdle	On timeout. Fault. PCInt on, Rx off, RxInt off, timer off.
//
// TO::kSDI12_StatWaitAct  On UART receive detect and valid address with
//						valid parity & stop bit (if sdi12_RxData == 0) or
//						address same as sdi12_RxAddr (if sdi12_RxData != 0).
//						Clear receive buffer and put character in the buffer.
//						Rx and RxInt remain on. Timer remains on to insure
//						proper response in case of a line fault.
//
//-----------------------------------------------------------------------------
//*****************************************************************************//
//
//               AUXVAR - Auxiliary State Control Variables
//
//See: PRIVATE variable declarations, below, for physical declarations
//
//-----------------------------------------------------------------------------
//
//uint8_t volatile sdi12_Status contains the current SDI-12 state. These are
//  defined by constants named kSDI12_StatXXXXXX. Descriptions of these states
//	are given in the list immediately preceding.
//
//-----------------------------------------------------------------------------
//
//uint8_t volatile sdi12_flags;	action bits are defined by constants given below

//	kSDI12_RcClr	0		//clear sdi12_flags
//	kSDI12_RxCmd	(1<<0)	//command received
//	kSDI12_ProcCmd  (1<<1)	//command has been processed, ready to send
//	kSDI12_ProcErr  (1<<2)	//command has been processed, invalid command
//	kSDI12_CRCFlg	(1<<3)	//command contained CRC request
//	kSDI12_CmdM		(1<<4)	//command M
//	kSDI12_CmdV		(1<<5)	//command V
//	kSDI12_CmdC		(1<<6)	//Command C
//	kSDI12_Abort	(1<<7)	//aborted command

//This variable controls interaction between the receiver and the command parser
//	and carries some of the parser results. When a new transaction starts, sdi12_flags
//	must be 0.
//
//	When a complete command is received by USART_Rx_ISR (as defined by valid address,
//	valid command and modifiers, plus \n\r), sdi12_flags bit corresponding to
//	kSDI12_RxCmd is set. Other bits MAY be set since the same receive routine is
//  used for both the initial command and the data request.
//
//	When sdi12_dotask() finds the kSDI12_RxCmd bit set in sdi12_flags, sdi12_cmd_parse( )
//	is called. sdi12_cmd_parse() clears this bit and may will set EITHER kSDI12_ProcCmd
//	bit OR kSDI12_ProcErr bit. In the case of the ProcErr bit, the SDI-12 state is
//	returned to idle and no response is sent. In the case of ProcCmd bit, a response
//	IS sent.
//
//	The remainder of the bits hold some results of the parse for later use when
//	data is requested. In particular, the kSDI12_CRCFlg bit is set if the command
//  requested data including a CRC. The kSDI12_CmdM, kSDI12_CmdV, & kSDI12_CmdC
//  bits are set according to the command (M, V, or C) received. "Single shot" commands
//  (M & V) have the bits fully cleared when the data is returned to the host. Other
//  commands (C) allow these bit to persist until a new, replacement, command is received.
//
//	The kSDI12_Abort bit is set when an abort condition has been detected.
//
//-----------------------------------------------------------------------------
//
//uint8_t volatile sdi12_RxData;	action bits are defined later
//
//Low 4 are for the "n" in "aDn\n\r" or corresponding "R" command
//High 4 are for flags
//
//	kSDI12_RxD  		(1<<4)	//D received - one-time data value
//	kSDI12_RxR			(1<<5)  //R received - one of series of continuous values
//
//So sdi12_RxData carries information about the data request - which request
//and the request index number. The index is just the ASCII index character with
//0x30 subtracted. If there is no index (as in the possible case of "D" command),
//a zero is used.
//
// -----------------------------------------------------------------------------
//
//GENERAL WORKING VARIABLES - receive data
//
//char volatile sdi12_RxBuf[10] 	sdi12 receive buffer - 7 obvious command chars, max
//									(inc \r\n)
//
//uint8_t volatile sdi12_RxIndx;	array index for sdi12_RxBuf
//
//uint8_t volatile sdi12_RxAddr;	received ASCII address
//
//char	volatile sdi12_cmdchr;		received command character
//
//uint8_t sdi12_query_count;		rotating index for query responses rotates through
//									the address list in response to unaddressed address
//									query "?!" so that the full suite of device
//									addresses is recognized.
//
//char	volatile sdi12_addr[5];		array of valid SDI-12 addresses used to test
//									the received address and to generate responses
//									to "?!" commands.
//
//GENERAL WORKING VARIABLES - transmit.
//	Note that the transmitter can use multiple buffers. The general purpose reserved
//	buffer is sdi12_TxBuf[] but a different buffer is usually provided by the wireless
//	side. The actual buffer is pointed to by sdi12_SendPtr while transmitting.
//
//char * volatile sdi12_DataPtr		pointer to data message provided by wireless
//
//char * volatile sdi12_SendPtr		pointer to data being transmitted. This CAN be
//									equal to sdi12_DataPtr when wireless data is being
//									sent or sdi12_TxBuf when other data is being sent.
//
//char sdi12_TxBuf[40]				sdi12 transmit buffer
//
//uint8_t volatile sdi12_TxIndx		array index for sdi12_TxBuf
//
//GENERAL WORKING VARIABLES - timing
//
// uint8_t volatile sdi12_ticcnt	counts 50ms ticks
//
// uint8_t volatile sdi12_seccnt	seconds counter
//
// uint8_t volatile sdi12_waitSRQ_cnt 50ms pass counter during wait_SRQ
//
//*****************************************************************************//
/*
//A framework for state handling in ISRs
	switch ( sdi12_Status )
		{
		case kSDI12_StatIdle:
			break;
		case kSDI12_StatTstBrk:
			break;
		case kSDI12_StatTstMrk:
			break;
		case kSDI12_StatWaitAct:
			break;
		case SDI12_StatWaitChr:
			break;
		case SDI12_StatRxChr:
			break;
		case kSDI12_StatSndMrk:
			break;
		case kSDI12_StatSndResp:
			break;
		case kSDI12_StatABrk:
			break;
		case kSDI12_StatWaitSRQ:
			break;
		case kSDI12_StatSendSRQ:
			break;
		case kSDI12_StatDBrk:
			break;
		case kSDI12_StatWaitD:
			break;
		case kSDI12_StatDChr:
			break;
		}
*/
//
//**************************************************************//
//COMMANDS. RESPONSES, and EVALUATION CRITERIA
//
//From V1.3 Specifications, Section 4.4 and following
//
//	command					response
//	a!		Ack active		a<cr><lf>
//	aI!		Send ident		allccccccccmmmmmmvvvxxx...xx<cr><lf>
//	aAb!	Addr change		b<cr><lf>
//	?!		Addr query		a<cr><lf>
//	aM!		Measure			atttn<cr><lf>
//	aMn!	Measure more	atttn<cr><lf>
//	aMC!	Measure+CRC		atttn<cr><lf>
//	aMCn!	Measure more+CRC atttn<cr><lf>
//	aDn!	Data Send grp-n	a<values><cr><lf> or a<values><CRC><cr><lf>
//	aV!		Verify			atttn<cr><lf>
//	aC!		Concurrent		atttnn<cr><lf>
//	aCn!	Concurrent more atttnn<cr><lf>
//	aCC!	Concurrent+CRC	atttnn<cr><lf>
//  aCCn!	Concurrent more+CRC atttnn<cr><lf>
//	aRn!	Continuous		a<values><cr><lf> or a<values><CRC><cr><lf>
//	aRCn!	Continuous+CRC	a<values><cr><lf> or a<values><CRC><cr><lf>
//  aXNNN!	Extended NNN	a<response><cr><lf>

//In a response, 'a' is responding address. 'ttt' is the time, in seconds,
// until a response will be ready; '000' indicates immediate return. 'n'
//is {0-9} is the number of values to be returned; 'nn' is {00-20}
//
//The character count for all commands must be at least 2 and no
// greater than 6
//
//First character must be the address (0-9,A-Z,a-z) for the receiving
// device or '?' to be valid.
//
//Last character must always be '!'
//
//If length is 3, the second character must be {I,M,D,V,C}.
// Case is important!
//
//If length is 4, the second (command) character must be {A,M,D,C,R}.
// The third character must be a valid address if command='A', a 'C'
// if command = {M,C,R}, {1-9} if command = {M,C}, or {0-9} if command = 'R'.
//
//If length is 5, the second (command) character must be {M,C,R}. The
// third character must be a 'C' and the fourth character, a {1-9} if
// command = {M,C}, or {0-9} if command = 'R'.
//
//If length is 6 or more, the command must be 'X' and 3rd to next-last
// characters are manufacturer defined.
//
//**************************************************************//
 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <avr/pgmspace.h>
 #include "sdi12.h"

#ifndef F_CPU
#error "PLEASE set CPU frequency in HZ in AVRStudio: Project>Configuration Options"
#endif

//In the folllowing set of defs, only ONE can be defined in the
//project. The selected UART cannot be used by other parts of the
//application. Defining SDI12_UART0 assigns UART0 (of a dual-UART
//porocesor) to the SDI12 interface, defining SDI12_UART1 assigns UART1
//of a dual UAER processor to the SDI12 interface. Defining SDI12-UART
//assigns the UART of a single-UART AVR to SDI-12. A single UART
//AVR does NOT know what UART0 or UART1 is; likewise, a 2-UART
//AVR does NOT know what UART is. See special note fillowing
//#elif defined (SDI12_UART) about bit names.

//Uncomment the following line to use UART0 in 2-UART AVR as the SDI12 serial interface
#define SDI12_UART0

//Uncomment the following line to use UART1 in 2-UART AVR as the SDI12 serial interface
//#define SDI12_UART1

//Uncomment the following line to use 1 UART AVR to use that UART as the SDI12 interface
//#define SDI12_UART

#ifndef SDI12_UART0
	#ifndef SDI12_UART1
		#ifndef SDI2_UART
			#error "No SDI12_UART was defined in sdi12.c Compile aborted"
		#endif
	#endif
#endif
//The following defines various Mega644P registers, bits, and definitions for
//break detecton, based on which of the two UARTs have been previously defined.
//A third definition is used for single UART devices
#if defined (SDI12_UART0)
  #if defined (SDI12_UART1) || defined (SDI12_UART)
  #error "Multiple SDI12 UARTS were defined in SDI12.c Compile aborted"
  #endif
#define USART_Rx_ISR	ISR(USART0_RX_vect)
#define USART_Tx_ISR	ISR(USART0_TX_vect)
#define USART_Port		PORTD
#define tx_pin			PD1
#define break_read		PIND
#define break_pin		PD0
#define PCINT_pin		PCINT24
#define UDRn			UDR0
#define UCSRnA			UCSR0A
#define RXCn			RXC0	// USART Receive Complete
#define TXCn			TXC0	// USART Transmit Complete
#define UDREn			UDRE0 	// USART Data Register Empty
#define	FEn				FE0		// Frame Error
#define	DORn			DOR0	// Data OverRun
#define	UPEn			UPE0	// USART Parity Error
#define	U2Xn			U2X0	// Double the USART Transmission Speed
#define MPCMn			MPCM0	// Multi-processor Communication Mode
#define	UCSRnB			UCSR0B
#define	RXCIEn			RXCIE0	// RX Complete Interrupt Enable n
#define	TXCIEn			TXCIE0	// TX Complete Interrupt Enable n
#define	UDRIEn			UDRIE0	// USART Data Register Empty Interrupt Enable n
#define	RXENn			RXEN0	// Receiver Enable n
#define	TXENn			TXEN0	// Transmitter Enable n
#define	UCSZn2			UCSZ02	// Character Size n
#define	RXB8n			RXB80	// Receive Data Bit 8 n
#define	TXB8n			TXB80	// Transmit Data Bit 8 n
#define	UCSRnC			UCSR0C
#define	UMSELn1			UMSEL01 // mode select
#define	UMSELn0			UMSEL00	// mode select
#define	UPMn1 			UPM01 	// parity select
#define	UPMn0			UPM00	// parity select
#define	USBSn			USBS0	// stop it select
#define	UCSZn1 			UCSZ01 	// bit select
#define	UCSZn0			UCSZ00	// bit select
#define	UCPOLn 			UCPOL0	// Clock Polarity
#define UBRRn			UBRR0

#elif defined (SDI12_UART1)
  #if defined (SDI12_UART0) || defined (SDI12_UART)
  #error "Multiple SDI12 UARTS were defined in SDI12.c Compile aborted"
  #endif
#define USART_Rx_ISR	ISR(USART1_RX_vect)
#define USART_Tx_ISR	ISR(USART1_TX_vect)
#define USART_Port		PORTD
#define tx_pin			PD3
#define break_read		PIND
#define break_pin		PD2
#define PCINT_pin		PCINT26
#define UDRn			UDR1
#define UCSRnA			UCSR1A
#define RXCn			RXC1	// USART Receive Complete
#define TXCn			TXC1	// USART Transmit Complete
#define UDREn			UDRE1 	// USART Data Register Empty
#define	FEn				FE1		// Frame Error
#define	DORn			DOR1	// Data OverRun
#define	UPEn			UPE1	// USART Parity Error
#define	U2Xn			U2X1	// Double the USART Transmission Speed
#define MPCMn			MPCM1	// Multi-processor Communication Mode
#define	UCSRnB			UCSR1B
#define	RXCIEn			RXCIE1	// RX Complete Interrupt Enable n
#define	TXCIEn			TXCIE1	// TX Complete Interrupt Enable n
#define	UDRIEn			UDRIE1	// USART Data Register Empty Interrupt Enable n
#define	RXENn			RXEN1	// Receiver Enable n
#define	TXENn			TXEN1	// Transmitter Enable n
#define	UCSZn2			UCSZ12	// Character Size n
#define	RXB8n			RXB81	// Receive Data Bit 8 n
#define	TXB8n			TXB81	// Transmit Data Bit 8 n
#define	UCSRnC			UCSR0C
#define	UMSELn1			UMSEL11 // mode select
#define	UMSELn0			UMSEL10	// mode select
#define	UPMn1 			UPM11 	// parity select
#define	UPMn0			UPM10	// parity select
#define	USBSn			USBS1	// stop it select
#define	UCSZn1 			UCSZ11 	// bit select
#define	UCSZn0			UCSZ10	// bit select
#define	UCPOLn 			UCPOL1	// Clock Polarity
#define UBRRn			UBRR1

#elif defined (SDI12_UART) //ALERT RECHECK ALL!!!
//NB - some of the register and/or bit names MAY need to be modified for some processors
//The given register and bit names are for Mega328P
  #if defined (SDI12_UART0) || defined (SDI12_UART1)
  #error "Multiple SDI12 UARTS were defined in SDI12.c Compile aborted"
  #endif
#define USART_Rx_ISR	ISR(USART_RX_vect)
#define USART_Tx_ISR	ISR(USART_TX_vect)
#define USART_Port		PORTD
#define tx_pin			PD1
#define break_read		PIND
#define break_pin		PD0
#define PCINT_pin		PCINT16
#define PCINT_bank		???
#define UDRn			UDR1
#define UCSRnA			UCSR1A
#define RXCn			RXC1	// USART Receive Complete
#define TXCn			TXC1	// USART Transmit Complete
#define UDREn			UDRE1 	// USART Data Register Empty
#define	FEn				FE1		// Frame Error
#define	DORn			DOR1	// Data OverRun
#define	UPEn			UPE1	// USART Parity Error
#define	U2Xn			U2X1	// Double the USART Transmission Speed
#define MPCMn			MPCM1	// Multi-processor Communication Mode
#define	UCSRnB			UCSR1B
#define	RXCIEn			RXCIE1	// RX Complete Interrupt Enable n
#define	TXCIEn			TXCIE1	// TX Complete Interrupt Enable n
#define	UDRIEn			UDRIE1	// USART Data Register Empty Interrupt Enable n
#define	RXENn			RXEN1	// Receiver Enable n
#define	TXENn			TXEN1	// Transmitter Enable n
#define	UCSZn2			UCSZ12	// Character Size n
#define	RXB8n			RXB81	// Receive Data Bit 8 n
#define	TXB8n			TXB81	// Transmit Data Bit 8 n
#define	UCSRnC			UCSR0C
#define	UMSELn1			UMSEL11 // mode select
#define	UMSELn0			UMSEL10	// mode select
#define	UPMn1 			UPM11 	// parity select
#define	UPMn0			UPM10	// parity select
#define	USBSn			USBS1	// stop it select
#define	UCSZn1 			UCSZ11 	// bit select
#define	UCSZn0			UCSZ10	// bit select
#define	UCPOLn 			UCPOL1	// Clock Polarity
#define UBRRn			UBRR1
#endif

//Specify the port and the pin for the SDI-12 external hardware transmit enable
#define SDI12_TxEn_Pin 	PD7
#define SDI12_TxEn_Port	PORTD
#define SDI12_TxEn_DDR	DDRD

//Specify the DISABLED state for the SDI-12 external hardware transmit
//only ONE of the following two defines
#define SDI12_TxEn_DisHi
//#define SDI12_TxEn_DisLo

#if defined (SDI12_TxEn_DisHi) && defined (SDI12_TxEn_DisLo)
#error "Both SDI12_TxEn_DisHi and SDI12_TxEn_DisLo have been defined - Compile aborted!"
#endif

//define the operations to enable or disable the external SDI-12 transmit interface
#if defined SDI12_TxEn_DisLo
#define	SDI12_TxEn 		SDI12_TxEn_Port |= (1<<SDI12_TxEn_Pin)
#define SDI12_TxDis 	SDI12_TxEn_Port &= ~(1<<SDI12_TxEn_Pin)
#elif defined SDI12_TxEn_DisHi
#define	SDI12_TxEn 		SDI12_TxEn_Port &= ~(1<<SDI12_TxEn_Pin)
#define SDI12_TxDis 	SDI12_TxEn_Port |= (1<<SDI12_TxEn_Pin)
#endif

//Macros to turn the USART receiver on and off
//And enable/disable the receive-complete interrupt.
#define SDI12_Rx_on	 	UCSRnB |= (1<<RXENn)
#define SDI12_Rx_off 	UCSRnB &= ~(1<<RXENn)
#define SDI12_Rx_IntOn	UCSRnB |= (1<<RXCIEn)
#define SDI12_Rx_IntOff	UCSRnB &= ~(1<<RXCIEn)

//Macros to turn the USART transmitter int on and off
#define SDI12_Tx_on	 	UCSRnB |= (1<<TXENn)
#define SDI12_Tx_off 	UCSRnB &= ~(1<<TXENn)

//Macro to return USART receive error status (non-zero if error)
#define SDI12_Rx_err	UCSRnA &= ( (1<<FEn) | (1<<DORn) | (1<<UPEn) )

//Macros to turn Timer1 on and off
#define SDI12_Tim_off 	TCCR1B &= ~( (1<<CS12) | (1<<CS11) | (1<<CS10) ) //clock off
#define SDI12_Tim_on	TCCR1B |= ( (1<<CS12) | (1<<CS10) ) //on with prescale by 1024
#define SDI12_Tim_rst	TCNT1 = 0	//reset timer1
#define SDI12_Iim_ocr	OCR1A
#define SDI12_Timer		TCNT1		//to read the elapsed time

//Macros to turn break detect on and off
#define SDI12_Brk_on	PCMSK3 |= (1<<PCINT_pin)	//enable break detect
#define SDI12_Brk_off	PCMSK3 &= ~(1<<PCINT_pin)	//disable break detect

//Macro to clear break detect int. NOTE: this clears ALL pending PC3 ints!!
#define SDI12_Brk_clr	PCIFR |= (1<<PCIF3)

//Macros to put the Tx pin in a specific state
#define SDI12_TxMark	USART_Port |= (1<<tx_pin)
#define SDI12_TxSpace	USART_Port &= ~(1<<tx_pin)

//Timer1CompareA values for various functions
//  timer runs at a tick time of 0.256ms (prescale = 1024 with Fclk=4MHz)
//  or 64us (0.064ms) at 16MHz.
//	Read the "_" in or to the right of the number as a decimal point milliseconds
//  The relationship between time and counts is time = counts / (F_CPU/1024)
//  For time in ms and frequency in KHz, counts = Tms * F_CPU/1024/1000 = Tms * F_CPU/(1024000)
//  The MAX time at 16MHz is 4.096 seconds
#define kSDI12_tim100_basic	100UL*F_CPU/1024000		//basic 100ms
#define kSDI12_tim50_basic	50UL*F_CPU/1024000		//basic 50ms
#define kSDI12_tim8_19short 8.19*F_CPU/1024000		//8.19ms, just short of 1 char.
#define kSDI12_tim8_45long	8.45*F_CPU/1024000		//8.45ms, just over 1 char
#define kSDI12_time100_char	(100-8.33)*F_CPU/1024000//100ms less one char time
#define kSDI12_time50_char	(50-8.33)*F_CPU/1024000	//50ms less one char time
#define kSDI12_time10_0  	10UL*F_CPU/1024000		//10.0ms max time from one char det to next
#define kSDI12_interchar	12UL*F_CPU/1024000		//12ms char_to_char max time
#define kSDI12_breakdur		12UL*F_CPU/1024000		//12ms minimum break duration
#define kSDI12_time85		85UL*F_CPU/1024000		//85ms window following SRQ
#define kSDI12_time200		200UL*F_CPU/1024000		//200ms failsafe for break after SQR
#define kSDI12_time1000		F_CPU/1024				//1 second wait time

//PRIVATE variable declarations
char sdi12_TxBuf[40];			//sdi12 transmit buffer
char volatile sdi12_RxBuf[10];	//sdi12 receive buffer - 7 obvious command chars, max (inc \r\n)
uint8_t volatile sdi12_TxIndx;	//index for sdi12_TxBuf
uint8_t volatile sdi12_RxIndx;	//index for sdi12_RxBuf
uint8_t volatile sdi12_Status;	//sdi12 interface status
char	volatile sdi12_addr[5];	//array of valid SDI-12 addresses
char	volatile sdi12_cmdchr;	//the command character
uint8_t volatile sdi12_seccnt;	//seconds counter
uint8_t volatile sdi12_ticcnt;	//counts 50ms ticks
uint8_t volatile sdi12_flags;	//action flags
uint8_t sdi12_query_count;		//rotating index for query responses
uint8_t volatile sdi12_waitSRQ_cnt; //pass counter during wait_SRQ
//char * volatile sdi12_DataPtr;	//pointer to data message
uint8_t volatile sdi12_RxAddr;	//received ASCII address
uint8_t volatile sdi12_NumAddr;	//numeric version of ASCII rx'd addr
char * volatile sdi12_SendPtr;	//pointer to data being transmitted
uint8_t volatile sdi12_RxData;	//holds conditions of previous measure command

#ifdef SDI12_DEBUG
	uint8_t volatile sdi12_debug[80];	//DEBUG ONLY REMOVE LATER
	uint8_t volatile sdi12_dbgidx;	//DEBUG ONLY REMOVE LATER
#endif

//Flags declarations for use with sdi12_flags
//these can be used for setting, clearing, and masking
#define kSDI12_RxClr	0		//sdi12_flags cleared
#define kSDI12_RxCmd	(1<<0)	//command received
#define kSDI12_ProcCmd  (1<<1)	//command has been processed, ready to send
#define kSDI12_ProcErr  (1<<2)	//command has been processed, invalid command
#define kSDI12_CRCFlg	(1<<3)	//command contained CRC reaquest
#define kSDI12_CmdM		(1<<4)	//command M
#define kSDI12_CmdV		(1<<5)	//command V
#define kSDI12_CmdC		(1<<6)	//Command C
#define kSDI12_Abort	(1<<7)	//any data request

//Flags declarations for use with sdi12_RxData
//these can be used for setting, clearing, and masking
//Low 4 are for the "n" in "aDn\n\r" or corresponding "R" command
//High 4 are for flags
//
#define kSDI12_RxD  	(1<<4)	//D received - one-time data value
#define kSDI12_RxR		(1<<5)  //R received - one of series of continuous values

//The constant applied to variable sdi12_action
#define kSDI12_ActNil		0x00
#define kSDI12_ActSavAddr	0x10

#ifdef SDI12_DEBUG
	//debug contants - remove when debugging is complete
	//There are up to 16 entry-exit flags. All will have
	//MSBit set to distinguish from states
	#define kSDI12_RxEnter		0x80
	#define kSDI12_RxErrExit	0x81
	#define kSDI12_RxExit		0x82
	#define kSDI12_TxEnter		0x83
	#define kSDI12_TxExit		0x84
	#define kSDI12_TmrEnter		0x85
	#define kSDI12_TmrExit		0x86
	#define kSDI12_PCIEnter		0x87
	#define kSDI12_PCIExit		0x88
	#define kSDI12_ParseEnter	0x89
	#define kSDI12_ParseMExit	0x8A
	#define kSDI12_ParseExit	0x8B
	#define kSDI12_TaskExit		0x8C
	#define KSDI12_SendExit		0x8D
	#define kSDI12_ParseAbExit	0x8E	//abort parsed

	//other identifiers
	#define kSDI12_EscChrIdx	0xC1
	#define kSDI12_EscURTErr	0xC2
	#define kSDI12_EscURTChr	0xC3
	#define kSDI12_EscTxIdx		0xC4
	#define kSDI12_EscTxChr		0xC5
	//END DEBUG
#endif

//PRIVATE constants used with sdi12_Status
#define kSDI12_StatIdle		0
#define kSDI12_StatTstBrk	1
#define kSDI12_StatTstMrk	3
#define kSDI12_StatWaitAct	4
#define kSDI12_StatWaitChr	6
#define kSDI12_StatRxChr	7
#define kSDI12_StatSndMrk	8
#define kSDI12_StatSndResp	9
#define kSDI12_StatSendSRQ	10
#define kSDI12_StatDChr		11
#define kSDI12_StatWaitSRQ	12
#define kSDI12_StatABrk		13
#define kSDI12_StatWaitDBrk	14
#define kSDI12_StatWaitDBrk2 15
#define kSDI12_StatDTst		16
#define kSDI12_StatDBrk		17

//PRIVATE function declarations - API declarations in sdi12.h
  void sdi12_RxBufClr( void );     //fills RxBuf with 0's
  void sdi12_txportinit( void );   //sets the SDI12 transmit enable direction
  void sdi12_cmd_parse( void );    //called from dotask
  void sdi12_send_atttn( char a ); //called from cmd_parse()
  void sdi12_send_atttnn( char a );//called from cmd_parse()
  void sdi12_send_abort_response( char a ); //called from cmd_parse()
  void sdi12_send_m_atttn( char a ); //called from cmd_parse()
  void sdi12_send_wireless( char a, char *msg, uint8_t control ); //called from cmd_parse()

//PROGMEM statements
char *sdi12_ttt PROGMEM = "000";	//the ttt string for command responses
char *sdi12_info PROGMEM = "13AZ_USGSXB10HS001"; //info response: llccccccccmmmmmmvvv

//******************************************************
// USARTn_RX_vect
//
// This block defines an SDI12 serial receive input ISR
//	depending upon whether SDI12_UART0 or SDI12_UART1 was
//	defined. Initially, there is no bounds checking on
//	receive buffer size.
//
// 	The first character is tested to see if it represents
// 	the address of one of my wireless devices. The valid address
//  depends on whether this is an "original" command or expected
//  to be a "D" following an "M" command. If the latter, then the
//  address MUST match the previously received one. If not valid,
//	it bails and goes back to break detection. If valid,
//	the address is put into the first slot of the buffer.
//	Subseqent characters are also put into the buffer until
//	a terminator 'LF' is received.
//
//	I/O Registers modified or accessed:
//		UDRn
//		PCMSK3
//		UCSRnB
//
//	Functions or macros "called"
//		none
//
//	Variables modified or accessed
//		sdi12_BrkStat 	 	global PRIVATE: break processing state variable
//		sdi12_Status  		global PRIVATE: sdi12 interface status
//		sdi12_addr[1]  		global PRIVATE: address array
//		sdi12_RxBuf[]		global PRIVATE: receive buffer
//		sdi12_RxIndx		global PRIVATE: receive buffer index
//******************************************************
USART_Rx_ISR
  	{
  	//error flags HAVE to be read before reading UDR!!!!
	uint8_t mask = ((1<<FEn)|(1<<DORn)|(1<<UPEn) );
  	uint8_t uart_err = UCSRnA & mask;	//mask only the Rx error bits
  	char temp = UDRn & 0x7F;			//mask the high bit to zero
  	uint8_t J;							//address scan loop index
	uint8_t ctemp;	//+JDW 04062001 numeric address

	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK
  	//First, the ISR identifier & state
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_RxEnter;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_Status;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}

	//Then the error identifier	and the error value
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_EscURTErr;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = uart_err;		//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
	//Then the char identifier	and the char value
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_EscURTChr;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = temp;			//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
	//END DEBUG BLOCK
	#endif

	//FEn - uart-specific Frame Error
	//DORn - uart-specific Data OverRun
	//UPEn - uart-specific USART Parity Error

  	if ( uart_err) {
  		SDI12_Rx_off;		//turn off uart rx
  		SDI12_Tim_rst;		//reset timer
  		SDI12_Tim_on;		//timer on
		SDI12_Brk_on;		//turn on break detect
		sdi12_flags = kSDI12_RxClr;
		sdi12_RxData = kSDI12_RxClr;	//clear the RxData - back to new command
		//sdi12_RxAddr stays same, will get reset on new command
  		if ( uart_err & (1<<FEn) ) {
			//Possible break start. Treat as if leading edge of new break
			SDI12_Iim_ocr = kSDI12_time100_char; //50ms less 1 character time
			sdi12_Status = kSDI12_StatTstBrk;
  			}
  		else { //all the other errors
			SDI12_Iim_ocr = kSDI12_tim8_19short; //1 char time
			sdi12_Status = kSDI12_StatTstMrk;
  			} //end other errors
 		#ifdef SDI12_DEBUG
  		//DEBUG BLOCK
		if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {			//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = kSDI12_RxErrExit;	//DEBUG ONLY
			sdi12_dbgidx ++;								//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = sdi12_Status;		//DEBUG ONLY
			sdi12_dbgidx ++;								//DEBUG ONLY
			}
  		//END DEBUG BLOCK
		#endif
  		return;			//early exit
  	} //end if uart_err

  	//finally, no serial error!
	switch ( sdi12_Status )
		{
		//case kSDI12_StatIdle:
			//break;
		//case kSDI12_StatTstBrk:
			//break;
		//case kSDI12_StatTstMrk:
			//break;
		case kSDI12_StatWaitAct:
		//Here for FIRST character after break+mark.
		//Only valid character is "?" or one of MY addresses!
		//
			if (temp == '?')
				ctemp = 0xff;		//invalid for numeric address
			else if (temp <= '9')
				ctemp = temp - '0'; //+JDW 04062010 numeric value
			else if (temp <= 'Z')
				ctemp = temp - 'A' + 0x0A;
			else if (temp <= 'z')
				ctemp = temp - 'a' + 0x34;
			else {
				SDI12_Rx_off;		//turn off uart rx
				SDI12_Brk_clr;		//clear any pending pin change int
				SDI12_Brk_on;		//turn on break detect
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				#ifdef SDI12_DEBUG
				//DEBUG BLOCK exit flag and status value
				if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
					sdi12_debug[sdi12_dbgidx] = kSDI12_RxExit; 	//DEBUG ONLY
					sdi12_dbgidx ++;							//DEBUG ONLY
					sdi12_debug[sdi12_dbgidx] = sdi12_Status; 	//DEBUG ONLY
					sdi12_dbgidx ++;							//DEBUG ONLY
				}
				//END DEBUG BLOCK
				#endif
				return; //early exit
			}

  			if ( ctemp < 0xff) { //not '?'
  				for ( J=0 ; J<number_of_nodes; J++ ) {
  					if (ctemp == node_ids[J]) { //*JDW 04062010s
  						sdi12_NumAddr = ctemp;	//the numeric address
  						sdi12_RxAddr = temp;	//the ASCII address
  						ctemp = 0xfe;	//valid match - J and temp are enough from there
  						break;
					} //end if temp
  				  //end for ..
  				} //end if ctemp == 0xff
  			}
  			//from here, ctemp = 0xff if '?', 0xfe if address match, < 0xfe if no match
			if ( ctemp >= 0xfe ) {//its valid address or "?"
				sdi12_RxBufClr( );			//clear RxBuf
				sdi12_RxBuf[0] = temp;		//save char just received
				sdi12_RxIndx = 1;			//index of NEXT char
				SDI12_Tim_rst;					//reset the timer
				SDI12_Iim_ocr = kSDI12_interchar; 	//max time 1 char detect to next
				sdi12_Status = kSDI12_StatWaitChr;
				}
			else {					//error, or invalid as address or something other than "?"
				SDI12_Rx_off;		//turn off uart rx
				SDI12_Brk_clr;		//clear any pending pin change int
				SDI12_Brk_on;		//turn on break detect
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			break;

		case kSDI12_StatWaitChr:
		//Here on all characters AFTER the first character of all commands.
			//if (temp == '\n') { //command terminator
			if (temp == '!') { //*JDW 10062010 command terminator
				sdi12_RxBuf[sdi12_RxIndx] = temp;	//save the received char
				sdi12_RxIndx ++;
				SDI12_Rx_off;		//turn off uart rx
				SDI12_Brk_off;		//turn off break detect
				SDI12_Tim_rst;		//reset the timer
				SDI12_TxMark;		//set tx out to mark with TXENn disabled
									//this controls the output state
				SDI12_TxEn;			//enable transmit buffer
				SDI12_Iim_ocr = kSDI12_tim8_45long;	//1 char time pause time to 1st response char
				SDI12_Tim_on;		//timer on
				//no change here to sdi12_RxData - that happens in parser
				sdi12_flags |= kSDI12_RxCmd; //signal new command rxd
				sdi12_Status = kSDI12_StatSndMrk;
				//NB: the response message will be generated in
				//sdi12_cmd_parse() while in kSDI12_SndMrk
				}
			else { //valid without error - put in buffer and prepare for next char
				sdi12_RxBuf[sdi12_RxIndx] = temp;	//save the received char
				sdi12_RxIndx ++;
				SDI12_Tim_rst;					//reset the timer
				SDI12_Iim_ocr = kSDI12_interchar; //1 char time + allowed mark time
				SDI12_Tim_on;		//timer on
				//no state change
				}
			break;

		//case kSDI12_StatRxChr:
			//break;
		//case kSDI12_StatSndMrk:
			//break;
		//case kSDI12_StatSndResp:
			//break;
		//case kSDI12_StatWaitSRQ:
			//break;
		//case kSDI12_StatSendSRQ:
			//break;
		//case kSDI12_StatWaitDBrk:
			//break;
		//case kSDI12_StatDTst:
			//break;
		//case kSDI12_StatDBrk:
			//break;
		case kSDI12_StatDChr:
		//here on first char of D command - HAS to match previous address
		//rx error cases have already been handled
			if (temp == sdi12_RxAddr) {  //this address matches the M address - this rejects "?"
				sdi12_RxBufClr( );			//clear RxBuf
				sdi12_RxBuf[0] = temp;		//save char just received
				sdi12_RxIndx = 1;			//index of NEXT char
				SDI12_Tim_rst;					//reset the timer
				SDI12_Iim_ocr = kSDI12_interchar; 	//max time 1 char detect to next
				sdi12_Status = kSDI12_StatWaitChr;
				}
			else {
				SDI12_Rx_off;		//turn off uart rx
				SDI12_Brk_clr;		//clear any pending pin change int
				SDI12_Brk_on;		//turn on break detect
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
			}
			break;
		}

		#ifdef SDI12_DEBUG
  		//DEBUG BLOCK exit flag and status value
		if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = kSDI12_RxExit; 	//DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = sdi12_Status; 	//DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			}
		//END DEBUG BLOCK
		#endif

  	} //end  USART_Rx_ISR

//******************************************************
// USARTn_TX_vect
//
// This block defines an SDI12 serial transmit output ISR
//	depending upon whether SDI12_UART0 or SDI12_UART1 was
//	defined. Initially, there is no bounds checking. On
//	each interrupt, the next character in the transmit
//	buffer is put into UDRn unless the character is zero.
//  If it is zero, it simply exits with no action. This
//	leaves UDRn empty and no new interrupt will happen.
//
//	Note that no end-of-line characters are added, here.
//	If required, they must be added when the characters
//	are added to the transmit buffer.
//
//	The process is started by writing the first character
// 	of the buffer into UDRn and setting the index to 1.
//
//  When complete, set the kSDI12_StatTxDone bit in sdi12_Status
//	When a new message is placed into the buffer, this bit
//	must be cleared.
//******************************************************
USART_Tx_ISR
    {
	PINB |= _BV(PB0); 
	PINB |= _BV(PB0); 
	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK Tx enter flag and status value
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_TxEnter;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_Status; 	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
	//END DEBUG BLOCK
	#endif

	char temp; //here so it can be captured by debug block
	if (sdi12_SendPtr == 0)
		temp = 0xff;
	else
		temp = *sdi12_SendPtr;

	#ifdef SDI12_DEBUG
	//DEBUG BLOCK - the trnamitted character
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {	//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_EscTxChr;		//DEBUG ONLY
		sdi12_dbgidx ++;						//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = temp;		//DEBUG ONLY
		sdi12_dbgidx ++;						//DEBUG ONLY
		}
	//END DEBUG BLOCK
	#endif

	switch ( sdi12_Status )
		{
		//case kSDI12_StatIdle:
			//break;
		//case kSDI12_StatTstBrk:
			//break;
		//case kSDI12_StatTstMrk:
			//break;
		//case kSDI12_StatWaitAct:
			//break;
		//case SDI12_StatWaitChr:
			//break;
		//case SDI12_StatRxChr:
			//break;
		//case kSDI12_StatSndMrk:
			//break;

		case kSDI12_StatSndResp:
			if ( temp != 0 ) { //NOT the last char!
				UDRn = temp;
				sdi12_SendPtr ++;	//next character
				//sdi12_TxIndx ++;
				//no state change
				}
			//then, the following cases are ALL with temp == 0
			//and they are all 'last char' cases
			else if (sdi12_flags & kSDI12_CmdM) {
				if (sdi12_RxData & kSDI12_RxD) { //then it was a response to a data request
					sdi12_DataPtr = 0;
					sdi12_flags = kSDI12_RxClr; //this and next do not happen with a continuus measurement!
					sdi12_RxData = kSDI12_RxClr;//or with multiple D requests for a single measure request.
					SDI12_Tim_rst;		//reset the timer
					SDI12_Tim_off;		//timer on
					SDI12_Rx_off;		//receiver off
					SDI12_TxDis;		//disable the tx buffer
					SDI12_Brk_on;		//turn on break detect
					SDI12_Brk_clr;		//clear any old ints
					SDI12_Tx_off;		//turn the transmitter off
					sdi12_Status = kSDI12_StatIdle;
					PINB |= _BV(PB0); 
					PINB |= _BV(PB0); 
				}
				else { //not a data request, just the 'M'
					SDI12_Iim_ocr = kSDI12_tim100_basic; 	//100ms
					sdi12_waitSRQ_cnt = 0;	//init the pass counter
					SDI12_Tim_rst;	//reset the timer
					SDI12_Tim_on;		//timer on
					SDI12_Rx_off;		//receiver off
					SDI12_TxDis;		//disable the tx buffer
					SDI12_Brk_on;		//turn on break detect
					SDI12_Brk_clr;		//clear any old ints
					SDI12_Tx_off;		//turn the transmitter off
					//TAG1 - end of response transmit
					sdi12_Status = kSDI12_StatWaitSRQ;
					}
				}
			else { //cmdchr is anything else - then go to idle
				SDI12_TxDis;		//disable the buffer
				SDI12_Tx_off;		//turn off uart rx
				SDI12_Brk_clr;	//clear any pending pin change int
				SDI12_Brk_on;		//turn on break detect
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			break;

		//case kSDI12_StatWaitSRQ:
		//	break;

		case kSDI12_StatSendSRQ:
			if ( temp != 0 ) { //its not end of the string
				UDRn = temp;
				sdi12_SendPtr ++;	//next character
				//sdi12_TxIndx ++;
				//no state change
				}
			else { //it IS end of the string
				SDI12_Iim_ocr = kSDI12_time85; 	//85ms window for break OR D command address *JDW 25062010
				SDI12_Tim_rst;		//reset the timer
				SDI12_Rx_on;		//receiver off
				SDI12_Rx_IntOff;	//disable the receive interrupt
				SDI12_TxDis;		//disable the buffer
				SDI12_Tx_off;		//turn off uart rx
				SDI12_Brk_clr;		//clear any pending pin change int
				SDI12_Brk_on;		//turm pm break detect
				//TAG3 - SRQ completed
				sdi12_Status = kSDI12_StatWaitDBrk;
				}
			break;

		//case kSDI12_StatWaitDBrk:
			//break;
		//case kSDI12_StatDTst:
				//break;
		//case kSDI12_StatDBrk:
				//break;
		}

		#ifdef SDI12_DEBUG
  		//DEBUG BLOCK - Tx exit flg and state
		if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = kSDI12_TxExit;	//DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = sdi12_Status;	//DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			}
		//END DEBUG BLOCK
		#endif

    } //end USART_Tx_ISR
//******************************************************
// TIMER1_COMPA_vect
//
// This block handles general timing actions. What it
// does depends on the state.
//******************************************************
ISR(TIMER1_COMPA_vect)
	{

	uint8_t temp;

	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK - timer entry flag and status value
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_TmrEnter; //DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_Status;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
  	//DEBUG BLOCK
	#endif

	switch ( sdi12_Status )
		{
		case kSDI12_StatTstBrk:
		//here on end of 100ms break timeout. Assume line fault.
			SDI12_Tim_off;				//turn off the timer
			SDI12_Brk_clr;				//clear all pending PCI3 ints
			SDI12_Brk_on;				//turn on pin change int.
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;			//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		case kSDI12_StatTstMrk:
		//here on 8.19ms timeout during a post-break mark
		//start bit is next, so enable UART receive, but keep the timer
		//ON to make sure this "mark" does not last too long (in a fault state).
		//Wait 100ms for timeout.
			SDI12_Tim_rst;
			SDI12_Iim_ocr = kSDI12_tim100_basic;//100ms
			SDI12_Tim_on;
			SDI12_Rx_on;				//turn on UART rcvr
			temp = UCSRnA;				//read to clear error flags
			SDI12_Brk_off;				//turn off PCInt
			sdi12_Status = kSDI12_StatWaitAct;
			break;

		case kSDI12_StatWaitAct:	//end of 100ms timeout
		//A whole 100ms has passed since end of break.
		//Return to idle (probable line fault).
			SDI12_Tim_off;				//turn off the timer
			SDI12_Rx_off;				//turn off uart receive
			SDI12_Brk_clr;				//clear all pending PCI3 ints
			SDI12_Brk_on;				//turn on pin change int.
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;	//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		case kSDI12_StatWaitChr:
		//Here on 10ms timeu after receiving FIRST charactr - line fault!
			#ifdef SDI12_DEBUG
			temp = UCSRnA;				//TEMP DEBUG
			#endif
			SDI12_Tim_off;				//turn off the timer
			SDI12_Rx_off;				//turn off uart receive
			SDI12_Brk_clr;				//clear all pending PCI3 ints
			SDI12_Brk_on;				//turn on pin change int.
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;	//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		//case SDI12_StatRxChr:
			//break;

		case kSDI12_StatSndMrk:
		//here on completion of the 1 char delay beteen command and response
		//start transmission of the response messaage. TxBuffer already enabled.
			SDI12_Tim_off;			//timer off
			SDI12_Tx_on;			//ready UART to transmit
			UDRn = *sdi12_SendPtr;	//first character
			sdi12_SendPtr ++;		//point to next character
			//UDRn = sdi12_TxBuf[0];	//1st tx chr into UDR; int already on
			//sdi12_TxIndx = 1;		//TxBuf[] index of NEXT char
			sdi12_Status = kSDI12_StatSndResp;
			break;

		case kSDI12_StatWaitSRQ:
		//waiting for notification that data is ready and an SRQ needs to be sent
			sdi12_waitSRQ_cnt ++;	//count this pass
			SDI12_Tim_rst;			//reset the timer
			if (sdi12_waitSRQ_cnt < 10*kSDI12_MeasureWait) { //not timeout yet
				if (sdi12_DataPtr != 0) { //then its time to send SRQ!
					//NB: sdi12_DataPtr points to msg from wireless and is
					// not the general transmit pointer, sdi12_SendPtr.
					SDI12_Tim_off;			//turn off the timer
					SDI12_Tx_on;			//+JDW 06/08/2010 ready UART to transmit
					SDI12_TxEn;				//+JDW 06/08/2010turn on tx buffer
					//the SRQ message follows
					sdi12_TxBuf[0] = sdi12_RxAddr;
					sdi12_TxBuf[1] = '\r';	//carriage return
					sdi12_TxBuf[2] = '\n'; 	//line feed char
					sdi12_TxBuf[3] = 0;		//terminator
					sdi12_SendPtr = sdi12_TxBuf;	//point to the SRQ string
					UDRn = *sdi12_SendPtr;	//send first character
					sdi12_SendPtr ++;		//point to next character
					//UDRn = sdi12_TxBuf[0];	//load first character
					//sdi12_TxIndx = 1;		//TxBuf[] index of NEXT char
					//TAG2 - msg rx'd from wireless, ready to send SRQ
					sdi12_Status = kSDI12_StatSendSRQ;
				}//end if not send SRQ
				//else don't do anything more than count and reset
			} //end if NOT timed out
			else { //it HAS timed out so bail
				#ifdef SDI12_DEBUG
				temp = UCSRnA;				//TEMP DEBUG
				#endif
				SDI12_Rx_off;				//turn off uart receive
				SDI12_Tim_off;				//turn off the timer
				SDI12_Brk_clr;				//clear all pending PCI3 ints
				SDI12_Brk_on;				//turn on pin change int.
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}  //end if timed out
			break;

		//case kSDI12_StatSndResp:
			//break;

		//case kSDI12_StatSendSRQ:
			//break;

		case kSDI12_StatWaitDBrk:
			//here on timeout while waiting for leading edge of D break after sending SRQ.
			//This is the end of the window for D command without a break.
			SDI12_Rx_off;				//turn off uart receive
			temp = UDRn;				//read to clear any pending int
			SDI12_Rx_IntOn;				//restore rx int
			SDI12_Tim_rst; 				//reset the timer
			SDI12_Iim_ocr = kSDI12_time200;	//200ms failsafe
			//PCI stays ON
			sdi12_Status = kSDI12_StatWaitDBrk2;
			break;

		case kSDI12_StatWaitDBrk2:
			//here on fault timeout. Return to idle
			SDI12_Tim_off;			//turn off the timer
			SDI12_Brk_clr;			//clear all pending PCI3 ints
			SDI12_Brk_on;			//turn on pin change int.
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;	//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		case kSDI12_StatABrk:
		//here on timeout while waiting for trailing edge of abort break. failure.
		case kSDI12_StatDBrk:
		//here on timeout while waiting for trailing edge of D break. Failure
			SDI12_Tim_off;			//turn off the timer
			SDI12_Brk_clr;			//clear all pending PCI3 ints
			SDI12_Brk_on;			//turn on pin change int.
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;	//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		case kSDI12_StatDTst:
		//Timeout while waiting for termination of a break. Line fault.
			SDI12_Tim_off;			//turn off the timer
			SDI12_Brk_clr;			//clear all pending PCI3 ints
			SDI12_Brk_on;			//turn on pin change int.
			temp = UDRn;			//read to clear any pending int
			SDI12_Rx_IntOn;
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;	//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		case kSDI12_StatDChr:
			//Timeout while waiting for first char of D  command. Line fault.
			SDI12_Rx_off;				//turn off uart receive
			SDI12_Tim_off;			//turn off the timer
			SDI12_Brk_clr;			//clear all pending PCI3 ints
			SDI12_Brk_on;			//turn on pin change int.
			sdi12_flags = kSDI12_RxClr;
			sdi12_RxData = kSDI12_RxClr;	//reset to new command
			sdi12_Status = kSDI12_StatIdle;
			break;

		//case kSDI12_StatDBr2:
			//break;

		} //end switch ( sdi12_Status )

		#ifdef SDI12_DEBUG
  		//DEBUG BLOCK timer exit and status
		if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = kSDI12_TmrExit; //DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = sdi12_Status; 	//DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			}
	  	//END DEBUG BLOCK
		#endif

	} // end ISR(TIMER1_COMPA_vect)
//******************************************************
// PCINT3_vect
//
// This block manages pin change interrupts for break
//	detection. On interuppt, the RxDn state is read.
//	Break state is low, idle is high. It then acts,
//	depending on the current state of the break state
//	machine.
//
// See state machine description for details!
//
//******************************************************
ISR(PCINT3_vect)
	{
	uint8_t temp;
	uint16_t deltatime = SDI12_Timer;	//get elapsed time as quickly as possible

	//temp is non-zero if the RxDn pin is high (rising edge)
	temp = break_read & (1<<break_pin);

	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK PCI enter flag and status
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_PCIEnter; //DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_Status;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
  	//END DEBUG BLOCK
	#endif

	switch ( sdi12_Status )
		{
		case kSDI12_StatIdle:
		//here on edge detect in idle state. If a
		//rising edge, ignore and stay idle. If
		//falling edge, turn on break timer and begin
		//break validation.
		if (!temp) { //ignore if rising pin change - same sdi12_Status!
			SDI12_Tim_rst;					//reset the timer
			#ifdef SDI12_DEBUG
			PORTA |= (1<<PA0);			//SET PA0 high DEBUG ONLY
			#endif
			SDI12_Iim_ocr = kSDI12_tim100_basic;	//100ms
			TIMSK1 |= (1<<OCIE1A);		//enable timer int
			SDI12_Tim_on;				//turn on the timer
			sdi12_Status = kSDI12_StatTstBrk;
			}
			break;

		case kSDI12_StatTstBrk:
		//Here on edge detect during break test. It
		//must be a rising edge (end of break) since
		//StatTstBrk was only entered on falling edge.
		//If the measured duration is too short, no break
		//and go back to idle. If long enough, its a valid
		//break, so continue with mark testing.
			if (deltatime < kSDI12_breakdur) { 		//too short for break
				#ifdef SDI12_DEBUG
				//temp = TCNT1;  //temp DEBUG - now in deltatime
				PORTA &= ~(1<<PA0);				//SET PA0 low DEBUG ONLY
				#endif
				SDI12_Tim_off;					//turn off the timer
				SDI12_Brk_clr;				//clear all pending PCI3 ints
				SDI12_Brk_on;				//turn on pin change int.
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			else { //valid for break
				#ifdef SDI12_DEBUG
				PORTA &= ~(1<<PA0);				//SET PA0 low DEBUG ONLY
				//temp = TCNT1;				//temp debug ONLY now in deltatime			//break detected
				#endif
				SDI12_Tim_rst; 						//reset the timer
				SDI12_Iim_ocr = kSDI12_tim8_19short;	//for mark testing
				SDI12_Tim_on;
				SDI12_Brk_on;
				sdi12_flags = kSDI12_RxClr;//turn on pin change int
				sdi12_RxData = kSDI12_RxClr;	//clear data receive control flag
				sdi12_Status = kSDI12_StatTstMrk;
				}
			break;

		case kSDI12_StatTstMrk:
		//Here on edge detected before the end of a valid mark interval.
		//Could be a new break.
			SDI12_Tim_rst;							//reset
			SDI12_Iim_ocr = kSDI12_tim100_basic;	//standard 100ms
			SDI12_Tim_on;
			sdi12_Status = kSDI12_StatTstBrk;
			break;

		//case kSDI12_StatWaitAct: edge detect is off
			//break;
		//case kSDI12_StatWaitChr:
			//break;
		//case SDI12_StatRxChr:
			//break;
		//case kSDI12_StatSndMrk:
			//break;
		//case kSDI12_StatSndResp:
			//break;
		//case kSDI12_StatSendSRQ:
			//break;

		case kSDI12_StatWaitSRQ:
		//here on edge detected (possible abort break) while waiting for SRQ request
			if (!temp) { //ignore if rising pin change - same sdi12_Status!
				#ifdef SDI12_DEBUG
				PORTA |= (1<<PA0);				//SET PA0 high DEBUG ONLY
				#endif
				SDI12_Tim_rst;					//reset the timer
				SDI12_Iim_ocr = kSDI12_tim100_basic;	//100ms
				sdi12_Status = kSDI12_StatABrk;//state for testing length of abort request break
				}
			break;

		case kSDI12_StatABrk:
		//here on a second edge detected while waiting for SRQ - test for break
		//leave break detect on, in either case
			if (deltatime < kSDI12_breakdur) { 		//too short for break
				#ifdef SDI12_DEBUG
				//temp = TCNT1;  //temp DEBUG - replaced by deltatime
				PORTA &= ~(1<<PA0);		//SET PA0 low DEBUG ONLY
				#endif
				SDI12_Tim_off;			//turn off the timer
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			else { //valid for break	- no mark, just abort and wait - timer was on
				sdi12_flags |= (kSDI12_RxCmd | kSDI12_Abort);	//flag the abort
				#ifdef SDI12_DEBUG
				PORTA &= ~(1<<PA0);				//SET PA0 low DEBUG ONLY
				//temp = TCNT1;	//temp debug ONLY
				#endif
				SDI12_Tim_rst; 				//reset the timer
				SDI12_Iim_ocr = kSDI12_tim8_19short;	//8.19ms min mark
				sdi12_Status = kSDI12_StatTstMrk;
				}
			break;

		case kSDI12_StatWaitDBrk:
		//Here,SRQ sent.edge detected as start of D-message break
			if (!temp) { //ignore if rising pin change - same sdi12_Status!
				SDI12_Tim_rst; 				//reset the timer
				SDI12_Iim_ocr = kSDI12_time200;	//200ms failsafe
				//break detect stays on
				//uart receive stays on
				//rx int stays off
				//TAG4 - edge detected inside no-break window
				sdi12_Status = kSDI12_StatDTst; }
			else { //rising edge - error so back to idle
				SDI12_Tim_off;			//turn off the timer
				SDI12_Rx_off;			//turn off uart receive
				temp = UDRn;			//read to clear any pending ints
				SDI12_Rx_IntOn;			//restore rx int
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			break;

		case kSDI12_StatDBrk:
		//here on a second edge detected after sending SRQ - test for break
		//leave break detect on, in either case
			if (deltatime < kSDI12_breakdur) { 		//too short for break
				#ifdef SDI12_DEBUG
				PORTA &= ~(1<<PA0);		//SET PA0 low DEBUG ONLY
				#endif
				SDI12_Tim_off;			//turn off the timer
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			else { //valid for break	- no mark, wait for mark
				#ifdef SDI12_DEBUG
				PORTA &= ~(1<<PA0);			//SET PA0 low DEBUG ONLY
				#endif
				SDI12_Tim_rst; 				//reset the timer
				SDI12_Iim_ocr = kSDI12_tim8_19short;	//8.19ms min mark
				sdi12_Status = kSDI12_StatTstMrk;
				}
			break;

		case kSDI12_StatWaitDBrk2:
			if (!temp) { //good if falling, otherwise bad
				SDI12_Tim_rst;							//reset the timer
				SDI12_Iim_ocr = kSDI12_tim100_basic;	//100ms
				SDI12_Tim_on;							//turn on the timer
				sdi12_Status = kSDI12_StatDBrk;
				}
			else { //faulty edge direction
				SDI12_Tim_off;			//turn off the timer
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
				}
			SDI12_Brk_on;
			break;

		case kSDI12_StatDTst:
		//here on a second edge detect after leading edge of break or character
		//what happens next depends on the timer count value in deltatime
			if (deltatime < kSDI12_tim8_19short) { //not break; probably a character
				SDI12_Tim_rst; 				//reset the timer
				SDI12_Iim_ocr = kSDI12_time10_0;	//10ms failsafe for 8.33ms full char
				//leave rx on, PCI off.
				SDI12_Brk_off;
				sdi12_Status = kSDI12_StatDChr;
			}
			else if (deltatime < kSDI12_breakdur) { //invalid duration
				//too long for a character, too short for break
				//break detect stays on
				SDI12_Rx_off;			//turn off uart receive
				SDI12_Tim_off;			//turn off the timer
				sdi12_flags = kSDI12_RxClr;
				sdi12_RxData = kSDI12_RxClr;	//reset to new command
				sdi12_Status = kSDI12_StatIdle;
			}
			else { //its equal or greater than breakdur, so a valid break!
				//mark is next
				SDI12_Iim_ocr = kSDI12_tim8_19short;	//for mark testing
				SDI12_Tim_on;
				sdi12_flags = kSDI12_RxClr;
				SDI12_Brk_on;					//turn on pin change int
				sdi12_RxData = kSDI12_RxClr;				//clear data receive control flag
				sdi12_Status = kSDI12_StatTstMrk;
			}
			temp = UDRn;		//clear any pending ints
			SDI12_Rx_IntOn;		//restore rx int
			break;

		//case kSDI12_StatDChr:
		//	break;

		//case kSDI12_StatDBrk:
			//break;
		}

		#ifdef SDI12_DEBUG
  		//DEBUG BLOCK - PCI exit flag and status value
		if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = kSDI12_PCIExit; //DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = sdi12_Status; 	//DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			}
	  	//END DEBUG BLOCK
		#endif

   	}   //end ISR(PCINT3_vect)

//******************************************************
// void sdi12_init( void ) - PUBLIC
//
// This function initializes the SDI-12 serial interface.
//	The initialization includes the define-specified UART,
//	the pin change interrupt for break detection, and the
//	receive and transmit buffers.
//
// UARTn will be set up for 1200 baud, e-7-1. The uart RX
//  & int are NOT turned on until later. Each time the
//  receive buffer is cleared, it is filled with 0's. So,
//  simply stuffing characters will result in a proper string
//
// Both PCI24 for RxD0 and PCI26 for RxD1 generate a PCI3 interrupt.
//  PCInt is on for PCI3.
//
// Timer1 is initialized to prescale = 0 (counter off), WGM Mode = 0.
//  Compare int is off and will be enabled as needed.
//
// I/O Registers modified:
//		UCSRnB
//		UCSRnC
//		UBRRn
//		PCICR
//		PIND = break_read
//
//	Functions or macros "called"
//		sdi12_RxBufClr()
//
//	Variables modified or accessed
//		sdi12_status	  	global PRIVATE: sdi-12 state variable
//		sdi12_TxIndx 		global PRIVATE: index to transmit buffer
//******************************************************
void sdi12_init( void ) //-PUBLIC
  	{

	#ifdef SDI12_DEBUG
	uint8_t j;			//DEBUG ONLY REMOVE LATER
	#endif

  	//init UARTn
  	//don't write anything to UDRn;
  	//UCSRnA default is OK;
  	UCSRnB = (1<<RXCIEn) | (1<<TXCIEn); //(UCSZn2=0 for 7 data bits
  	//(UMSELn1:0 = 0 for async uart)(USBSn = 0 for 1 stop)
  	UCSRnC = (1<<UPMn1) | (1<<UCSZn1);  		//Even parity, 7 data bits
	UBRRn = 0x0fff & ((F_CPU/(16*1200)) - 1);	//1200 baud not double rate

	//set pin change input as input
	DDRD &= ~(1<<break_pin);
	//set up the pin change interrupt
	PCICR |= (1<<PCIE3);	//enable PCI3 interrupt bank
	SDI12_Brk_on;	//enable break detect

	sdi12_TxIndx = 0;
	sdi12_RxBufClr();

	SDI12_TxEn_DDR |= (1<<SDI12_TxEn_Pin);
	SDI12_TxDis;
	sdi12_flags = kSDI12_RxClr;
	sdi12_RxData = kSDI12_RxClr;	//reset to new command
	sdi12_Status = kSDI12_StatIdle;
	sdi12_msg_signal = 0xff;		//not a valid address
	sdi12_SendPtr = 0;				//default nil pointer - don't need this since vars default to zero

	//Init Timer1
	//output compare outputs are disconnected.
	TCCR1A = 0; //default 0 is OK, WGM=0 (normal), compare match outputs all off
	TCCR1B = 0; //default 0 is OK, off, normal WGM mode, no force compare
	//TCNT1 does not get initialized
	//OCR1A set later
	//OCR1B does not get initialized because it is not used
	TIMSK1 = 0;	//enable interrupt on CompareA when needed; no overflow.
	TIFR1 = ( (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1) ); //clear all int flags

	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK	- initialize
	DDRA |= (1<<PA0);			//SET PA0 out
	PORTA &= ~(1<<PA0);			//SET PA0 low
	for ( j = 0 ; j < sizeof(sdi12_debug); j ++)	//DEBUG ONLY
		sdi12_debug[j] = 0;		//DEBUG ONLY
	sdi12_dbgidx = 0;			//DEBUG ONLY
  	//END DEBUG BLOCK
	#endif

	//init control variables
	sdi12_query_count = 0;		//round-robin query response index
	sdi12_action = kSDI12_ActNil;
	sdi12_flags = 0;
	sdi12_Status = kSDI12_StatIdle;
	//
  	} //end sdi12_init( void )

//******************************************************
// void sdi12_disable( void ) - PUBLIC
//
// This function disaables the SDI-12 serial interface.
//	The disable includes the define-specified UART,
//	the pin change interrupt for break detection, and the
//	defined timer.
//
// The Tx and Rx buffers will be cleared with sdi12_init()
//	is called to restore the interface.
//
// Both PCI24 for RxD0 and PCI26 for RxD1 generate a PCI3 interrupt.
//  PCInt is on for PCI3.
//
// Timer1 is initialized to prescale = 0 (counter off),.
//  Compare int is off..
//
// I/O Registers modified:
//		UCSRnB
//		UCSRnC
//		UBRRn
//		PCICR
//		PIND = break_read
//
//	Variables modified or accessed
//		sdi12_status	  	global PRIVATE: sdi-12 state variable
//		sdi12_TxIndx 		global PRIVATE: index to transmit buffer
//******************************************************

	void sdi12_disable( void )
 	{

  	//disable  UARTn
	UCSRnB &= ~( (1<<RXCIEn) | (1<<TXCIEn) | (1<<RXENn) | (1<<TXENn) );

	//disaable PCI3
	PCICR &= ~(1<<PCIE3);	//disaable PCI3
	SDI12_Brk_off;			//disable break detect

	SDI12_TxDis;			//turn off the hardware tx buffer IC

	//disable Timer1
	//output compare outputs are disconnected.
	SDI12_Tim_off;
	TIMSK1 = 0;	//all timer ints off.
	//TIFR1 will be cleared on init

	//init control variables - just in case
	sdi12_action = kSDI12_ActNil;
	sdi12_flags = 0;
	sdi12_Status = kSDI12_StatIdle;
	//
  	} //end sdi12_disable( void )

//******************************************************
// void sdi12_enable( void ) - PUBLIC
//
// This function enaables the SDI-12 serial interface.
//	The enableable includes the define-specified UART,
//	the pin change interrupt for break detection, and the
//	defined timer.
//
// The Tx and Rx buffers will be cleared with sdi12_init()
//	is called to restore the interface.
//
// Both PCI24 for RxD0 and PCI26 for RxD1 generate a PCI3 interrupt.
//  PCInt is on for PCI3.
//
// Timer1 is initialized to prescale = 0 (counter off),.
//  Compare int is off..
//
// I/O Registers modified:
//		PCICR
//		PIND = break_read
//		TIMSK1
//
//	Variables modified or accessed
//		sdi12_status	  	global PRIVATE: sdi-12 state variable
//		sdi12_TxIndx 		global PRIVATE: index to transmit buffer
//******************************************************

	void sdi12_enable( void )
 	{

  	//enable  UARTn after break detected

	//enable PCI3
	PCICR |= (1<<PCIE3);	//enable PCI3 interrupt bank
	SDI12_Brk_on;	//enable break detectt

	//turn on the hardware tx buffer only when time to transmit

	//Timer1 stays off until break is detected.
	SDI12_Tim_off;

	//re-init control variables
	sdi12_action = kSDI12_ActNil;
	sdi12_flags = 0;
	sdi12_Status = kSDI12_StatIdle;;

	//clear the receive buffer
	sdi12_RxBufClr( );
	//
  	} //end sdi12_ensable( void )

//******************************************************
// void sdi12_RxBufClr( void ) - PRIVATE
//
//  clears the sdi12 serial receive buffer by filling
//  it with zereos and resets the buffer index to zero.
//
//  Since it is filled with zeros, incoming characters
//	can be simply stuffed into the buffer. The result
//	will always be a valid string so long as the buffer
//  is not over-run.
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//		none
//
//	Variables modified or accessed
//		sdi12_RxIndx	global PRIVATE
//		sdi12_RxBuf[] 	global PRIVATE
//		j 				local
//******************************************************
void sdi12_RxBufClr( void ) //-PRIVATE
	{
	uint8_t j;

	for ( j = 0 ; j < sizeof(sdi12_RxBuf); j ++)
		sdi12_RxBuf[j] = 0;
	sdi12_RxIndx = 0;
	} //sdi12_RxBufClr( void )

//******************************************************
//void sdi12_dotask( void ) PUBLIC
// This is one of the public API functions of the sdi12
// 	interface. It is called regularly as part of the main
//	loop of the host module.
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//		sdi12_cmd_parse()
//
//	Variables modified or accessed
//		sdi12_flags		global PRIVATE
//		sdi12_Status	global PRIVATE
//
//******************************************************
void sdi12_dotask( void ) //PUBLIC
	{
	if ( sdi12_flags & kSDI12_RxCmd )   //then a complete command is in the buffer
		sdi12_cmd_parse( );
		//with sdi12_flags == kSDI12_RxCmd, there are two states,
		//sdi12_Status = kSDI12_StatSndMrk (initial command) and
		//sdi12_Status = kSDI12_StatSendSRQ;
	//do it this way to respond in the same pass after coming out of sdi12_cmd_parse()
	//sdi12_flags = kSDI12_ProcCmd if there is a response to send
	if ( sdi12_flags & kSDI12_ProcCmd ) {
		switch ( sdi12_Status ) {
		case kSDI12_StatSndMrk:
			//timer is already started. That will trigger the
			//transmissiokn of the response message when it expires
			break;

		} //end switch

		sdi12_flags &= ~kSDI12_ProcCmd;		//clear the flag
	} //end if kSDI12_ProcCmd

	//do it this way to respond in the same pass after comming
	//out of sdi12_cmd_parse()
	sdi12_flags &= ~kSDI12_ProcErr;		//clear the flag


}  //end sdi12_dotask( void )

//******************************************************
//void sdi12_cmd_parse( void ); //PRIVATE
//call from sdi12_dotask() IF kSDI12 bit of sdi12_flags
//
//	First character will always be '?' or address.
//  In the case of a "D" command, the address has already
//	been validated as the same as the preceding "M" command.
//
//  The last character is always 'LF'
//
//  If this is the first pass on a new command, then
//	sdi12_RxData = 0 and sdi12_Flags = kSDI12_RxCmd.
//
//  If the "M" command
//  was aborted, sdi12_RxData & kSDI12_Abort > 0. If a
//  request for CRC'd data was received in the "M" command.
//  sdi12_RxData & kSDI12_CRC > 0. If an "M" command of
// 	any kind was received, sdi12_RxData & kSDI12_RxM > 0
//  If either kSDI12_CRC or kSDI12_Abort bits are set,
//  kSDI12_RxM must also be set. Similar pattern is available
// 	for C/R pair.
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//		sdi12_RxBufClr( void )
//		sdi12_send_atttn( char a )
//		sdi12_send_atttnn( char a )
//		sdi12_send_wireless( char a, char *msg )
//
//	Variables modified or accessed
//		query_count		local static
//		sdi12_RxIndx	global PRIVATE
//		sdi12_RxBuf[] 	global PRIVATE
//		sdi12_TxIndx	global PRIVATE
//		sdi12_TxBuf[]	global PRIVATE
//		sdi12_status	global PRIVATE
//		sdi12_flags		global public
//
//******************************************************
  void sdi12_cmd_parse( void ) //PRIVATE called from dotask
  {

	uint8_t temp;		//general use var esp for reconstructing addresses
	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK parse enter flag and RxData
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_ParseEnter; //DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_RxData;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
  	//END DEBUG BLOCK
	#endif

	//here only if sdi12_flags & kSDI12_RxCmd is true.
	sdi12_flags &= ~(kSDI12_RxCmd);  //clear the RxCmd flag

	//There are three generalized cases:
	// sdi12_flags & kSDI12_Abort is true must handle an abort. Ignore the
	//  status of sdi12_RxData (and clear it when done)

	// sdi12_flags & kSDI12_Abort AND sdi12_RxData & (kSDI12_RxR | kSDI12_RxD) is false
	//  which indicates a new command

	// sdi12_flags & kSDI12_Abort AND sdi12_RxData & (kSDI12_RxR | kSDI12_RxD) is true
	//  which indicates the command second of a command pair.

	if (sdi12_flags & kSDI12_Abort) {  //abort condition received
		sdi12_flags = kSDI12_RxClr;
		sdi12_RxData = 0;
		sdi12_send_abort_response( sdi12_RxAddr ); //kSDI12_ProcCmd bit handled in function
		#ifdef SDI12_DEBUG
		//DEBUG BLOCK abort exit flag
		if (sdi12_dbgidx < sizeof(sdi12_debug)) {		//DEBUG ONLY
			sdi12_debug[sdi12_dbgidx] = kSDI12_ParseAbExit; //DEBUG ONLY
			sdi12_dbgidx ++;							//DEBUG ONLY
			}
		//END DEBUG BLOCK
		#endif
		return;	//done
	}

	//from here, on, the message will be parsed, then checked against
	// the kSDI12_RxR and kSDI12_RxD bits in sdi12_RxData on a case-by-
	// case basis

  	if (sdi12_RxBuf[0] == '?') { //then its a query command
	//-------------------------------------------------
	//Each time a query request is received, a response is
	//sent for one of the possible addresses, and
	//increments the index for the next query. This allows
	//it to provide,.round-robin, all of the addresses.
	//NOTE: this DOES NOT properly handle addresses above 9!!!
	//-------------------------------------------------
  		temp = node_ids[sdi12_query_count];
  		if (temp < 10) {
  			sdi12_TxBuf[0] = node_ids[sdi12_query_count] + '0';
  		}
  		else if (temp < 36) { // 10 corresponds to 'A' & 35 corresponds to 'Z'
  			sdi12_TxBuf[0] = node_ids[sdi12_query_count] - 10 + 'A';
  		}
  		else if (temp < 62) { //36 corresponds to 'a' & 61 corresponds to 'z'
  			sdi12_TxBuf[0] = node_ids[sdi12_query_count] - 36 + 'a';
  		}
  		else { //its an error - send nothing and bail out
			sdi12_flags = kSDI12_ProcErr;	//error - clear all other flags
  			sdi12_RxData = kSDI12_RxClr;
  			return;
  		}
		sdi12_TxBuf[1] = '\r'; 	//carriage return
		sdi12_TxBuf[2] = '\n'; 	//line feed char
		sdi12_TxBuf[3] = 0;		//string terminator
		sdi12_query_count ++;			//for next query
		if (sdi12_query_count >= number_of_nodes)	//limit max value to number_of_nodes-1
			sdi12_query_count = 0;
		sdi12_SendPtr = sdi12_TxBuf;	//point to the string
		//sdi12_TxIndx = 0;
		sdi12_flags |= kSDI12_ProcCmd; //ready to send
		} //end first char is '?'

	else {//the first char is the address & second MUST be command.
		//First char as a valid local address has been verified in
		//USART_Rx_ISR. It is NOT a '?' which has been handled above
		sdi12_RxAddr = sdi12_RxBuf[0];		//address of command

		switch ( sdi12_RxIndx-1 ) {//number of rxd chars - ignoring terminator '!'
		//-------------------------------------------------
		//
		//First addressed case has only 2 characters. And,
		//there is only one valid instance: ack. Since it
		//has ONLY 2 characters and the first is verified
		//as address and the last is verified '!', there
		//is no need to check anything more. No error is
		//possible since both (all) characters have already
		//been checked.
		//
		//-------------------------------------------------
		case 1:	//2 characters - ack only
			if (sdi12_RxBuf[0] == 'A'){
				sdi12_TxBuf[0] = sdi12_RxAddr;
				sdi12_TxBuf[1] = '\r'; 	//carriage return
				sdi12_TxBuf[2] = '\n'; 	//line feed char
				sdi12_TxBuf[3] = 0;		//string terminator
				sdi12_SendPtr = sdi12_TxBuf;	//point to the string
				//sdi12_TxIndx = 0;
				sdi12_flags |= kSDI12_ProcCmd; //ready to send - preserve any C/R flags
			}
			else	//error!
				sdi12_flags = kSDI12_ProcErr;	//error - clear all other flags
			//don't set kSDI12_ProcCmd unless it was a valid 'A' command
			//preserve contents of sdi123_RxData
			break;
		//-------------------------------------------------
		//
		//Second addressed case has 3 characters. Only the
		// the command char needs checking because the first
		// (address) and the last ('!') have already been
		// verified. So, the only possible error is incorrect
		// command character. This command version CANNOT request
		// a CRC nor can it specify a measurement number.
		//
		//Command 'M' has a response string of "atttn" and 'C'
		// has a response of "atttnn".
		//
		//Commands 'I' and 'V' responses are node/sensor dependent.
		//
		//-------------------------------------------------
		case 2: //3 chars, command = {I,M,V,C} //A,D,R not until 4 chars
			if ( sdi12_RxBuf[1] == 'I' ) {
				sdi12_TxBuf[0] = sdi12_RxAddr; //
				strcpy_P ( sdi12_TxBuf+1, (PGM_P)sdi12_info ); //copy 17 info chars
				sdi12_TxBuf[18] = '0'; 	//temp node addr filler ALERT
				sdi12_TxBuf[19] = '0'; 	//temp node addr filler ALERT
				sdi12_TxBuf[20] = '0'; 	//temp node addr filler ALERT
				sdi12_TxBuf[21] = '0'; 	//temp node addr filler ALERT
				sdi12_TxBuf[22] = '\r'; //carriage return
				sdi12_TxBuf[23] = '\n'; //line feed char
				sdi12_TxBuf[24] = 0;	//string terminator
				sdi12_SendPtr = sdi12_TxBuf;	//point to the string
				sdi12_flags |= kSDI12_ProcCmd; //ready to send
				if (sdi12_flags & (kSDI12_CmdM | kSDI12_CmdV) ) { //then bad command sequence -
					//clear the remnants of the prior M or V command
					//leqve CmD flag
					sdi12_flags &= ~(kSDI12_CmdM | kSDI12_CmdV| kSDI12_CRCFlg);	//clear the prior M
					}
				}

			else if ( sdi12_RxBuf[1] == 'M' ) {
				sdi12_send_m_atttn( sdi12_RxAddr );
				sdi12_flags |= (kSDI12_CmdM | kSDI12_ProcCmd);	//M without CRC
				sdi12_flags &= ~(kSDI12_CmdC | kSDI12_CmdV);	//clear any remnants
				}

			else if ( sdi12_RxBuf[1] == 'V' ) {
				sdi12_send_atttn( sdi12_RxAddr );
				sdi12_flags |= (kSDI12_CmdV | kSDI12_ProcCmd);	//V without CRC
				sdi12_flags &= ~( kSDI12_CmdM);		//retain any C, clear any M
			}

			else if ( sdi12_RxBuf[1] == 'C' ) {
				sdi12_send_atttnn( sdi12_RxAddr );
				sdi12_flags |= (kSDI12_CmdC | kSDI12_ProcCmd);	//C without CRC
				sdi12_flags &= ~( kSDI12_CmdM| kSDI12_CmdV);	//clear any M or V
			}

			else //its an error
				sdi12_flags = kSDI12_ProcErr;	//error - clear all other flags

			sdi12_RxData = kSDI12_RxClr;		//nothing for here
			break;

		//-------------------------------------------------
		//
		//Third addressed case has 4 characters. Only the
		// the command char & 3rd char need checking because the first
		// (address) and the last ('!') have already been
		// verified. So, possible errors include incorrect
		// command character and incorrect modifiers
		//
		//Command 'MC' or "Mn" has a response string of "atttn" and
		// 'CC' or "Cn" has a response of "atttnn". "Rn" has a response
		// of data. The "ttt" strings are PROGMEM strings
		// defined at compile. PROGMEM implementation costs extra code,
		// but it allows a single defined string to function in all
		// locations. This, in turn, means that only one location has
		// to be edited to uniformly change all of the instances.
		//
		//Data strings are sensor dependent.
		//
		//-------------------------------------------------
		case 3: //4 chars, command = {A,M,D,C,R}
			if ( sdi12_RxBuf[1] == 'A' ) {
				//NB - this is INVALID in wireless system!!!
				if ( sdi12_RxAddr == sdi12_addr[0] ) {
					sdi12_addr[0] = sdi12_RxAddr;
					sdi12_action = kSDI12_ActSavAddr;	//save chan0 to NVRAM!
					}
  				else if ( sdi12_RxAddr == sdi12_addr[1] ) {
					sdi12_addr[1] = sdi12_RxAddr;
					sdi12_action = kSDI12_ActSavAddr + 1;	//save chan1 to NVRAM!
					}
  				else if ( sdi12_RxAddr == sdi12_addr[2] ) {
					sdi12_addr[2] = sdi12_RxAddr;
					sdi12_action = kSDI12_ActSavAddr + 2;	//save chan2 to NVRAM!
					}
  				else if ( sdi12_RxAddr == sdi12_addr[3] ) {
					sdi12_addr[3] = sdi12_RxAddr;
					sdi12_action = kSDI12_ActSavAddr + 3;	//save chan3 to NVRAM!
					}
  				else if ( sdi12_RxAddr == sdi12_addr[4] ) {
					sdi12_addr[4] = sdi12_RxAddr;
					sdi12_action = kSDI12_ActSavAddr + 4;	//save chan4 to NVRAM!
					}
				//send the response
				//DO NOT do any address change in wireless system
				sdi12_TxBuf[0] = sdi12_RxAddr;
				sdi12_TxBuf[1] = '\r'; 	//carriage return
				sdi12_TxBuf[2] = '\n'; 	//line feed char
				sdi12_TxBuf[3] = 0;		//string terminator
				sdi12_SendPtr = sdi12_TxBuf;	//point to the string
				//sdi12_TxIndx = 0;
				sdi12_flags |= kSDI12_ProcCmd; 	//ready to send
				sdi12_RxData = kSDI12_RxClr;	//nothing to add
				} //end 'A'

			else if ( sdi12_RxBuf[1] == 'M' ) {
				//4 char M must be followed by 'C' or {'1'-'9'}
				//same response, either case
				if ( sdi12_RxBuf[2] == 'C') {
					sdi12_send_m_atttn( sdi12_RxAddr );
					sdi12_flags = ( kSDI12_CRCFlg | kSDI12_CmdM | kSDI12_ProcCmd );	//set the M with CRC flag
					sdi12_RxData = kSDI12_RxClr;	//nothing to add
				}
				else if ( ( sdi12_RxBuf[2] >= '1') && ( sdi12_RxBuf[2] <= '9') ) {
					sdi12_flags = ( kSDI12_CmdM | kSDI12_ProcCmd );	//M without CRC
					sdi12_RxData = sdi12_RxBuf[2] - '0';			//store n
				}
				else //its an error
					sdi12_flags = kSDI12_ProcErr;	//error
					sdi12_RxData = kSDI12_RxClr;
				} //end "MC" or "Mn"

			else if ( sdi12_RxBuf[1] == 'C' ) {
				//4 char C must be followed by 'C' or {'1'-'9'}
				if ( sdi12_RxBuf[2] == 'C') {
					sdi12_send_m_atttn( sdi12_RxAddr );
					sdi12_flags = ( kSDI12_CRCFlg | kSDI12_CmdC | kSDI12_ProcCmd );	//set the M with CRC flag
					sdi12_RxData = kSDI12_RxClr;	//nothing to add
				}
				else if ( ( sdi12_RxBuf[2] >= '1') && ( sdi12_RxBuf[2] <= '9') ) {
					sdi12_flags = ( kSDI12_CmdC | kSDI12_ProcCmd );	//M without CRC
					sdi12_RxData = sdi12_RxBuf[2] - '0';			//store n
				}
				else {//its an error
					sdi12_flags = kSDI12_ProcErr;
					sdi12_RxData = kSDI12_RxClr;
					}
				} //end "CC" or "Cn"

			//now, we come to response commands, D after C, M OR V

			else if ( sdi12_RxBuf[1] == 'D' ) {
				//D command MUST be preceded by an C, M, or V
				if (sdi12_flags & (kSDI12_CmdM | kSDI12_CmdC | kSDI12_CmdV)) {
					//4 char D must be followed by {'0'-'9'} that matches the low nibble of sdi12_RxData
					if ( ( sdi12_RxBuf[2] - '0') == (sdi12_RxData & 0x0f) ) {
						sdi12_flags |= kSDI12_ProcCmd;	//this is OK
						sdi12_RxData |= kSDI12_RxD;		//flag as D received
						sdi12_send_wireless( sdi12_RxAddr, sdi12_DataPtr, sdi12_flags );
					}
					else {//its an error
						sdi12_flags = kSDI12_ProcErr;
						sdi12_RxData = kSDI12_RxClr;
						} //end if numeric D
					} //end if preceding M
				else {//not preceded by D,M,V so error
					sdi12_flags = kSDI12_ProcErr;
					sdi12_RxData = kSDI12_RxClr;
					}
				}//end D handler

			else if ( sdi12_RxBuf[1] == 'R' ) {
				//R handler not functional yet
					sdi12_flags = kSDI12_ProcErr;
					sdi12_RxData = kSDI12_RxClr;
				}//end R handler

  			else { //.not one of the valid commands so error
				sdi12_flags = kSDI12_ProcErr;
				sdi12_RxData = kSDI12_RxClr;
  				}
			break; //end of case 3

		//-------------------------------------------------
		//
		//Fourth addressed case has 5 characters. The 3rd character
		// must be 'C" and the fourth must be 'n' (allowable n
		// depends on command). The command char also needs
		// checking. So, possible errors include incorrect
		// command character and incorrect modifiers
		//
		//Command 'MCn' has a response string of "atttn" and
		// 'CCn' has a response of "atttnn". "RCn" has a response
		// of data (but is NOT functional). The "ttt" strings are PROGMEM strings
		// defined at compile. PROGMEM implementation costs extra code,
		// but it allows a single defined string to function in all
		// locations. This, in turn, means that only one location has
		// to be edited to uniformly change all of the instances.
		//
		//Data strings are sensor dependent.
		//
		//-------------------------------------------------
		case 4: //5 chars, command = {M,C,R}
			//All 5 char commands MUST have a 'C' as 3rd character and {1-9}
			//if M or C and {0-9} if R as 4th character.
			//test 3rd char first since this is common to ALL commands
			if ( sdi12_RxBuf[2] == 'C' ) {

				if ( sdi12_RxBuf[1] == 'M' )  {
					if ( ( sdi12_RxBuf[3] >= '1') && ( sdi12_RxBuf[3] <= '9') )  { //valid number range
						sdi12_send_m_atttn( sdi12_RxAddr );
						sdi12_RxData = sdi12_RxBuf[3] - '0';
						sdi12_flags = (kSDI12_CmdM | kSDI12_CRCFlg);	//set the M & CRC flag
						}
					else { //error
						sdi12_flags |= kSDI12_ProcErr;	//error
						sdi12_RxData = kSDI12_RxClr;	//flush it
						}
					} // end "MCn"

				else if ( sdi12_RxBuf[1] == 'C' ) {
					if (( sdi12_RxBuf[3] >= '1') && ( sdi12_RxBuf[3] <= '9')) { //valid number range
						sdi12_send_atttnn( sdi12_RxAddr );
						sdi12_RxData = sdi12_RxBuf[3] - '0';
						sdi12_flags = (kSDI12_CmdC | kSDI12_CRCFlg);	//set the C & CRC flag
						}
					else { //error
						sdi12_flags |= kSDI12_ProcErr;	//error
						sdi12_RxData = kSDI12_RxClr;	//flush it
						}
					} //end "CCn"

				else if ( sdi12_RxBuf[1] == 'R' ) {
					if (( sdi12_RxBuf[3] >= '0') && ( sdi12_RxBuf[3] <= '9')) { //valid number range
						//'R' NOT HANDLED YET!
						//ALERT
						sdi12_flags |= kSDI12_ProcErr;	//error
						sdi12_RxData = kSDI12_RxClr;	//flush it
						}
					else {
						sdi12_flags |= kSDI12_ProcErr;	//error
						sdi12_RxData = kSDI12_RxClr;	//flush it
						}
					} // end "RCn"

				else //its an error
					sdi12_flags |= kSDI12_ProcErr;	//error
					sdi12_RxData = kSDI12_RxClr;	//flush it
				} //end outer if

			else {// RxBuf[2] is not 'C'
				sdi12_flags |= kSDI12_ProcErr;	//error
				sdi12_RxData = kSDI12_RxClr;	//flush it
				}
			break;

		//-------------------------------------------------
		//
		//Fifth addressed case has 6 or more chars. The command
		// character must be "X". Subsequent characters up to
		// the '!' are manufacturer dependent.
		//
		//-------------------------------------------------
		default: //6 or more chars, X only
			if ( sdi12_RxBuf[1] == 'X' ) {
				//ALERT
				//NOT HANDLED YET
				sdi12_flags |= kSDI12_ProcErr;	//error
				sdi12_RxData = kSDI12_RxClr;	//flush it
				}
			else {//NO X so its an error
				sdi12_flags |= kSDI12_ProcErr;	//error
				sdi12_RxData = kSDI12_RxClr;	//flush it
				}
			break;

			} //end switch

		} //end RxBuf[0] == '?'

		sdi12_RxBufClr( );

  	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK parse exit flag and flags
	if (sdi12_dbgidx < sizeof(sdi12_debug)-1) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = kSDI12_ParseExit; //DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_flags;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = sdi12_RxData;	//DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
  	//END DEBUG BLOCK
	#endif

  } //end sdi12_cmd_parse()

//******************************************************
//void sdi12_send_atttn( char a ); //PRIVATE
//call from sdi12_parse_cmd() for
//
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//		none
//
//	Variables modified or accessed

//		sdi12_TxBuf[]	global PRIVATE
//		sdi12_flags		global public
//******************************************************
void sdi12_send_atttn( char a ) 	//PRIVATE called from sdi12_cmd_parse()
    {
    sdi12_TxBuf[0] = a; 	// 'a'
	sdi12_TxBuf[1] = '0'; 	// 1st t
	sdi12_TxBuf[2] = '0'; 	// 2nd t
	sdi12_TxBuf[3] = '0'+kSDI12_MeasureWait; 	//max allowable delay,in seconds
	sdi12_TxBuf[4] = '4';
	sdi12_TxBuf[5] = '\r';	//carriage return
	sdi12_TxBuf[6] = '\n'; 	//line feed char
	sdi12_TxBuf[7] = 0;		//string terminator
	sdi12_SendPtr = sdi12_TxBuf;	//point to the string
    } //end sdi12_send_atttn

 //******************************************************
//void sdi12_send_attnn( char* a ); //PRIVATE
//call from sdi12_parse_cmd() for
//
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//		none
//
//	Variables modified or accessed
//		sdi12_TxBuf[]	global PRIVATE
//		sdi12_flags		global public
//******************************************************
void sdi12_send_atttnn( char a ) 	//PRIVATE called from sdi12_cmd_parse()
    {
    sdi12_TxBuf[0] = a; // 'a'
	sdi12_TxBuf[1] = '0'; //t
	sdi12_TxBuf[2] = '0'; //t
	sdi12_TxBuf[3] = '0'; //t = 0
	sdi12_TxBuf[4] = '0'; //n
	sdi12_TxBuf[5] = '0'; //n = 0
	sdi12_TxBuf[6] = '\r';	//carriage return
	sdi12_TxBuf[7] = '\n'; 	//line feed char
	sdi12_TxBuf[8] = 0;		//string terminator
	sdi12_SendPtr = sdi12_TxBuf;	//point to the string
    } //end sdi12_send_atttnn

 //******************************************************
//void sdi12_send_m_attn( char* a ); //PRIVATE +JDW 06062010
//call from sdi12_parse_cmd() for
//
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//		none
//
//	Variables modified or accessed
//		sdi12_msg_signal global public
//		sdi12_SendPtr	global PRIVATE
//		sdi12_TxBuf[]	global PRIVATE
//		sdi12_flags		global public
//******************************************************
void sdi12_send_m_atttn( char a ) 	//PRIVATE called from sdi12_cmd_parse()
    {
    sdi12_TxBuf[0] = a; // 'a'
	sdi12_TxBuf[1] = '0'; //t
	sdi12_TxBuf[2] = '0'; //t
	sdi12_TxBuf[3] = '1'; //t one second delay
	sdi12_TxBuf[4] = '2'; //n = one value
	sdi12_TxBuf[5] = '\r';	//carriage return
	sdi12_TxBuf[6] = '\n'; 	//line feed char
	sdi12_TxBuf[7] = 0;		//string terminator
	sdi12_SendPtr = sdi12_TxBuf;	//point to the string
	//signal wireless that data is needed - no longer valid
	sdi12_msg_signal = sdi12_NumAddr;	//device numeric address
    } //end sdi12_send_m_atttn

 //******************************************************
//void sdi12_send_abort_response( char* a ); //PRIVATE
// call from sdi12_parse_cmd() when an abort after 'M' has
// been aserted.
//
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//
//	Variables modified or accessed
//		sdi12_TxIndx	global PRIVATE
//		sdi12_TxBuf[]	global PRIVATE
//		sdi12_flags		global public
//		sdi12_DataPtr	global PRIVAE - pointer to data string in RAM
//******************************************************
void sdi12_send_abort_response( char a ) 	//PRIVATE called from sdi12_cmd_parse()
    {
    sdi12_TxBuf[0] = a; // 'a'
	sdi12_TxBuf[1] = '\r';	//carriage return
	sdi12_TxBuf[2] = '\n'; 	//line feed char
	sdi12_TxBuf[3] = 0;		//string terminator
	sdi12_SendPtr = sdi12_TxBuf;	//point to the string
    } //end

//******************************************************
//void sdi12_send_wireless( char a, char *msg, uint8_t control ); //PRIVATE
//
//Called from wireless side in response to a aD! command.
// All previous serial activity has been completed, so
// sdi12_SendPtr CAN be set to the message to be transmitted.
// Transmission begins immediately upon exit.
//
//
//	I/O Registers modified:
//		none
//
//	Functions or macros "called"
//
//	Variables modified or accessed
//		sdi12_msg_signal	global public
//		sdi12_DataPtr		global PRIVATE
//		sdi12_SendData		global PRIVATE
//		sdi12_TxBuf[]		global PRIVATE
//
//******************************************************
void sdi12_send_wireless( char a, char *msg, uint8_t control  ) 	//PRIVATE
	{

	char *StrPtr;	//The CRC scan pointer
	uint16_t CRC;	//the CRC working var
	uint16_t SCRC;	//shifter version of CRC for character assignment
	uint8_t count;	//shift counter for CRC opertion
	char Char1, Char2, Char3;

	sdi12_msg_signal = 0xff; 	//reset it
	if (sdi12_DataPtr == 0 ) {  //then wireless has not set it yet
	    sdi12_TxBuf[0] = a; // 'a'
	    sdi12_TxBuf[1] = '0';
	    sdi12_TxBuf[2] = '0';
	    sdi12_TxBuf[3] = '0';
	    sdi12_TxBuf[4] = '0';
		sdi12_TxBuf[5] = '\r';	//carriage return
		sdi12_TxBuf[6] = '\n'; 	//line feed char
		sdi12_TxBuf[7] = 0;		//string terminator
		sdi12_SendPtr = sdi12_TxBuf;	//point to this buffer
	} //end if empty sdi12_DataPtr

	else { //there must be a wireless message to send
		sdi12_DataPtr = msg; 	//the working data ptr
		*sdi12_DataPtr = a;		//first becomes address
		while (*sdi12_DataPtr > 0) { //scan for the first terminating null
			sdi12_DataPtr++;
			}
		//sdi12_DataPtr now points to first terminator
		//CRC goes here if requested!
		if (control & kSDI12_CRCFlg ) { //then CRC has to be added!
			StrPtr = msg;
			while (*StrPtr != 0) {
				CRC = CRC ^ *StrPtr;
				StrPtr ++;
				for ( count = 0; count < 8; count ++) {
					if (CRC & 0x0001) {
						CRC = CRC / 2;
						CRC = CRC ^ 0xA001;
					}
					else
						CRC = CRC / 2;
				} //end for

			} //end while

			Char3 = 0x40 | (CRC & 0x003F); //'right-most CRC char
			SCRC = CRC / 64; //right shift 6 times
			Char2 = 0x40 | (SCRC & 0x003F); //middle CRC char
			SCRC = SCRC / 64; //right shift 6 times
			Char1 = 0x40 | (SCRC & 0x3F); //left-most CRC char
			*sdi12_DataPtr = Char1;
			sdi12_DataPtr ++;
			*sdi12_DataPtr = Char2;
			sdi12_DataPtr ++;
			*sdi12_DataPtr = Char3;
			sdi12_DataPtr ++;	//now points to location of CR
		}
		//now add the CR/LF
		*sdi12_DataPtr = '\r';	//carriage return
		sdi12_DataPtr ++;
		*sdi12_DataPtr = '\n'; 	//line feed char
		sdi12_SendPtr = msg;	//the start of the data string
	}

	#ifdef SDI12_DEBUG
  	//DEBUG BLOCK parse exit flag and flags
	if (sdi12_dbgidx < sizeof(sdi12_debug)) {		//DEBUG ONLY
		sdi12_debug[sdi12_dbgidx] = KSDI12_SendExit; //DEBUG ONLY
		sdi12_dbgidx ++;							//DEBUG ONLY
		}
  	//END DEBUG BLOCK
	#endif

	//end sdi12_send_wireless
	}
