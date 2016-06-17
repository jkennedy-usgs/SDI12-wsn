/*
//  FlowChart module for Wireless SDI-12
//  Author: 	James Wagner, Oregon Research Electronics
//  Date:		Jul 16, 2010
//  Version:	V1.2
//  Processor:	AVR AtMega644P/V

CONTENTS:
	1. Flow chart Notation

	2. Flow chart (unrolled in places where it loops back upon itself)

	3. Normal_Flow provides a listing of the sequence of states in a normal, error-free M/D transaction.

State names are normally kSDI12_xxxxxx. To save space in the following, the "kSDI12_" preamble is dropped
but should be assumed when cross-referencing code.

Time-sensitive events are shown in their time order. It is assumed that if a shorter-time event does not
"fire", then the next event is available. Thus, a PCI interrupt given as "<8.3ms" is the path that would
be followed if the interrupt occurs less than 8.3ms into the state. If the interrupt happens later than
8.3ms but less than 12ms, it will follow the path marked "<12ms". And so forth. An event in the middle 
of a time interval is simply assumed to happen any time before the end of the interval. Any "block" without
a time duration can be considered "instantaneous" or simply run until the specified action is
complete.

"Next states" are assumed to go to the start of that state block since timers are reset on almost every
transistion.


			 TO   -> Timeout
             PCI+ -> Pin Change Int rising edge
             PCI- -> Pin Change Int falling edge
			 PCI* -> Pin Change Int no sense check

      [wait for start of break]
                StatIdle
 -------------------------------------------------
 |             			forever                  | (no TO)
 -------------------------------------------------
			|					|
			|PCI-	`			|PCI+
			|					|
			|					+------ StatIdle
			|	[wait end of break]
			|      StatTstBrk
			|---------------------------
			|            100ms         |
			+--------------------------|
			       |          |        |TO
				   |PCI*	  |PCI*    |
				   |<12ms     |>12ms   +-----StatIdle
				   |		  |
				   |		  |   [wait end of mark]     
				StatIdle      |     StatTstMrk
                              |---------------------|
                              |      8.19ms         |
                              |---------------------|
                                  |                 |
                                  |PCI*             |TO (possible mark)
                                  |                 |
                                  +---StatTstBrk    | [wait for 1st cmd char]    
                                  					|      StatWaitAct
                                                    |-----------------
                                                    |      100ms     |
                                                    |----------------|
                                                       |             |
                                                       |Rx           |TO
                                                      /\             |
                                            yes     /    \    no     +----StatIdle
                               StatTstBrk --------/  0x00  \-------
                                                  \    ?   /      |
                                                    \    /        |
                                                      \/          |
                                                                 /\
                                                      yes      /    \   no
       +---------------------------------------------------- / valid  \--- StatIdle
       |                                                     \  addr  /    
       |                                                       \  ? /      
       |   [wait for command chrs]                               \/        
       |     StatWaitChr                                                                   
 +---->|----------------------
 |     |       10ms          |
 |     -----------------------
 |                  |        |TO
 |                 /\        +----StatIdle
 |        yes    /    \   no
 |        +-----/ valid \--- StatIdle
 |        |     \  addr /
 |       /\      \  ? /       [send mark]           [send ack]
 | no  /   \ yes   \/          StatSndMrk           StatSndResp      StatWaitSRQ
 +---/  /n?  \ ----+--->|------------------TO  ---------------    ---------------
     \       /     |    |      8.45ms     |--->|    ACK      |--->|     1sec    |
       \    /      |    |------------------    ---------------    ---------------
         \/        |             ^                                  |   |       |
                   |             |                                  |   |PCI-   +-StatIdle
                   |             +----------|                       |   |
                   |    --------------------|                       |   |  [wait end break]
                   +--->| sdi12_cmd_parse() |                       |   |   StatABrk
                        ---------------------                       |   |-----------------
                           (in parallel)           sdi1_DataPtr > 0 |   |   100ms        |
                                                                    |   -----------------|
+-------------------------------------------------------------------+    |      |        |
|                                                                        |PCI*  |PCI*    |TO
|                                                                        |<12ms |>12ms   |
|                                                                        |      |        +--StatIdle
|                                                                   StatIdle    |
|                                                                               |
|                                                                          StatTstMrk
|
| StatSendSRQ
-------------
|   SRQ     | END
-------------
			|
			|    [wait start brk or chr]
			|     StatWaitDBrk
			|---------------------
			|          85ms      |
            ----------------------
              |      |           |
              |PCI-  |PCI+       |TO         [wait start brk]
              |      |           |           StatWaitDBrk2
              |      + StatIdle  |---------------------------
              |                  |           200ms          |
              |                  ----------------------------
              |	    [brk or chr finish]		   | |          |
              |         StatDTst        	   | |	 	 	|
              |--------------------------      | |PCI-	 	|TO
              |            200ms        |      | |	 	 	|
              ---------------------------  	   | |	 	 	+------ StatIdle
                |       |      |        | 	   | |
                |PCI*   |PCI*  |PCI*    |TO    | |     [wait end break]
                |<8.3ms |<12ms |>12ms   |      | |       StatDBrk
                |       |      |     StatIdle  | |-----------------------------
                |    StatIdle  |               | |       100ms                |
                |              +StatTstMrk     | |-----------------------------
                |                              |    |           | PCI* >12ms  |TO
                |   [wait chr finish]          |    |           |             +-- StatTstIdle
                |     StatDChr                 |    |PCI* <12ms +---> StatTstMrk
                |----------------------        |    |
                |       10ms          |        |    +--StatIdle
                -----------------------        |
                        |             |        |PCI+
                        | RX          |TO      |
                        |             |        +- StatIdle
                        |         StatIdle
                       /\
                yes  /   \   no
   StatWaitChar-----/valid\--- StatIdle
                    \ addr/
                     \ ? /
                      \ /



 Normal_Flow: Here is the sequence of events in a normal Measurement/Data command sequence

 1. Initial state is kSDI12_StateIdle

 2. In kSDI12_StatIdle, PCI detects a negative edge marking the start of a break. State becomes kSDI12_StatTstBrk

 3. In kSDI12_StatTstBrk, PCI detects an edge (it must be positive but is not tested because the only possible
    edge after a negative edge must be positive) more than 12ms after entry into the state but less than 100ms.
    The state becomes kSDI12_StatTstMrk.

 4. In kSDI12_StatTstMrk, the timer expires after 8.19ms with NO edge detected. State becomes kSDI12_StatWaitAct

 5. In kSDI12_StatWaitAct, the first (address) character is detected as a valid address for this device by the
 	UART receive ISR. State changes to kSDI12_StatWaitChr.

 6. In kSDI12_StatWaitChr, the UART receive ISR receives an 'M'. State remains same.

 7. in KSDI12_StatWaitChr, the UART receive ISR receives an '!'. State remains same.

 8. in KSDI12_StatWaitChr, the UART receive ISR receives an /r. State remains same.

 9. in KSDI12_StatWaitChr, the UART receive ISR receives an /n. State changes to kSDI12_StatSndMrk.
 	sdi12_flags kSDI12_RxCmd bit is set.

 10. In kSDI12_StatSndMrk, the UART transmit line is turned on and the output is set to the MARK level.
 	 While in this state, the application main() loop has called sdi12_DoTask() which detects the
 	 kSDI12_RxCmd bit in sdi12_flags. This, in turn, causes sdi12_cmd_parse() to be called. The command
 	 is parsed, the kSDI12_RxCmd bit is cleared and kSDI12_ProcCmd is set, as well as kSDI12_CmdM. Further,
 	 sdi12_msg_signal is set to the numeric (NOT the ASCII) version of the received address. The command
 	 sequence number (the "n" in "aMn!") is loaded into the low nibble of sdi12_RxData; in the case of
 	 a "aM!" command, zero is used as the sequence number. The parser calls sdi12_send_m_atttn() which
 	 generates the acknowledgment string pointed to by sdi12_SendPtr.

 11. In kSDI12_StatSndMrk, the timer expires, 8.45ms after the state was entered. The UART transmit output
 	 is turned on, the first character, the ack device address, a, of the transmit buffer (pointed to by
 	 sdi12_SendPtr) is loaded into the UART trasnmit register, and sdi12_SendPtr is incremented so that
 	 it points to the second acknowledgment character. The state is changed to kSDI12_StatSndResp.

 12. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 '0' pointed to by sdi12_SendPtr, (the first ack 't' character) is read from the buffer and loaded into
 	 the UART transmit buffer. sid12_SendPtr is incremented and the state remains unchanged.

 13. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 '0' pointed to by sdi12_SendPtr, (the second ack 't' character) is read from the buffer and loaded into
 	 the UART transmit buffer. sid12_SendPtr is incremented and the state remains unchanged.

 14. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 '1' pointed to by sdi12_SendPtr, (the third ack 't' character) is read from the buffer and loaded into
 	 the UART transmit buffer. sid12_SendPtr is incremented and the state remains unchanged.

 15. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 '1' pointed to by sdi12_SendPtr, (the ack 'n' character) is read from the buffer and loaded into
 	 the UART transmit buffer. sid12_SendPtr is incremented and the state remains unchanged.

 16. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 '1' pointed to by sdi12_SendPtr, (the ack '/r' character) is read from the buffer and loaded into
 	 the UART transmit buffer. sid12_SendPtr is incremented and the state remains unchanged.

 16. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 '1' pointed to by sdi12_SendPtr, (the ack '/r' character) is read from the buffer and loaded into
 	 the UART transmit buffer. sid12_SendPtr is incremented and the state remains unchanged.

 17. In kSDI12_StatSndResp, transmit-complete interrupts are serviced. The next transmit buffer character,
 	 (null) pointed to by sdi12_SendPtr, (ack terminator) is read from the buffer. The UART transmitter
 	 is turned off, the transmit connection to the data bus is disabled, and the state changes to kSDI12_ StatWaitSRQ.

 18. In kSDI12_StatWaitSRQ, a timer generates 100ms "ticks". On each tick, the value of sdi12_DataPtr is tested
  	 to be non-zero. If it IS zero, the wireless side has not yet generated a data message. 8 ticks (800ms) are
  	 allotted for this, to prevent possible collisions between sending an SRQ and the normal host-generated data
  	 request (aD0!) that will come after 1 second. If a non-zero sdi12_DataPtr is detected on or before
  	 the 8th 100ms tick, the UART transmitter is turned on, the transmitter connection to the data bus is
 	 turned on, and the first character of an SRQ message, a, is loaded into the UART transmit register. The state
 	 changes to kSDI12_StatSendSRQ.

 19. In kSDI12_StatSendSRQ, the transmit complete interrupt is serviced and the next SRQ charaacter, /r, is
 	 laoded into the UART transmit buffer. The state does not change.

 20. In kSDI12_StatSendSRQ, the transmit complete interrupt is serviced and the next SRQ charaacter, /n, is
 	 laoded into the UART transmit buffer. The state does not change.

 21. In kSDI12_StatSendSRQ, the transmit complete interrupt is serviced and a null character is detected in
  	 the transmit buffer. When this happens, he UART transmitter is turned off, and the transmit output is
  	 disconnected from the data bus. The state changes to kSDI12_StatWaitDBrk.

 22. In kSDI12_StatWaitDBrk, the host has an 87ms window in which it CAN send the data request (aD!) without
 	 starting with a break. If the delay is longer than 87ms, a break MUST be issued before the data request.
 	 However, the host CAN start with a break within the 87ms window, if it chooses. The first detected edge
 	 will be the same sense for both the start of a break and the beginning of the start bit of a data character.
 	 Both the Pin Change Interrupt AND the UART receiver have to be ON when this state began, though the UART
 	 receiver interrupt enable does not. This results in a state change to kSDI12_StatDTst.

 23. In kSDI12_StatDTst, either a second edge is detected greater than 12ms after the beginning of the state
 	 (signifying a break) or the second edge is detected in less than 8.3ms due to edges within the first
 	 character of the aD0! command. If the first edge is within the first 8.3ms (and it will be much less than
 	 8.3ms, if this is the case), then the UART receiver interrupt enable is turned on and the state changes
 	 to kSDI12_StatDChr to wait for the coming character reception interrupt. Keeping the receiver interrupt
 	 off until this second pin-change is received prevents a bogus RxInt from occurring and needing service
 	 when a real break occurs.

 24  In kSDI23_StatDChr, a character is received.. If a valid character IS detected AND it matches the address
 	 of the preceding "aM!" command, the state changes to kSDI12_StatWaitChr.

 25. In kSDI12_StatWaitChr, the UART receive ISR receives an 'D'. State remains same.

 26. In KSDI12_StatWaitChr, the UART receive ISR receives an '0'. State remains same.

 27. In KSDI12_StatWaitChr, the UART receive ISR receives an !. State remains same.

 28. In KSDI12_StatWaitChr, the UART receive ISR receives an /r. State remains same.

 29. In KSDI12_StatWaitChr, the UART receive ISR receives an /n. State changes to kSDI12_StatSndMrk.
 	 sdi12_flags kSDI12_RxCmd bit is set.

 30. In kSDI12_StatSndMrk, the UART transmit line is turned on and the output is set to the MARK level.
 	 While in this state, the application main() loop has called sdi12_DoTask() which detects the
 	 kSDI12_RxCmd bit in sdi12_flags. This, in turn, causes sdi12_cmd_parse() to be called. The command
 	 is parsed, the kSDI12_RxCmd bit is cleared and kSDI12_ProcCmd is set. If the commend sequence number
 	 (should be zero, from aD0!) matches the value in the low nibble of sdi12_RxData, then it is a
 	 valid data request. Here the response is not an ack, but the actual data. The parser calls
 	 sdi12_send_wireless( char a, char *msg, uint8_t control ). This function constructs the response
 	 string. If sdi12_DataPtr IS zero, then the string is "a0000/r/n" to indicate that no data is available.
 	 If sdi12_DataPtr is NOT zero, then the data string constructed by the wireless side is expected
 	 to have a dummy (not-null) character in the first slot and be be terminated by 6 nulls. If a
 	 CRC was requested, then the three ASCII CRC characters are written into the first 3 null spaces.
 	 Whether a CRC has been added or not, an /r/n sequence is written into the first of the remaining
 	 null positions.

 31. In kSDI12_StatSndMrk, the timer expires, 8.45ms after the state was entered. The UART transmit output
 	 is turned on, the first character of the transmit buffer (pointed to by sdi12_SendPtr) is loaded into
 	 the UART trasnmit register, and sdi12_SendPtr is incremented so that it points to the second
 	 character. The state is changed to kSDI12_StatSndResp.

 32. In kSDI12_StatSndResp, transmit-complete interrupts are serviced, Each time one happens, a new character
  	 is read from the transmit buffer with sdi12_SendPtr and sdi12_SendPtr is then incremented. The character
  	 is tested for null (zero). If it is NOT zero, it is loaded into the UART transmitter buffer and the state
  	 remains unchanged. This process is repeated until a zero character IS read from the transmit buffer. At
  	 this point, the UART transmitter is turned off, the transmit connection to the data bus is disabled. If
  	 the kSDI12_CmdM bit was set in sdi12_flags, then both sdi12_flags and sdi12_RxData are cleared and state
  	 returns to kSDI12_StatIdle.

 */
