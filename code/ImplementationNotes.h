/*
//  ImplementationNotes module for Wireless SDI-12
//  Author: 	James Wagner, Oregon Research Electronics
//  Date:		Sep 4, 2010
//  Version:	V1.2
//  Processor:	AVR AtMega644P/V
 *
 *
 * The following notes apply to V1.2 - See DEBUGGING later in this file.
 * Search for keyword IMPORTANT for changes from previous versions
 *
 * X command handling is NOT implemented. See   void sdi12_cmd_parse( void )
 *
 * V command handling is NOT implemented. See   void sdi12_cmd_parse( void )
 *
 * C command handling is NOT implemented. See   void sdi12_cmd_parse( void )
 * 	in particular, C abort and handling other commands after a C are not functional.
 *
 * R command handling is NOT implemented. See   void sdi12_cmd_parse( void )
 *
 * Segmented data responses have NOT been implemented. This means several things:
 * 1. ONLY aM and ad0 are acceptable
 * 2. Data is limited to 35 characters in the value field
 * 3. The response to an aM! command MUST be a0011
 *
 * The data string returned by the wireless side must be terminated in 6 null
 * characters, /0. This provides space for 3 CRC characters and CR+LF AND
 * leaves a terminating null for end identification. If there is no CRC, then
 * there are 4 terminating null characters following CR+LF and the first null
 * is recognized as the "terminator".
 *
 * The first character of the data string is a dummy. The device ASCII address
 * replaces this before sending.
 *
 * Addressing is somewhat non-standard due to requirements as a wireless bridge
 * and the need to support several wireless devices. This software supports up to
 * 5 addresses. It responds to each of these addresses, uniquely. Wireless network
 * addresses are stored in the array node_ids[]. It is the responsibility of the
 * implementer to assign these values. It is expected that the first number_of_nodes
 * addresses are valid. number_of_nodes is initialized to zero and must be set when
 * new addresses are added to node_ids[]. It is expected that all of the first
 * number_of_nodes entries in node_ids[] represent valid addresses.
 *
 * IMPORTANT: Wireless addresses map to SDI-12 addresses according to the following rules:
 *
 * 	SDI-12		Wireless
 *	'0'			0x00
 *	...			...
 *	'9'			0x09
 *	'A'			0x0A
 *	...			...
 *	'Z'			0x33
 *	'a'			0x34
 *  ...			...
 *  'z'			0x65

 * Address changes are NOT allowed. This is a constraint of the wireless bridge nature
 * of this device. SDI-12 addresses map directly to wireless system addresses and there
 * is no obvious way to allow the SDI-12 address to shift while maintaining the
 * corresponding wireless address.
 *
 * One of the challenges in implementing Concurrent or Continuous measurements is managing
 * the 5 addresses independently. There are other challenges, but this one is near the
 * top of the list.
 *
 * The mechanism for requesting and sending data is as follows:
 *
 * 	1. The SDI-12 protocol stack receives an aM! or aMC! command, where 'a' matches one of
 *     the valid addresses for the bridge. When a match occurs, 'a' is saved as sdi12_RxAddr
 *     and the numeric version is saved as sdi12_NumAddr.
 *
 *  2. The command is acknowledged with an a0011 response, indicating
 *  	data available in 1 second and in 1 block. Response is generated
 *  	by sdi12_send_m_atttn( sdi12_RxAddr ) and the response is begun here. It continues
 *  	during the following steps.
 *
 *  3. Within sdi12_send_m_atttn( sdi12_RxAddr ), sdi12_msg_signal is set to sdi12_NumAddr.
 *  	sdi12_msg_signal is initialized to 0xff <- IMPORTANT BIG CHANGE FROM PRIOR IMPLEMENTATION!
 *  	0x00 is not usable (qs an idle indicator) since '0' is a valid SDI12 address.
 *
 *  4. The main() loop of the parent application has to test the value of sdi12_msg_signal
 *  	regularly. It is recommended to test on every pass through the loop. The test
 *  	is for value NOT equal to 0xff.
 *
 *  5. When a valid sdi12_msg_signal is detected, the wireless code does what it needs to
 *  	to construct a data message. The wireless code has approximately 1 second to do this.
 *  	if this proves to be inadequate, then the response atttn string and wait timers both
 *  	need to be adjusted. When the message is ready, the wireless code copies the
 *  	data pointer into sdi12_DataPtr.
 *
 *  6. In the kSDI12_StatWaitSRQ state, the sdi12 interface tests for a non-zero value
 *  	for sdi12_DataPtr. If this happens early enough, the sdi12 interface will send
 *  	an SRQ message on the SDI12 bus. If it is NOT early enough, it just waits for
 *  	the data request message from the SDI12 host. When the non-zero sdi12_DataPtr
 *  	is detected, sid12_msg_signl is reset to 0xff.
 *
 *  7. When the data request is finally received, whether in response to an SRQ, or from
 *  	the timeout process, the parser handles that request (an 'aDn!' message) and
 *  	initiates the transmission of the data. IF a CRC had been requested, that is
 *  	added by the sending routine sdi12_send_wireless( char a, char *msg ). It is NOT clear
 *  	whether the CRC is in addition to the 35 character data limit, or whether these
 *  	three characters must be counted as part of the data field. When transmission
 *  	is finished,  sdi12_DataPtr is reset to zero.
 *
 *  8. If data is not ready when when it has to be sent, the string "a0000" is sent, (as
 *  	indicated by a zero sdi12_DataPtr) as prescribed in the protocol. If this happens,
 *  	sdi12_msg_signal is cleared to 0xff. If sdi12_DoTask finds a non-zero sdi12_DataPtr
 *  	when sdi12_msg_signal is 0xff, sdi12_DataPtr is cleared to zero, functionally
 *  	discarding the data.
 *
 *  IMPORTANT: Formerly, sdi12_send_wireless( char a, char *msg ) was sdi12_send_wireless( char *msg )
 *  and was called from the main program loop. Now, it is called by ParaseCmd() in response
 *  to a received aDn! command. DO NOT call it from main(); it is now PRIVATE!
 *
 *  CRC implementation: CRC is detailed in section 4.12 of the SDI12 Specification document.
 *  The CRC operation begins with (includes) the address character and extends through the
 *  last character of the value field - that is, in the structure used in this code, up to but
 *  not including the first trailing null before any /r/n is added.
 *
 *	DEBUGGING:
 *
 *  When SDI12_DEBUG is defined in sdi12.h, a "debug stack", sdi12_debug[] is implemented. It
 *  is currently dimensioned to 80 bytes which barely covers an initial command. If possible,
 *  increase this to 160 to 200 for full coverage.
 *
 *  The debug buffer has the advantage of being able to watch quite closely how execution occurred
 *  with minimal impact on code execution. This is particularly important because you cannot break
 *  and resume execution since the execution is driven by external events that won't wait for a
 *  debugging pause.
 *
 *  When a break occurs, you can inspect sdi12_debug[] in a memory view window. You may need to
 *  use a "watch" window to determine where it begins.
 *
 *  Each entry into an ISR results in a byte in the debug stack that identifies the ISR, followed
 *  by a byte that indicates the state. The ISR identifier is given in the define list that begins
 *  with kSDI12_RxEnter in sdi12.c. The state are listed in the define list that begins with
 *  kSDI12_StatIdle. There may be other entries, such as the UART received character, depending
 *  on the ISR.
 *
 *  Each ISR exit includes, at the very least, the state.
 *
 *  I use this facility by copying the debug buffer to a text file. Then, I use the debug contents
 *  to trace the execution path by manually following the buffer keys through the code and making
 *  notes in the text file. While this IS slow, it is minimally invasive and and lets you see what
 *  the program did without stopping it.
 *
 */
