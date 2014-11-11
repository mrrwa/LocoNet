#ifndef LOCONET_INCLUDED
#define LOCONET_INCLUDED

/****************************************************************************
 * 	Copyright (C) 2009 to 2013 Alex Shepherd
 * 	Copyright (C) 2013 Damian Philipp
 * 
 * 	Portions Copyright (C) Digitrax Inc.
 * 	Portions Copyright (C) Uhlenbrock Elektronik GmbH
 * 
 * 	This library is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU Lesser General Public
 * 	License as published by the Free Software Foundation; either
 * 	version 2.1 of the License, or (at your option) any later version.
 * 
 * 	This library is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * 	Lesser General Public License for more details.
 * 
 * 	You should have received a copy of the GNU Lesser General Public
 * 	License along with this library; if not, write to the Free Software
 * 	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 *****************************************************************************
 * 
 * 	IMPORTANT:
 * 
 * 	Some of the message formats used in this code are Copyright Digitrax, Inc.
 * 	and are used with permission as part of the MRRwA (previously EmbeddedLocoNet) project.
 *  That permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	MRRwA, please contact Digitrax Inc, for specific permission.
 * 
 * 	Note: The sale any LocoNet device hardware (including bare PCB's) that
 * 	uses this or any other LocoNet software, requires testing and certification
 * 	by Digitrax Inc. and will be subject to a licensing agreement.
 * 
 * 	Please contact Digitrax Inc. for details.
 * 
 *****************************************************************************
 * 
 * 	IMPORTANT:
 * 
 * 	Some of the message formats used in this code are Copyright Uhlenbrock Elektronik GmbH
 * 	and are used with permission as part of the MRRwA (previously EmbeddedLocoNet) project.
 *  That permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	MRRwA, please contact Copyright Uhlenbrock Elektronik GmbH, for specific permission.
 * 
 *****************************************************************************
 * 	DESCRIPTION
 * 	This module provides functions that manage the sending and receiving of LocoNet packets.
 * 	
 * 	As bytes are received from the LocoNet, they are stored in a circular
 * 	buffer and after a valid packet has been received it can be read out.
 * 	
 * 	When packets are sent successfully, they are also appended to the Receive
 * 	circular buffer so they can be handled like they had been received from
 * 	another device.
 * 
 * 	Statistics are maintained for both the send and receiving of packets.
 * 
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 * 
 *****************************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "utility/ln_buf.h"
#include "utility/ln_opc.h"

typedef enum
{
	LN_CD_BACKOFF = 0,
	LN_PRIO_BACKOFF,
	LN_NETWORK_BUSY,
	LN_DONE,
	LN_COLLISION,
	LN_UNKNOWN_ERROR,
	LN_RETRY_ERROR
} LN_STATUS ;

// CD Backoff starts after the Stop Bit (Bit 9) and has a minimum or 20 Bit Times
// but initially starts with an additional 20 Bit Times 
#define   LN_CARRIER_TICKS      20  // carrier detect backoff - all devices have to wait this
#define   LN_MASTER_DELAY        6  // non master devices have to wait this additionally
#define   LN_INITIAL_PRIO_DELAY 20  // initial attempt adds priority delay
#define   LN_BACKOFF_MIN      (LN_CARRIER_TICKS + LN_MASTER_DELAY)      // not going below this
#define   LN_BACKOFF_INITIAL  (LN_BACKOFF_MIN + LN_INITIAL_PRIO_DELAY)  // for the first normal tx attempt
#define   LN_BACKOFF_MAX      (LN_BACKOFF_INITIAL + 10)                 // lower priority is not supported

//
// LNCV error codes
// Used by the LNCV callbacks to signal what kind of error has occurred.
//

// Error-codes for write-requests
#define LNCV_LACK_ERROR_GENERIC (0)
// Unsupported/non-existing CV
#define LNCV_LACK_ERROR_UNSUPPORTED (1)
// CV is read only
#define LNCV_LACK_ERROR_READONLY (2)
// Value out of range
#define LNCV_LACK_ERROR_OUTOFRANGE (3)
// Everything OK
#define LNCV_LACK_OK (127)

// the valid range for module addresses (CV0) as per the LNCV spec.
#define LNCV_MIN_MODULEADDR (0)
#define LNCV_MAX_MODULEADDR (65534)

class LocoNetClass
{
  private:
    LnBuf   LnBuffer ;
	void 		setTxPin(uint8_t txPin);

  public:
    LocoNetClass();
    void        init(void);
    void        init(uint8_t txPin);
    lnMsg*      receive(void);
    LN_STATUS   send(lnMsg *TxPacket);
    LN_STATUS   send(lnMsg *TxPacket, uint8_t PrioDelay);
    LN_STATUS   send(uint8_t OpCode, uint8_t Data1, uint8_t Data2);
    LN_STATUS   send(uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay);
    LN_STATUS   sendLongAck(uint8_t ucCode);
    
    LnBufStats* getStats(void);
    
	const char*	getStatusStr(LN_STATUS Status);
    
    uint8_t processSwitchSensorMessage( lnMsg *LnPacket ) ;
	
    LN_STATUS requestSwitch( uint16_t Address, uint8_t Output, uint8_t Direction ) ;
    LN_STATUS reportSwitch( uint16_t Address ) ;
    LN_STATUS reportSensor( uint16_t Address, uint8_t State ) ;
    LN_STATUS reportPower( uint8_t State ) ;
};

extern LocoNetClass LocoNet;

typedef enum
{
  TH_ST_FREE   = 0,
  TH_ST_ACQUIRE,
  TH_ST_SELECT,
  TH_ST_DISPATCH,
  TH_ST_SLOT_MOVE,
  TH_ST_SLOT_FREE,
  TH_ST_SLOT_RESUME,
  TH_ST_IN_USE
} TH_STATE ;

typedef enum
{
  TH_ER_OK = 0,
  TH_ER_SLOT_IN_USE,
  TH_ER_BUSY,
  TH_ER_NOT_SELECTED,
  TH_ER_NO_LOCO,
  TH_ER_NO_SLOTS
} TH_ERROR ;

#define TH_OP_DEFERRED_SPEED 0x01

class LocoNetThrottleClass
{
  private:
	TH_STATE	myState ;         // State of throttle
	uint16_t	myTicksSinceLastAction ;
	uint16_t	myThrottleId ;		// Id of throttle
	uint8_t		mySlot ;          // Master Slot index
	uint16_t	myAddress ;       // Decoder Address
	uint8_t		mySpeed ;         // Loco Speed
	uint8_t		myDeferredSpeed ; // Deferred Loco Speed setting
	uint8_t		myStatus1 ;       // Stat1
	uint8_t		myDirFunc0to4 ;   // Direction
	uint8_t		myFunc5to8 ;       // Direction
	uint8_t		myUserData ;
	uint8_t		myOptions ;
	uint32_t 	myLastTimerMillis;
	
	void updateAddress(uint16_t Address, uint8_t ForceNotify );
	void updateSpeed(uint8_t Speed, uint8_t ForceNotify );
	void updateState(TH_STATE State, uint8_t ForceNotify );
	void updateStatus1(uint8_t Status, uint8_t ForceNotify );
	void updateDirectionAndFunctions(uint8_t DirFunc0to4, uint8_t ForceNotify );
	void updateFunctions5to8(uint8_t Func5to8, uint8_t ForceNotify );
  
  public:
	void init(uint8_t UserData, uint8_t Options, uint16_t ThrottleId ) ;

	void processMessage(lnMsg *LnPacket ) ;
	void process100msActions(void);

	uint16_t getAddress(void) ;
	TH_ERROR setAddress(uint16_t Address) ;
	TH_ERROR resumeAddress(uint16_t Address, uint8_t LastSlot) ;
	TH_ERROR dispatchAddress(uint16_t Address) ;
	TH_ERROR acquireAddress(void) ;
	void releaseAddress(void) ;
	TH_ERROR freeAddress(uint16_t Address) ;

	uint8_t getSpeed(void) ;
	TH_ERROR setSpeed(uint8_t Speed) ;

	uint8_t getDirection(void) ;
	TH_ERROR setDirection(uint8_t Direction) ;

	uint8_t getFunction(uint8_t Function) ;
	TH_ERROR setFunction(uint8_t Function, uint8_t Value) ;
	TH_ERROR setDirFunc0to4Direct(uint8_t Value) ;
	TH_ERROR setFunc5to8Direct(uint8_t Value) ;

	TH_STATE getState(void) ;
	const char *getStateStr( TH_STATE State );
	const char *getErrorStr( TH_ERROR Error );
};

/************************************************************************************
	The LocoNet fast clock in the Command Station is driven from a 65.535 ms
    time base. A normal minute takes approximately 915 x 65.535 ms ticks.

	The LocoNet fast clock values are stored in a special slot in the Command
	Station called the fast clock slot which is slot number 0x7B or 123
	
	Each of the fields in the slot are supposed to count up until the most significant bit
	is 0x80 and then rollover the appropriate values and reset however this behaviour
	does not seem to hold for all fields and so some corrction factors are needed

	An important part of syncing to the Fast Clock master is to interpret the current
	FRAC_MINS fields so that a Fast Clock Slave can sync to the part minute and then
	rollover it's accumulators in sync with the master. The FRAC_MINS counter is a
	14 bit counter that is stored in the two 7 bit FRAC_MINSL & FRAC_MINSH fields.
	It counts up the FRAC_MINSL field until it rolls over to 0x80 and then increments
	the FRAC_MINSH high field until it rolls over to 0x80 and then increments the minute,
	hour and day fields as appropriate and then resets the FRAC_MINS fields to 0x4000 - 915
	which is stored in each of the 7 bit fields.
 
	HOWEVER when the DCS100 resets FRAC_MINS fields to 0x4000 - 915, it then immediately
	rolls over a 128 count and so the minute is short by 915 - 128 65.535 ms ticks, so it
	runs too fast. To correct this problem the fast clock slot can be overwritten with
	corrected FRAC_MINS field values that the DCS100 will then increment correctly.

	This implementation of a LocoNet Fast Clock Slave has two features to correct these
	short commings:
	
	A) It has the option to reduce the FRAC_MINS count by 128 so that it keeps in step with
	the DCS100 Fast Clock which normally runs too fast. This is enabled by passing in the
	FC_FLAG_DCS100_COMPATIBLE_SPEED flag bit to the init() function. 
	
	B) It has the option to overwrite the LocoNet Fast Clock Master slot values with corrected
	FRAC_MINS fields imediately after it rolls-over the fast minute, to make the DCS100 not run
	too fast as it normally does.	
	
	There also seems to be problems with the hours field not rolling over correctly from 23
	back to 0 and so there is extra processing to work out the hours when it has rolled over
	to 0x80 or 0x00 by the time the bit 7 is cleared. This seems to cause the DT400 throttle
	problems as well and so when running in FC_FLAG_MINUTE_ROLLOVER_SYNC mode, this should
	be corrected. 

	The DT400 throttle display seems to decode the minutes incorrectly by 1 count and so we
	have to make the same interpretation here which is why there is a 127 and not a 128
    roll-over for the minutes. 
***********************************************************************************************/

typedef enum
{
  FC_ST_IDLE,
  FC_ST_REQ_TIME,
  FC_ST_READY,
  FC_ST_DISABLED,
} FC_STATE ;

class LocoNetFastClockClass
{
  private:
	FC_STATE 		fcState ;			// State of the Fast Clock Slave 
	uint8_t			fcFlags ;			// Storage of the option flags passed into initFastClock()
	fastClockMsg 	fcSlotData ;		// Primary storage for the Fast Clock slot data 
	uint8_t 		fcLastPeriod ;		// Period of last tick so we can alternate between
	
	void doNotify( uint8_t Sync );

  public:
	void init(uint8_t DCS100CompatibleSpeed, uint8_t CorrectDCS100Clock, uint8_t NotifyFracMin);
	void poll(void);
	void processMessage(lnMsg *LnPacket );
	void process66msActions(void);
};

/************************************************************************************
    SV (System Variable Handling
************************************************************************************/

typedef enum
{
  SV_EE_SZ_256 = 0,
  SV_EE_SZ_512 = 1,
  SV_EE_SZ_1024 = 2,
  SV_EE_SZ_2048 = 3,
  SV_EE_SZ_4096 = 4,
  SV_EE_SZ_8192 = 5
} SV_EE_SIZE ;

typedef enum
{
  SV_WRITE_SINGLE = 0x01,
  SV_READ_SINGLE = 0x02,
  SV_WRITE_MASKED = 0x03,
  SV_WRITE_QUAD = 0x05,
  SV_READ_QUAD = 0x06,
  SV_DISCOVER = 0x07,
  SV_IDENTIFY = 0x08,
  SV_CHANGE_ADDRESS = 0x09,
  SV_RECONFIGURE = 0x0F
} SV_CMD ;

typedef enum
{
  SV_ADDR_EEPROM_SIZE = 1,
  SV_ADDR_SW_VERSION = 2,
  SV_ADDR_NODE_ID_L = 3,
  SV_ADDR_NODE_ID_H = 4,
  SV_ADDR_SERIAL_NUMBER_L = 5,
  SV_ADDR_SERIAL_NUMBER_H = 6,
  SV_ADDR_USER_BASE = 7,
} SV_ADDR ;

typedef enum
{
  SV_OK = 0,
  SV_ERROR = 1,
  SV_DEFERRED_PROCESSING_NEEDED = 2
} SV_STATUS ;

class LocoNetSystemVariableClass
{
  private:
	uint16_t 	vendorId ;
	uint16_t 	deviceId ;
    uint8_t     swVersion ;
    
    uint8_t DeferredProcessingRequired ;
    uint8_t DeferredSrcAddr ;
    
	/** Read a value from the given EEPROM offset.
	 *
	 * There are two special values for the Offset parameter:
	 *	SV_ADDR_EEPROM_SIZE - Return the size of the EEPROM
	 *  SV_ADDR_SW_VERSION - Return the value of swVersion
	 *  3 and on - Return the byte stored in the EEPROM at location (Offset - 2)
	 *
	 * Parameters:
	 *		Offset: The offset into the EEPROM. Despite the value being passed as 2 Bytes, only the lower byte is respected.
	 *
	 * Returns:
	 *		A Byte containing the EEPROM size, the software version or contents of the EEPROM.
	 *
	 */
    uint8_t readSVStorage(uint16_t Offset );
	
	/** Write the given value to the given Offset in EEPROM.
	 *
	 * TODO: Writes to Offset 0 and 1 will cause data corruption.
	 *
	 * Fires notifySVChanged(Offset), if the value actually chaned.
	 *
	 * Returns:
	 *		A Byte containing the new EEPROM value (even if unchanged).
	 */
    uint8_t writeSVStorage(uint16_t Offset, uint8_t Value);
	
	/** Checks whether the given Offset is a valid value.
	 *
	 * Returns:
	 *		True - if the given Offset is valid. False Otherwise.
	 */
    uint8_t isSVStorageValid(uint16_t Offset);
	
	/** Read the NodeId (Address) for SV programming of this module.
	 *
	 * This method accesses multiple special EEPROM locations.
	 */
    uint16_t readSVNodeId(void);
	
	/** Write the NodeId (Address) for SV programming of this module.
	 *
	 * This method accesses multiple special EEPROM locations.
	 */
    uint16_t writeSVNodeId(uint16_t newNodeId);
	
	/**
	 * Checks whether all addresses of an address range are valid (defers to
	 * isSVStorageValid()). Sends a notification for the first invalid address
	 * (long Ack with a value of 42).
	 *
	 *	TODO: There is a Type error in this method. Return type is bool, but
	 *		actual returned values are Integer.
	 *
	 * Returns:
	 *		0 if at least one address of the range is not valid.
	 *		1 if all addresses out of the range are valid.
	 */
    bool CheckAddressRange(uint16_t startAddress, uint8_t Count);

  public:
	void init(uint16_t newVendorId, uint16_t newDeviceId, uint8_t newSwVersion);
	
	/**
	 * Check whether a message is an SV programming message. If so, the message
	 * is processed.
	 * Call this message in your main loop to implement SV programming.
	 *
	 * TODO: This method should be updated to reflect whether the message has
	 *	been consumed.
	 *
	 * Note that this method will not send out replies.
	 *
	 * Returns:
	 *		SV_OK - the message was or was not an SV programming message.
				It may or may not have been consumed.
	 *		SV_DEFERRED_PROCESSING_NEEDED - the message was an SV programming
				message and has been consumed. doDeferredProcessing() must be
				called to actually process the message.
	 *		SV_ERROR - the message was an SV programming message and carried
				an unsupported OPCODE.
	 *
	 */
	SV_STATUS processMessage(lnMsg *LnPacket );
	
	/**
	 * Attempts to send a reply to an SV programming message.
	 * This method will repeatedly try to send the message, until it succeeds.
	 *
	 * Returns:
	 *		SV_OK - Reply was successfully sent.
	 *		SV_DEFERRED_PROCESSING_NEEDED - Reply was not sent, a later retry is needed.
	 */
    SV_STATUS doDeferredProcessing( void );
};

class LocoNetCVClass
{
  private:
    void makeLNCVresponse( UhlenbrockMsg & ub, uint8_t originalSource, uint16_t first, uint16_t second, uint16_t third, uint8_t last );
    
      // Computes the PXCT byte from the data bytes in the given UhlenbrockMsg.
    void computePXCTFromBytes( UhlenbrockMsg & ub) ;

      // Computes the correct data bytes using the containes PXCT byte
    void computeBytesFromPXCT( UhlenbrockMsg & ub) ;

      // Computes an address from a low- and a high-byte
    uint16_t getAddress(uint8_t lower, uint8_t higher) ;

  public:
	  //Call this method when you want to implement a module that can be configured via Uhlenbrock LNVC messages
	uint8_t processLNCVMessage( lnMsg *LnPacket ) ;
};

/************************************************************************************
    Call-back functions
************************************************************************************/

#if defined (__cplusplus)
	extern "C" {
#endif

extern void notifySensor( uint16_t Address, uint8_t State ) __attribute__ ((weak));

// Address: Switch Address.
// Output: Value 0 for Coil Off, anything else for Coil On
// Direction: Value 0 for Closed/GREEN, anything else for Thrown/RED
extern void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction ) __attribute__ ((weak));
extern void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction ) __attribute__ ((weak));
extern void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) __attribute__ ((weak));
extern void notifyPower( uint8_t State ) __attribute__ ((weak));

// Throttle notify Call-back functions
extern void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t Slot ) __attribute__ ((weak));
extern void notifyThrottleSpeed( uint8_t UserData, TH_STATE State, uint8_t Speed ) __attribute__ ((weak));
extern void notifyThrottleDirection( uint8_t UserData, TH_STATE State, uint8_t Direction ) __attribute__ ((weak));
extern void notifyThrottleFunction( uint8_t UserData, uint8_t Function, uint8_t Value ) __attribute__ ((weak));
extern void notifyThrottleSlotStatus( uint8_t UserData, uint8_t Status ) __attribute__ ((weak));
extern void notifyThrottleError( uint8_t UserData, TH_ERROR Error ) __attribute__ ((weak));
extern void notifyThrottleState( uint8_t UserData, TH_STATE PrevState, TH_STATE State ) __attribute__ ((weak));

// FastClock notify Call-back functions
extern void notifyFastClock( uint8_t Rate, uint8_t Day, uint8_t Hour, uint8_t Minute, uint8_t Sync ) __attribute__ ((weak));
extern void notifyFastClockFracMins( uint16_t FracMins ) __attribute__ ((weak));

// System Variable notify Call-back functions
extern void notifySVChanged(uint16_t Offset) __attribute__ ((weak));

// LNCV notify Call-back functions

// Negative return codes will result in no message being sent.
// Where a value response is appropriate, a return value of LNCV_LACK_OK will trigger the
// response being sent.
// Other values greater than 0 will result in a LACK message being sent.
// When no value result is appropriate, LNCV_LACK_OK will be sent as a LACK.

/**
 * TODO: General LNCV documentation
 * Pick an ArtNr
 * Implement your code to the following behaviour...
 */

/**
 * Notification that an LNCVDiscover message was sent. If a module wants to react to this,
 * It should return LNCV_LACK_OK and set ArtNr and ModuleAddress accordingly.
 * A response just as in the case of notifyLNCVProgrammingStart will be generated.
 * If a module responds to a LNCVDiscover, it should apparently enter programming mode immediately.
 */
extern int8_t notifyLNCVdiscover( uint16_t & ArtNr, uint16_t & ModuleAddress ) __attribute__ ((weak));;

/**
 * Notification that a LNCVProgrammingStart message was received. Application code should process this message and
 * set the return code to LNCV_LACK_OK in case this message was intended for this module (i.e., the addresses match).
 * In case ArtNr and/or ModuleAddress were Broadcast addresses, the Application Code should replace them by their
 * real values.
 * The calling code will then generate an appropriate ACK message.
 * A return code different than LACK_LNCV_OK will result in no response being sent.
 */
extern int8_t notifyLNCVprogrammingStart ( uint16_t & ArtNr, uint16_t & ModuleAddress ) __attribute__ ((weak));

/**
 * Notification that a LNCV read request message was received. Application code should process this message,
 * set the lncvValue to its respective value and set an appropriate return code.
 * return LNCV_LACK_OK leads the calling code to create a response containing lncvValue.
 * return code >= 0 leads to a NACK being sent.
 * return code < 0 will result in no reaction.
 */
extern int8_t notifyLNCVread ( uint16_t ArtNr, uint16_t lncvAddress, uint16_t, uint16_t & lncvValue ) __attribute__ ((weak));

/**
 * Notification that a LNCV value should be written. Application code should process this message and
 * set an appropriate return code.
 * Note 1: LNCV 0 is spec'd to be the ModuleAddress.
 * Note 2: Changes to LNCV 0 must be reflected IMMEDIATELY! E.g. the programmingStop command will
 * be sent using the new address.
 *
 * return codes >= 0 will result in a LACK containing the return code being sent.
 * return codes < 0 will result in no reaction.
 */
extern int8_t notifyLNCVwrite ( uint16_t ArtNr, uint16_t lncvAddress, uint16_t lncvValue ) __attribute__ ((weak));

/**
 * Notification that an LNCV Programming Stop message was received.
 * This message is noch ACKed, thus does not require a result to be returned from the application.
 */
extern void notifyLNCVprogrammingStop( uint16_t ArtNr, uint16_t ModuleAddress ) __attribute__ ((weak));

#if defined (__cplusplus)
}
#endif

#endif
