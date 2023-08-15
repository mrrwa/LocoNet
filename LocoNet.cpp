/****************************************************************************
 * 	Copyright (C) 2009..2013 Alex Shepherd
 *	Copyright (C) 2020 Damian Philipp
 *
 * 	Portions Copyright (C) Digitrax Inc.
 *	Portions Copyright (C) Uhlenbrock Elektronik GmbH
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

 // Uncomment to enable SV Processing Debug Print statements
 // #define DEBUG_SV

#include "LocoNet.h"

#include "ln_sw_uart.h"
#include "ln_config.h"
#include "utils.h"

#if defined(ESP8266)
#  include <EEPROM.h>
#  define E2END 0x1FF // Simulate 512 byte EEPROM

uint8_t eeprom_read_byte(const uint8_t* offset) {
	return EEPROM.read((int)offset);
}

void eeprom_write_byte(const uint8_t* offset, uint8_t value) {
	EEPROM.write((int)offset, value);
}
#elif defined(STM32F1)
#  include <FreeRTOS.h>
#  include <task.h>
#  include <libopencm3/stm32/gpio.h>
#  include <cstdint>
#else
#  include <avr/eeprom.h>
#  include <avr/wdt.h>
#endif

const char* LoconetStatusStrings[] = {
	"CD Backoff",
	"Prio Backoff",
	"Network Busy",
	"Done",
	"Collision",
	"Unknown Error",
	"Retry Error"
};

LocoNetClass::LocoNetClass()
{
}

void LocoNetClass::init(void)
{
#ifdef ESP8266
	init(D7); // By default use pin D7 as the Tx pin to be compatible with the previous library default
#else
	init(6);  // By default use pin 6 as the Tx pin to be compatible with the previous library default
#endif
}

const char* LocoNetClass::getStatusStr(LN_STATUS Status)
{
	if ((Status >= LN_CD_BACKOFF) && (Status <= LN_RETRY_ERROR))
		return LoconetStatusStrings[Status];

	return "Invalid Status";
}


void LocoNetClass::init(uint8_t txPin)
{
	initLnBuf(&LnBuffer);
	setTxPin(txPin);
	initLocoNetHardware(&LnBuffer);

#ifdef ESP8266
	// Setup EEProm emulation on EPS8266
	EEPROM.begin(E2END);
#endif
}

void LocoNetClass::setTxPin(uint8_t txPin)
{
	//#if defined(STM32F1) && !defined(__ARDUINO__)
	{
		//  uint16_t bitNum = 15;
		//  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, (1 << bitNum));  // LN TX
		//  LnPortRegisterType out = GPIOB; // TODO: This shold be configurable..
	}
	//#else
	pinMode(txPin, OUTPUT);

#ifndef ESP8266
	// Not figure out which Port bit is the Tx Bit from the Arduino pin number
	LnPortRegisterType bitMask = digitalPinToBitMask(txPin);
	LnPortRegisterType bitMaskTest = 0x01;
	LnPortRegisterType bitNum = 0;

	LnPortRegisterType port = digitalPinToPort(txPin);
	LnPortAddrType out = portOutputRegister(port);

	while (bitMask != bitMaskTest)
		bitMaskTest = 1 << ++bitNum;

	setTxPortAndPin(out, bitNum);
#else
	setTxPortAndPin(nullptr, txPin);
#endif
}

// Check to see if any messages is ready to receive()?
bool LocoNetClass::available(void)
{
	return lnPacketReady(&LnBuffer);
}

// Check the size in bytes of waiting message
uint8_t LocoNetClass::length(void)
{
	if (lnPacketReady(&LnBuffer))
	{
		lnMsg* m = (lnMsg*)&(LnBuffer.Buf[LnBuffer.ReadIndex]);
		return getLnMsgSize(m);
	}
	else
		return 0;
}

lnMsg* LocoNetClass::receive(void)
{
	return recvLnMsg(&LnBuffer);
}

/*
  Send a LocoNet message, using the default priority delay.

  When an attempt to send fails, this method will continue to try to re-send
  the message until it is successfully sent or until the maximum retry
  limit is reached.

  Return value is one of:
	LN_DONE -         Indicates successful send of the message
	LN_RETRY_ERROR -  Could not successfully send the message within
						LN_TX_RETRIES_MAX attempts
	LN_UNKNOWN -      Indicates an abnormal exit condition for the send attempt.
						In this case, it is recommended to make another send
						attempt.
*/
LN_STATUS LocoNetClass::send(lnMsg* pPacket)
{
	return send(pPacket, LN_BACKOFF_INITIAL);
}

/*
  Send a LocoNet message, using an argument for the priority delay.

  When an attempt to send fails, this method will continue to try to re-send
  the message until it is successfully sent or until the maximum retry
  limit is reached.

  Return value is one of:
	LN_DONE -         Indicates successful send of the message
	LN_RETRY_ERROR -  Could not successfully send the message within
						LN_TX_RETRIES_MAX attempts
	LN_UNKNOWN -      Indicates an abnormal exit condition for the send attempt.
						In this case, it is recommended to make another send
						attempt.
*/
LN_STATUS LocoNetClass::send(lnMsg* pPacket, uint8_t ucPrioDelay)
{
	unsigned char ucTry;
	LN_STATUS enReturn;
	unsigned char ucWaitForEnterBackoff;

	for (ucTry = 0; ucTry < LN_TX_RETRIES_MAX; ucTry++)
	{

		// wait previous traffic and than prio delay and than try tx
		ucWaitForEnterBackoff = 1;  // don't want to abort do/while loop before
		do                          // we did not see the backoff state once
		{
			enReturn = sendLocoNetPacketTry(pPacket, ucPrioDelay);
			if (enReturn == LN_DONE)  // success?
				return LN_DONE;

			if (enReturn == LN_PRIO_BACKOFF)
				ucWaitForEnterBackoff = 0; // now entered backoff -> next state != LN_BACKOFF is worth incrementing the try counter
		} while ((enReturn == LN_CD_BACKOFF) ||                             // waiting CD backoff
			(enReturn == LN_PRIO_BACKOFF) ||                           // waiting master+prio backoff
			((enReturn == LN_NETWORK_BUSY) && ucWaitForEnterBackoff)); // or within any traffic unfinished
			// failed -> next try going to higher prio = smaller prio delay
		if (ucPrioDelay > LN_BACKOFF_MIN)
			ucPrioDelay--;
	}
	LnBuffer.Stats.TxErrors++;
	return LN_RETRY_ERROR;
}

/*
  Create a four-byte LocoNet message from the three method parameters plus a
  computed checksum, and send that message to LocoNet using a default priority
  delay.

  When an attempt to send fails, this method will continue to try to re-send
  the message until it is successfully sent or until the maximum retry
  limit is reached.

  Return value is one of:
	LN_DONE -         Indicates successful send of the message
	LN_RETRY_ERROR -  Could not successfully send the message within
						LN_TX_RETRIES_MAX attempts
	LN_UNKNOWN -      Indicates an abnormal exit condition for the send attempt.
						In this case, it is recommended to make another send
						attempt.
*/
LN_STATUS LocoNetClass::send(uint8_t OpCode, uint8_t Data1, uint8_t Data2)
{
	lnMsg SendPacket;

	SendPacket.data[0] = OpCode;
	SendPacket.data[1] = Data1;
	SendPacket.data[2] = Data2;

	return send(&SendPacket);
}

/*
  Create a four-byte LocoNet message from the three method parameters plus a
  computed checksum, and send that message to LocoNet using a parameter for the
  priority delay.

  This method will make exactly one attempt to send the LocoNet message which
  may or may not succeed.  Code which uses this method must check the return
  status value to determine if the send should be re-tried.  Such code must also
  implement a retry limit counter.

  Return value is one of:
	LN_DONE -         Indicates successful send of the message.
	LN_CD_BACKOFF -   Indicates that the message cannot be sent at this time
						because the LocoNet state machine is in the "Carrier Detect
						Backoff" phase.  This should not count against the "retry"
						count.
	LN_PRIO_BACKOFF - Indicates that the message cannot be sent at this time
						because the LocoNet state machine is in the "Priority
						Backoff" phase.  This should not count against the "retry"
						count.
	LN_NETWORK_BUSY - Indicates that the message cannot be sent at this time
						because some other LocoNet agent is currently sending
						a message.  This should not count against the "retry"
						count.
	LN_COLLISSION -   Indicates that an attempt was made to send the message
						but that the message was corrupted by some other LocoNet
						agent's LocoNe traffic.  The retry counter should be
						decremented and another send should be attempted if the
						retry limit has not been reached.
	LN_UNKNOWN -      Indicates an abnormal exit condition for the send attempt.
						In this case, it is recommended to decrement the retry
						counter and make another send attempt if the retry limit
						has not been reached.
*/
LN_STATUS LocoNetClass::send(uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay)
{
	lnMsg SendPacket;

	SendPacket.data[0] = OpCode;
	SendPacket.data[1] = Data1;
	SendPacket.data[2] = Data2;

	return sendLocoNetPacketTry(&SendPacket, PrioDelay);
}

/* send a LONG_ACK (LACK) message to Loconet as a response to an OPC_PEER_XFER
   message, using the method parameter as the error code in the LONG_ACK message.

  When an attempt to send fails, this method will continue to try to re-send
  the message until it is successfully sent or until the maximum retry
  limit is reached.

  Return value is one of:
	LN_DONE -         Indicates successful send of the message.
	LN_RETRY_ERROR -  Indicates that the method could not successfully send the
						message within LN_TX_RETRIES_MAX attempts.
	LN_UNKNOWN -      Indicates an abnormal exit condition for the send attempt.
						In this case, it is recommended to make another send
						attempt.
*/
LN_STATUS LocoNetClass::sendLongAck(uint8_t ucCode)
{
	lnMsg SendPacket;

	SendPacket.data[0] = OPC_LONG_ACK;
	SendPacket.data[1] = OPC_PEER_XFER - 0x80;
	SendPacket.data[2] = ucCode;

	return send(&SendPacket);
}

LnBufStats* LocoNetClass::getStats(void)
{
	return &LnBuffer.Stats;
}

LN_STATUS LocoNetClass::reportPower(uint8_t State)
{
	lnMsg SendPacket;

	if (State)
		SendPacket.data[0] = OPC_GPON;
	else
		SendPacket.data[0] = OPC_GPOFF;

	return send(&SendPacket);
}

uint8_t LocoNetClass::processSwitchSensorMessage(lnMsg* LnPacket)
{
	uint16_t Address;
	uint8_t  Direction;
	uint8_t  Output;
	uint8_t  ConsumedFlag = 1;

	Address = (LnPacket->srq.sw1 | ((LnPacket->srq.sw2 & 0x0F) << 7));
	if (LnPacket->sr.command != OPC_INPUT_REP)
		Address++;

	switch (LnPacket->sr.command)
	{
	case OPC_INPUT_REP:
		Address <<= 1;
		Address += (LnPacket->ir.in2 & OPC_INPUT_REP_SW) ? 2 : 1;

		if (notifySensor)
			notifySensor(Address, LnPacket->ir.in2 & OPC_INPUT_REP_HI);
		break;

	case OPC_GPON:
		if (notifyPower)
			notifyPower(1);
		break;

	case OPC_GPOFF:
		if (notifyPower)
			notifyPower(0);
		break;

	case OPC_SW_REQ:
		if (notifySwitchRequest)
			notifySwitchRequest(Address, LnPacket->srq.sw2 & OPC_SW_REQ_OUT, LnPacket->srq.sw2 & OPC_SW_REQ_DIR);
		break;

	case OPC_SW_REP:
		if (LnPacket->srp.sn2 & OPC_SW_REP_INPUTS)
		{
			if (notifySwitchReport)
				notifySwitchReport(Address, LnPacket->srp.sn2 & OPC_SW_REP_HI, LnPacket->srp.sn2 & OPC_SW_REP_SW);
		}
		else
		{
			if (notifySwitchOutputsReport)
				notifySwitchOutputsReport(Address, LnPacket->srp.sn2 & OPC_SW_REP_CLOSED, LnPacket->srp.sn2 & OPC_SW_REP_THROWN);
		}
		break;

	case OPC_SW_STATE:
		Direction = LnPacket->srq.sw2 & OPC_SW_REQ_DIR;
		Output = LnPacket->srq.sw2 & OPC_SW_REQ_OUT;

		if (notifySwitchState)
			notifySwitchState(Address, Output, Direction);
		break;

	case OPC_SW_ACK:
		break;

	case OPC_MULTI_SENSE:

		switch (LnPacket->data[1] & OPC_MULTI_SENSE_MSG)
		{
		case OPC_MULTI_SENSE_DEVICE_INFO:
			// This is a PM42 power event.

			/* Working on porting this */
			if (notifyMultiSensePower)
			{
				uint8_t pCMD;
				pCMD = (LnPacket->msdi.arg3 & 0xF0);

				if ((pCMD == 0x30) || (pCMD == 0x10))
				{
					// Autoreverse & Circuitbreaker
					uint8_t cm1 = LnPacket->msdi.arg3;
					uint8_t cm2 = LnPacket->msdi.arg4;

					uint8_t Mode; // 0 = AutoReversing 1 = CircuitBreaker

					uint8_t boardID = ((LnPacket->msdi.arg2 + 1) + ((LnPacket->msdi.arg1 & 0x1) == 1) ? 128 : 0);

					// Report 4 Sub-Districts for a PM4x
					uint8_t d = 1;
					for (uint8_t i = 1; i < 5; i++)
					{
						if ((cm1 & d) != 0)
						{
							Mode = 0;
						}
						else {
							Mode = 1;
						}
						Direction = cm2 & d;
						d = d * 2;
						notifyMultiSensePower(boardID, i, Mode, Direction); // BoardID, Subdistrict, Mode, Direction
					}
				}
				else if (pCMD == 0x70) {
					// Programming
				}
				else if (pCMD == 0x00) {
					// Device type report
				}
			}
			break;

		case OPC_MULTI_SENSE_ABSENT:
		case OPC_MULTI_SENSE_PRESENT:
			// Transponding Event
			if (notifyMultiSenseTransponder)
			{
				uint16_t Locoaddr;
				uint8_t  Present;
				char     Zone;

				Address = LnPacket->mstr.zone + ((LnPacket->mstr.type & 0x1F) << 7);
				Present = (LnPacket->mstr.type & 0x20) != 0 ? true : false;

				Address++;

				if (LnPacket->mstr.adr1 == 0x7D)
					Locoaddr = LnPacket->mstr.adr2;
				else
					Locoaddr = (LnPacket->mstr.adr1 * 128) + LnPacket->mstr.adr2;

				if ((LnPacket->mstr.zone & 0x0F) == 0x00) Zone = 'A';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x02) Zone = 'B';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x04) Zone = 'C';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x06) Zone = 'D';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x08) Zone = 'E';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x0A) Zone = 'F';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x0C) Zone = 'G';
				else if ((LnPacket->mstr.zone & 0x0F) == 0x0E) Zone = 'H';
				else Zone = LnPacket->mstr.zone & 0x0F;

				notifyMultiSenseTransponder(Address, Zone, Locoaddr, Present);
				break;
			}
		}
		break;

	case OPC_LONG_ACK:
		if (LnPacket->lack.opcode == (OPC_SW_STATE & 0x7F))
		{
			Direction = LnPacket->lack.ack1 & 0x01;
			if (notifyLongAck)
				notifyLongAck(LnPacket->data[1], LnPacket->data[2]);
		}
		else
			ConsumedFlag = 0;
		break;

	default:
		ConsumedFlag = 0;
	}

	return ConsumedFlag;
}

LN_STATUS LocoNetClass::requestSwitch(uint16_t Address, uint8_t Output, uint8_t Direction)
{
	uint8_t AddrH = (--Address >> 7) & 0x0F;
	uint8_t AddrL = Address & 0x7F;

	if (Output)
		AddrH |= OPC_SW_REQ_OUT;

	if (Direction)
		AddrH |= OPC_SW_REQ_DIR;

	return send(OPC_SW_REQ, AddrL, AddrH);
}

LN_STATUS LocoNetClass::reportSwitch(uint16_t Address)
{
	Address -= 1;
	return send(OPC_SW_STATE, (Address & 0x7F), ((Address >> 7) & 0x0F));
}

LN_STATUS LocoNetClass::reportSensor(uint16_t Address, uint8_t State)
{
	uint8_t AddrH = ((--Address >> 8) & 0x0F) | OPC_INPUT_REP_CB;
	uint8_t AddrL = (Address >> 1) & 0x7F;
	if (Address % 2)
		AddrH |= OPC_INPUT_REP_SW;

	if (State)
		AddrH |= OPC_INPUT_REP_HI;

	return send(OPC_INPUT_REP, AddrL, AddrH);
}

LocoNetClass LocoNet = LocoNetClass();

// LocoNet Throttle Support


// To make it easier to handle the Speed steps 0 = Stop, 1 = Em Stop and 2 -127
// normal speed steps we will swap speed steps 0 and 1 so that the normal
// range for speed steps is Stop = 1 2-127 is normal as before and now 0 = EmStop
static uint8_t SwapSpeedZeroAndEmStop(uint8_t Speed)
{
	if (Speed == 0)
		return 1;

	if (Speed == 1)
		return 0;

	return Speed;
}

void LocoNetThrottleClass::updateAddress(uint16_t Address, uint8_t ForceNotify)
{
	if (ForceNotify || myAddress != Address)
	{
		myAddress = Address;
		if (notifyThrottleAddress)
			notifyThrottleAddress(myUserData, myState, Address, mySlot);
	}
}

void LocoNetThrottleClass::updateSpeed(uint8_t Speed, uint8_t ForceNotify)
{
	if (ForceNotify || mySpeed != Speed)
	{
		mySpeed = Speed;
		if (notifyThrottleSpeed)
			notifyThrottleSpeed(myUserData, myState, SwapSpeedZeroAndEmStop(Speed));
	}
}

void LocoNetThrottleClass::updateState(TH_STATE State, uint8_t ForceNotify)
{
	TH_STATE  PrevState;

	if (ForceNotify || myState != State)
	{
		PrevState = myState;
		myState = State;
		if (notifyThrottleState)
			notifyThrottleState(myUserData, PrevState, State);
	}
}

void LocoNetThrottleClass::updateStatus1(uint8_t Status, uint8_t ForceNotify)
{
	register uint8_t Mask;	// Temporary uint8_t Variable for bitwise AND to force
	// the compiler to only do 8 bit operations not 16

	if (ForceNotify || myStatus1 != Status)
	{
		myStatus1 = Status;
		if (notifyThrottleSlotStatus)
			notifyThrottleSlotStatus(myUserData, Status);

		Mask = LOCO_IN_USE;
		updateState(((Status & Mask) == Mask) ? TH_ST_IN_USE : TH_ST_FREE, ForceNotify);
	}
}

void LocoNetThrottleClass::updateDirectionAndFunctions(uint8_t DirFunc0to4, uint8_t ForceNotify)
{
	uint8_t Diffs;
	uint8_t Function;
	uint8_t Mask;

	if (ForceNotify || myDirFunc0to4 != DirFunc0to4)
	{
		Diffs = myDirFunc0to4 ^ DirFunc0to4;
		myDirFunc0to4 = DirFunc0to4;

		// Check Functions 1-4
		for (Function = 1, Mask = 1; Function <= 4; Function++)
		{
			if (notifyThrottleFunction && (ForceNotify || Diffs & Mask))
				notifyThrottleFunction(myUserData, Function, DirFunc0to4 & Mask);

			Mask <<= 1;
		}

		// Check Functions 0
		if (notifyThrottleFunction && (ForceNotify || Diffs & DIRF_F0))
			notifyThrottleFunction(myUserData, 0, DirFunc0to4 & (uint8_t)DIRF_F0);

		// Check Direction
		if (notifyThrottleDirection && (ForceNotify || Diffs & DIRF_DIR))
			notifyThrottleDirection(myUserData, myState, DirFunc0to4 & (uint8_t)DIRF_DIR);
	}
}

void LocoNetThrottleClass::updateFunctions5to8(uint8_t Func5to8, uint8_t ForceNotify)
{
	uint8_t Diffs;
	uint8_t Function;
	uint8_t Mask;

	if (ForceNotify || myFunc5to8 != Func5to8)
	{
		Diffs = myFunc5to8 ^ Func5to8;
		myFunc5to8 = Func5to8;

		// Check Functions 5-8
		for (Function = 5, Mask = 1; Function <= 8; Function++)
		{
			if (notifyThrottleFunction && (ForceNotify || Diffs & Mask))
				notifyThrottleFunction(myUserData, Function, Func5to8 & Mask);

			Mask <<= 1;
		}
	}
}

void LocoNetThrottleClass::updateSpeedSteps(TH_SPEED_STEPS SpeedSteps, uint8_t ForceNotify)
{
	if (ForceNotify || ((myStatus1 & 0x07) != SpeedSteps))
	{
		myStatus1 = (myStatus1 & 0xF8) | SpeedSteps;
		if (notifyThrottleSpeedSteps)
			notifyThrottleSpeedSteps(myUserData, SpeedSteps);
	}
}

#define SLOT_REFRESH_TICKS   		600   // 600 * 100ms = 60 seconds between speed refresh

void LocoNetThrottleClass::process100msActions(void)
{
	if (myState == TH_ST_IN_USE)
	{
		myTicksSinceLastAction++;

		bool isDeferredSpeedChanged = myDeferredSpeed > LN_DEFERRED_SPEED_NOOP;
		if (isDeferredSpeedChanged || (myTicksSinceLastAction > SLOT_REFRESH_TICKS))
		{
			LocoNet.send(OPC_LOCO_SPD, mySlot, (isDeferredSpeedChanged) ? myDeferredSpeed : mySpeed);

			myDeferredSpeed = LN_DEFERRED_SPEED_NOOP;

			myTicksSinceLastAction = 0;
		}
	}
}

void LocoNetThrottleClass::init(uint8_t UserData, uint8_t Options, uint16_t ThrottleId)
{
	myState = TH_ST_FREE;
	myThrottleId = ThrottleId;
	myDeferredSpeed = LN_DEFERRED_SPEED_NOOP;
	myUserData = UserData;
	myOptions = Options;
}

void LocoNetThrottleClass::processMessage(lnMsg* LnPacket)
{
	uint8_t  Data2;
	uint16_t  SlotAddress;

	// Update our copy of slot information if applicable
	if (LnPacket->sd.command == OPC_SL_RD_DATA)
	{
		SlotAddress = (uint16_t)((LnPacket->sd.adr2 << 7) + LnPacket->sd.adr);

		if (mySlot == LnPacket->sd.slot)
		{
			// Make sure that the slot address matches even though we have the right slot number
			// as it is possible that another throttle got in before us and took our slot.
			if (myAddress == SlotAddress)
			{
				if ((myState == TH_ST_SLOT_RESUME) && (myThrottleId != (uint16_t)((LnPacket->sd.id2 << 7) + LnPacket->sd.id1)))
				{
					updateState(TH_ST_FREE, 1);
					if (notifyThrottleError)
						notifyThrottleError(myUserData, TH_ER_NO_LOCO);
				}
				else
				{
					updateState(TH_ST_IN_USE, 1);
					updateAddress(SlotAddress, 1);
					updateSpeed(LnPacket->sd.spd, 1);
					updateDirectionAndFunctions(LnPacket->sd.dirf, 1);
					updateFunctions5to8(LnPacket->sd.snd, 1);

					updateSpeedSteps(mySpeedSteps, 1);

					// We need to force a State update to cause a display refresh once all data is known
					updateState(TH_ST_IN_USE, 1);

					// Now Write our own Speed Steps and Throttle Id to the slot and write it back to the command station
					LnPacket->sd.command = OPC_WR_SL_DATA;
					LnPacket->sd.stat = (LnPacket->sd.stat & 0xf8) | mySpeedSteps;
					updateStatus1(LnPacket->sd.stat, 1);

					LnPacket->sd.id1 = (uint8_t)(myThrottleId & 0x7F);
					LnPacket->sd.id2 = (uint8_t)(myThrottleId >> 7);

					LocoNet.send(LnPacket);
				}
			}
			// Ok another throttle did a NULL MOVE with the same slot before we did
			// so we have to try again
			else if (myState == TH_ST_SLOT_MOVE)
			{
				updateState(TH_ST_SELECT, 1);
				LocoNet.send(OPC_LOCO_ADR, (uint8_t)(myAddress >> 7), (uint8_t)(myAddress & 0x7F));
			}
		}
		// Slot data is not for one of our slots so check if we have requested a new addres
		else
		{
			if (myAddress == SlotAddress)
			{
				if ((myState == TH_ST_SELECT) || (myState == TH_ST_DISPATCH))
				{
					if ((LnPacket->sd.stat & STAT1_SL_CONUP) == 0 &&
						(LnPacket->sd.stat & LOCO_IN_USE) != LOCO_IN_USE)
					{
						if (myState == TH_ST_SELECT)
						{
							updateState(TH_ST_SLOT_MOVE, 1);
							mySlot = LnPacket->sd.slot;
							Data2 = LnPacket->sd.slot;
						}
						else
						{
							updateState(TH_ST_FREE, 1);
							Data2 = 0;
						}

						LocoNet.send(OPC_MOVE_SLOTS, LnPacket->sd.slot, Data2);
					}
					else
					{
						if (notifyThrottleError)
							notifyThrottleError(myUserData, TH_ER_SLOT_IN_USE);
						updateState(TH_ST_FREE, 1);
					}
				}
				else if (myState == TH_ST_SLOT_STEAL)
				{
					// Make Sure the Slot is actually IN_USE already as we are not going to do an SLOT_MOVE etc
					if ((LnPacket->sd.stat & STAT1_SL_CONUP) == 0 &&
						(LnPacket->sd.stat & LOCO_IN_USE) == LOCO_IN_USE)
					{
						mySlot = LnPacket->sd.slot;

						updateState(TH_ST_IN_USE, 1);

						updateAddress(SlotAddress, 1);
						updateSpeed(LnPacket->sd.spd, 1);
						updateDirectionAndFunctions(LnPacket->sd.dirf, 1);
						updateFunctions5to8(LnPacket->sd.snd, 1);
						updateStatus1(LnPacket->sd.stat, 1);

						// We need to force a State update to cause a display refresh once all data is known
						updateState(TH_ST_IN_USE, 1);
					}
					else
					{
						if (notifyThrottleError)
							notifyThrottleError(myUserData, TH_ER_NO_LOCO);
						updateState(TH_ST_FREE, 1);
					}
				}
				else if (myState == TH_ST_SLOT_FORCE_FREE)
				{
					LocoNet.send(OPC_SLOT_STAT1, LnPacket->sd.slot, (uint8_t)(myStatus1 & ~(STAT1_SL_BUSY | STAT1_SL_ACTIVE)));
					mySlot = 0xFF;
					updateState(TH_ST_FREE, 1);
				}
			}

			if (myState == TH_ST_ACQUIRE)
			{
				mySlot = LnPacket->sd.slot;
				updateState(TH_ST_IN_USE, 1);

				updateAddress(SlotAddress, 1);
				updateSpeed(LnPacket->sd.spd, 1);
				updateDirectionAndFunctions(LnPacket->sd.dirf, 1);
				updateStatus1(LnPacket->sd.stat, 1);
			}
		}
	}

	else if (((LnPacket->sd.command >= OPC_LOCO_SPD) && (LnPacket->sd.command <= OPC_LOCO_SND)) ||
		(LnPacket->sd.command == OPC_SLOT_STAT1))
	{
		if (mySlot == LnPacket->ld.slot)
		{
			if (LnPacket->ld.command == OPC_LOCO_SPD)
				updateSpeed(LnPacket->ld.data, 0);

			else if (LnPacket->ld.command == OPC_LOCO_DIRF)
				updateDirectionAndFunctions(LnPacket->ld.data, 0);

			else if (LnPacket->ld.command == OPC_LOCO_SND)
				updateFunctions5to8(LnPacket->ld.data, 0);

			else if (LnPacket->ld.command == OPC_SLOT_STAT1)
				updateStatus1(LnPacket->ld.data, 0);
		}
	}
	else if (LnPacket->lack.command == OPC_LONG_ACK)
	{
		if ((myState >= TH_ST_ACQUIRE) && (myState <= TH_ST_SLOT_MOVE))
		{
			if (LnPacket->lack.opcode == (OPC_MOVE_SLOTS & 0x7F))
				if (notifyThrottleError)
					notifyThrottleError(myUserData, TH_ER_NO_LOCO);

			if (LnPacket->lack.opcode == (OPC_LOCO_ADR & 0x7F))
				if (notifyThrottleError)
					notifyThrottleError(myUserData, TH_ER_NO_SLOTS);

			updateState(TH_ST_FREE, 1);
		}
	}
}

uint16_t LocoNetThrottleClass::getAddress(void)
{
	return myAddress;
}

TH_ERROR LocoNetThrottleClass::stealAddress(uint16_t Address)
{
	if (myState <= TH_ST_RELEASE)
	{
		updateAddress(Address, 1);
		updateState(TH_ST_SLOT_STEAL, 1);

		LocoNet.send(OPC_LOCO_ADR, (uint8_t)(Address >> 7), (uint8_t)(Address & 0x7F));
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_BUSY);
	return TH_ER_BUSY;
}


TH_ERROR LocoNetThrottleClass::setAddress(uint16_t Address)
{
	if (myState <= TH_ST_RELEASE)
	{
		updateAddress(Address, 1);
		updateState(TH_ST_SELECT, 1);

		LocoNet.send(OPC_LOCO_ADR, (uint8_t)(Address >> 7), (uint8_t)(Address & 0x7F));
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_BUSY);
	return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottleClass::resumeAddress(uint16_t Address, uint8_t LastSlot)
{
	if (myState <= TH_ST_RELEASE)
	{
		mySlot = LastSlot;
		updateAddress(Address, 1);
		updateState(TH_ST_SLOT_RESUME, 1);

		LocoNet.send(OPC_RQ_SL_DATA, LastSlot, 0);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_BUSY);
	return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottleClass::freeAddressForce(uint16_t Address)
{
	if (myState <= TH_ST_RELEASE)
	{
		updateAddress(Address, 1);
		updateState(TH_ST_SLOT_FORCE_FREE, 1);

		LocoNet.send(OPC_LOCO_ADR, (uint8_t)(Address >> 7), (uint8_t)(Address & 0x7F));
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_BUSY);
	return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottleClass::dispatchAddress(void)
{
	if (myState == TH_ST_IN_USE)
	{
		updateState(TH_ST_FREE, 1);
		LocoNet.send(OPC_MOVE_SLOTS, mySlot, 0);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottleClass::dispatchAddress(uint16_t Address)
{
	if (myState <= TH_ST_RELEASE)
	{
		updateAddress(Address, 1);
		updateState(TH_ST_DISPATCH, 1);

		LocoNet.send(OPC_LOCO_ADR, (uint8_t)(Address >> 7), (uint8_t)(Address & 0x7F));
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_BUSY);
	return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottleClass::acquireAddress(void)
{
	if (myState <= TH_ST_RELEASE)
	{
		updateState(TH_ST_ACQUIRE, 1);

		LocoNet.send(OPC_MOVE_SLOTS, 0, 0);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_BUSY);
	return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottleClass::releaseAddress(void)
{
	if (myState == TH_ST_IN_USE)
	{
		LocoNet.send(OPC_SLOT_STAT1, mySlot, (uint8_t)(myStatus1 & ~(STAT1_SL_BUSY)));

		mySlot = 0xFF;
		updateState(TH_ST_RELEASE, 1);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottleClass::idleAddress(void)
{
	if (myState == TH_ST_IN_USE)
	{
		LocoNet.send(OPC_SLOT_STAT1, mySlot, (uint8_t)(myStatus1 & ~(STAT1_SL_ACTIVE)));

		mySlot = 0xFF;
		updateState(TH_ST_IDLE, 1);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottleClass::freeAddress(void)
{
	if (myState == TH_ST_IN_USE)
	{
		LocoNet.send(OPC_SLOT_STAT1, mySlot, (uint8_t)(myStatus1 & ~(STAT1_SL_BUSY | STAT1_SL_ACTIVE)));

		mySlot = 0xFF;
		updateState(TH_ST_FREE, 1);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

uint8_t LocoNetThrottleClass::getSpeed(void)
{
	return SwapSpeedZeroAndEmStop(mySpeed);
}

TH_ERROR LocoNetThrottleClass::setSpeed(uint8_t Speed)
{
	if (myState == TH_ST_IN_USE)
	{
		Speed = SwapSpeedZeroAndEmStop(Speed);

		if (mySpeed != Speed)
		{
			// Always defer any speed other than stop or em stop
			if ((myOptions & TH_OP_DEFERRED_SPEED) &&
				(Speed > 1 || myTicksSinceLastAction == 0))
			{
				myDeferredSpeed = Speed;
			}
			else
			{
				LocoNet.send(OPC_LOCO_SPD, mySlot, Speed);
				myTicksSinceLastAction = 0;
				myDeferredSpeed = LN_DEFERRED_SPEED_NOOP;
			}
		}
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

uint8_t LocoNetThrottleClass::getDirection(void)
{
	return myDirFunc0to4 & (uint8_t)DIRF_DIR;
}

TH_ERROR LocoNetThrottleClass::setDirection(uint8_t Direction)
{
	if (myState == TH_ST_IN_USE)
	{
		LocoNet.send(OPC_LOCO_DIRF, mySlot,
			(Direction) ? (uint8_t)(myDirFunc0to4 | DIRF_DIR) : (uint8_t)(myDirFunc0to4 & ~DIRF_DIR));

		myTicksSinceLastAction = 0;
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

uint8_t LocoNetThrottleClass::getFunction(uint8_t Function)
{
	uint8_t Mask;

	if (Function <= 4)
	{
		Mask = (uint8_t)(1 << ((Function) ? Function - 1 : 4));
		return myDirFunc0to4 & Mask;
	}

	Mask = (uint8_t)(1 << (Function - 5));
	return myFunc5to8 & Mask;
}

TH_ERROR LocoNetThrottleClass::setFunction(uint8_t Function, uint8_t Value)
{
	uint8_t Mask;
	uint8_t OpCode;
	uint8_t Data;

	if (myState == TH_ST_IN_USE)
	{
		if (Function <= 4)
		{
			OpCode = OPC_LOCO_DIRF;
			Data = myDirFunc0to4;
			Mask = (uint8_t)(1 << ((Function) ? Function - 1 : 4));
		}
		else
		{
			OpCode = OPC_LOCO_SND;
			Data = myFunc5to8;
			Mask = (uint8_t)(1 << (Function - 5));
		}

		if (Value)
			Data |= Mask;
		else
			Data &= (uint8_t)~Mask;

		LocoNet.send(OpCode, mySlot, Data);

		myTicksSinceLastAction = 0;
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottleClass::setDirFunc0to4Direct(uint8_t Value)
{
	if (myState == TH_ST_IN_USE)
	{
		LocoNet.send(OPC_LOCO_DIRF, mySlot, Value & 0x7F);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottleClass::setFunc5to8Direct(uint8_t Value)
{
	if (myState == TH_ST_IN_USE)
	{
		LocoNet.send(OPC_LOCO_SND, mySlot, Value & 0x7F);
		return TH_ER_OK;
	}

	if (notifyThrottleError)
		notifyThrottleError(myUserData, TH_ER_NOT_SELECTED);
	return TH_ER_NOT_SELECTED;
}

TH_STATE LocoNetThrottleClass::getState(void)
{
	return myState;
}


const char* LocoNetThrottleClass::getStateStr(TH_STATE State)
{
	switch (State)
	{
	case TH_ST_FREE:
		return "Free";

	case TH_ST_IDLE:
		return "Idle";

	case TH_ST_RELEASE:
		return "Release";

	case TH_ST_ACQUIRE:
		return "Acquire";

	case TH_ST_SELECT:
		return "Select";

	case TH_ST_DISPATCH:
		return "Dispatch";

	case TH_ST_SLOT_MOVE:
		return "Slot Move";

	case TH_ST_SLOT_FORCE_FREE:
		return "Slot Force Free";

	case TH_ST_SLOT_RESUME:
		return "Slot Resume";

	case TH_ST_SLOT_STEAL:
		return "Slot Steal";

	case TH_ST_IN_USE:
		return "In Use";

	default:
		return "Unknown";
	}
}

const char* LocoNetThrottleClass::getErrorStr(TH_ERROR Error)
{
	switch (Error)
	{
	case TH_ER_OK:

		return "Ok";

	case TH_ER_SLOT_IN_USE:

		return "In Use";

	case TH_ER_BUSY:

		return "Busy";

	case TH_ER_NOT_SELECTED:

		return "Not Sel";

	case TH_ER_NO_LOCO:

		return "No Loco";

	case TH_ER_NO_SLOTS:

		return "No Free Slots";

	default:

		return "Unknown";
	}
}

TH_SPEED_STEPS LocoNetThrottleClass::getSpeedSteps(void)
{
	return mySpeedSteps;
}

void LocoNetThrottleClass::setSpeedSteps(TH_SPEED_STEPS newSpeedSteps)
{
	mySpeedSteps = newSpeedSteps;
	if ((myState == TH_ST_IN_USE) && ((myStatus1 & 0x07) != mySpeedSteps))
	{
		myStatus1 = (myStatus1 & 0xf8) | mySpeedSteps;
		LocoNet.send(OPC_SLOT_STAT1, mySlot, myStatus1);
	}
	updateSpeedSteps(mySpeedSteps, 1);
}

const char* LocoNetThrottleClass::getSpeedStepStr(TH_SPEED_STEPS speedStep)
{
	switch (speedStep)
	{
	case TH_SP_ST_28:	  // 000=28 step/ 3 BYTE PKT regular mode
		return "28";

	case TH_SP_ST_28_TRI:  // 001=28 step. Generate Trinary packets for this Mobile ADR
		return "28 Tri";

	case TH_SP_ST_14:      // 010=14 step MODE
		return "14";

	case TH_SP_ST_128:     // 011=send 128 speed mode packets
		return "128";

	case TH_SP_ST_28_ADV:  // 100=28 Step decoder ,Allow Advanced DCC consisting
		return "28 Adv";

	case TH_SP_ST_128_ADV: // 111=128 Step decoder, Allow Advanced DCC consisting
		return "128 Adv";
	}

	return "Unknown";
}

#define FC_FLAG_DCS100_COMPATIBLE_SPEED	0x01
#define FC_FLAG_MINUTE_ROLLOVER_SYNC	0x02
#define FC_FLAG_NOTIFY_FRAC_MINS_TICK	0x04
#define FC_FRAC_MIN_BASE   				0x3FFF
#define FC_FRAC_RESET_HIGH	 			0x78
#define FC_FRAC_RESET_LOW 	 			0x6D
#define FC_TIMER_TICKS         			65        // 65ms ticks
#define FC_TIMER_TICKS_REQ	  			250        // 250ms waiting for Response to FC Req

void LocoNetFastClockClass::init(uint8_t DCS100CompatibleSpeed, uint8_t CorrectDCS100Clock, uint8_t NotifyFracMin)
{
	fcState = FC_ST_IDLE;

	fcFlags = 0;
	if (DCS100CompatibleSpeed)
		fcFlags |= FC_FLAG_DCS100_COMPATIBLE_SPEED;

	if (CorrectDCS100Clock)
		fcFlags |= FC_FLAG_MINUTE_ROLLOVER_SYNC;

	if (NotifyFracMin)
		fcFlags |= FC_FLAG_NOTIFY_FRAC_MINS_TICK;
}

void LocoNetFastClockClass::poll(void)
{
	LocoNet.send(OPC_RQ_SL_DATA, FC_SLOT, 0);
}

void LocoNetFastClockClass::doNotify(uint8_t Sync)
{
	if (notifyFastClock)
		notifyFastClock(fcSlotData.clk_rate, fcSlotData.days,
			(fcSlotData.hours_24 >= (128 - 24)) ? fcSlotData.hours_24 - (128 - 24) : fcSlotData.hours_24 % 24,
			fcSlotData.mins_60 - (127 - 60), Sync);
}

void LocoNetFastClockClass::processMessage(lnMsg* LnPacket)
{
	if ((LnPacket->fc.slot == FC_SLOT) && ((LnPacket->fc.command == OPC_WR_SL_DATA) || (LnPacket->fc.command == OPC_SL_RD_DATA)))
	{
		if (LnPacket->fc.clk_cntrl & 0x40)
		{
			if (fcState >= FC_ST_REQ_TIME)
			{
				memcpy(&fcSlotData, &LnPacket->fc, sizeof(fastClockMsg));

				doNotify(1);

				if (notifyFastClockFracMins && fcFlags & FC_FLAG_NOTIFY_FRAC_MINS_TICK)
					notifyFastClockFracMins(FC_FRAC_MIN_BASE - ((fcSlotData.frac_minsh << 7) + fcSlotData.frac_minsl));

				fcState = FC_ST_READY;
			}
		}
		else
			fcState = FC_ST_DISABLED;
	}
}

void LocoNetFastClockClass::process66msActions(void)
{
	// If we are all initialised and ready then increment accumulators
	if (fcState == FC_ST_READY)
	{
		fcSlotData.frac_minsl += fcSlotData.clk_rate;
		if (fcSlotData.frac_minsl & 0x80)
		{
			fcSlotData.frac_minsl &= ~0x80;

			fcSlotData.frac_minsh++;
			if (fcSlotData.frac_minsh & 0x80)
			{
				// For the next cycle prime the fraction of a minute accumulators
				fcSlotData.frac_minsl = FC_FRAC_RESET_LOW;

				// If we are in FC_FLAG_DCS100_COMPATIBLE_SPEED mode we need to run faster
				// by reducong the FRAC_MINS duration count by 128
				fcSlotData.frac_minsh = FC_FRAC_RESET_HIGH + (fcFlags & FC_FLAG_DCS100_COMPATIBLE_SPEED);

				fcSlotData.mins_60++;
				if (fcSlotData.mins_60 >= 0x7F)
				{
					fcSlotData.mins_60 = 127 - 60;

					fcSlotData.hours_24++;
					if (fcSlotData.hours_24 & 0x80)
					{
						fcSlotData.hours_24 = 128 - 24;

						fcSlotData.days++;
					}
				}

				// We either send a message out onto the LocoNet to change the time,
				// which we will also see and act on or just notify our user
				// function that our internal time has changed.
				if (fcFlags & FC_FLAG_MINUTE_ROLLOVER_SYNC)
				{
					fcSlotData.command = OPC_WR_SL_DATA;
					LocoNet.send((lnMsg*)&fcSlotData);
				}
				else
					doNotify(0);
			}
		}

		if (notifyFastClockFracMins && (fcFlags & FC_FLAG_NOTIFY_FRAC_MINS_TICK))
			notifyFastClockFracMins(FC_FRAC_MIN_BASE - ((fcSlotData.frac_minsh << 7) + fcSlotData.frac_minsl));
	}

	if (fcState == FC_ST_IDLE)
	{
		LocoNet.send(OPC_RQ_SL_DATA, FC_SLOT, 0);
		fcState = FC_ST_REQ_TIME;
	}
}

#if defined(STM32F1)
// STM31F1 has no EEPROM.
#else

void LocoNetSystemVariableClass::init(uint8_t newMfgId, uint8_t newDevId, uint16_t newProductId, uint8_t newSwVersion)
{
	DeferredProcessingRequired = 0;
	DeferredSrcAddr = 0;

	mfgId = newMfgId;
	devId = newDevId;
	productId = newProductId;
	swVersion = newSwVersion;
}

uint8_t LocoNetSystemVariableClass::readSVStorage(uint16_t Offset)
{
	uint8_t retValue;

	if (notifySVRead)
	{
		//if handled by callback, the callback must return true
		if(notifySVRead(Offset, retValue))
		{
			return retValue;
		}
	}

	if (Offset == SV_ADDR_EEPROM_SIZE)
#if (E2END==0x0FF)	/* E2END is defined in processor include */
		return SV_EE_SZ_256;
#elif (E2END==0x1FF)
		return SV_EE_SZ_512;
#elif (E2END==0x3FF)
		return SV_EE_SZ_1024;
#elif (E2END==0x7FF)
		return SV_EE_SZ_2048;
#elif (E2END==0xFFF)
		return SV_EE_SZ_4096;
#else
		return 0xFF;
#endif
	if (Offset == SV_ADDR_SW_VERSION)
		retValue = swVersion;

	else
	{
		Offset -= 2;    // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
		retValue = eeprom_read_byte((uint8_t*)Offset);
	}
	return retValue;
}

uint8_t LocoNetSystemVariableClass::writeSVStorage(uint16_t Offset, uint8_t Value)
{
	if (notifySVWrite)
	{
		//if handled by callback, the callback must return true
		if(notifySVWrite(Offset, Value))
		{
			if (notifySVChanged)
				notifySVChanged(Offset);
			return Value;
		}
	}

	Offset -= 2;      // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
	if (eeprom_read_byte((uint8_t*)Offset) != Value)
	{
		eeprom_write_byte((uint8_t*)Offset, Value);

		if (notifySVChanged)
			notifySVChanged(Offset + 2);
	}
	return eeprom_read_byte((uint8_t*)Offset);
}

uint8_t LocoNetSystemVariableClass::isSVStorageValid(uint16_t Offset)
{
	return (Offset >= SV_ADDR_EEPROM_SIZE) && (Offset <= E2END + 2);
}

bool LocoNetSystemVariableClass::CheckAddressRange(uint16_t startAddress, uint8_t Count)
{
#ifdef DEBUG_SV
	Serial.print("LNSV CheckAddressRange: Start: ");
	Serial.print(startAddress);
	Serial.print(" Size: ");
	Serial.println(Count);
#endif    
	while (Count != 0)
	{
		if (notifyCheckAddressRange)
		{
			bool bValidAddress = false;
			if (notifyCheckAddressRange(startAddress, bValidAddress))
			{
				if (!bValidAddress)
				{
					LocoNet.sendLongAck(42); // report invalid SV address error
					return 0;
				}
				//force next item in loop, address validation handled in callback
				startAddress++;
				Count--;
				continue;
			}
		}
			
		if (!isSVStorageValid(startAddress))
		{
			LocoNet.sendLongAck(42); // report invalid SV address error
			return 0;
		}
		startAddress++;
		Count--;
	}

	return 1; // all valid
}

uint16_t LocoNetSystemVariableClass::writeSVNodeId(uint16_t newNodeId)
{
#ifdef DEBUG_SV
	Serial.print("LNSV Write Node Addr: ");
	Serial.println(newNodeId);
#endif

	writeSVStorage(SV_ADDR_NODE_ID_H, newNodeId >> (byte)8);
	writeSVStorage(SV_ADDR_NODE_ID_L, newNodeId & (byte)0x00FF);

	return readSVNodeId();
}

uint16_t LocoNetSystemVariableClass::readSVNodeId(void)
{
	return (readSVStorage(SV_ADDR_NODE_ID_H) << 8) | readSVStorage(SV_ADDR_NODE_ID_L);
}

typedef union
{
	word                   w;
	struct { byte lo, hi; } b;
} U16_t;

typedef union
{
	struct
	{
		U16_t unDestinationId;
		U16_t unMfgIdDevIdOrSvAddress;
		U16_t unproductId;
		U16_t unSerialNumber;
	}    stDecoded;
	byte abPlain[8];
} SV_Addr_t;

SV_STATUS LocoNetSystemVariableClass::processMessage(lnMsg* LnPacket)
{
	SV_Addr_t unData;

	if ((LnPacket->sv.mesg_size != (byte)0x10) ||
		(LnPacket->sv.command != (byte)OPC_PEER_XFER) ||
		(LnPacket->sv.sv_type != (byte)0x02) ||
		(LnPacket->sv.sv_cmd & (byte)0x40) ||
		((LnPacket->sv.svx1 & (byte)0xF0) != (byte)0x10) ||
		((LnPacket->sv.svx2 & (byte)0xF0) != (byte)0x10))
		return SV_OK;

	decodePeerData(&LnPacket->px, unData.abPlain);

#ifdef DEBUG_SV
	Serial.print("LNSV Src: ");
	Serial.print(LnPacket->sv.src);
	Serial.print("  Dest: ");
	Serial.print(unData.stDecoded.unDestinationId.w);
	Serial.print("  CMD: ");
	Serial.println(LnPacket->sv.sv_cmd, HEX);
#endif
	if ((LnPacket->sv.sv_cmd != SV_DISCOVER) &&
		(LnPacket->sv.sv_cmd != SV_CHANGE_ADDRESS) &&
		(unData.stDecoded.unDestinationId.w != readSVNodeId()))
	{
#ifdef DEBUG_SV
		Serial.print("LNSV Dest Not Equal: ");
		Serial.println(readSVNodeId());
#endif
		return SV_OK;
	}

	switch (LnPacket->sv.sv_cmd)
	{
	case SV_WRITE_SINGLE:
		if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 1)) return SV_ERROR;
		writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, unData.abPlain[4]);
		// fall through intended!
	case SV_READ_SINGLE:
		if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 1)) return SV_ERROR;
		unData.abPlain[4] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w);
		break;

	case SV_WRITE_MASKED:
		if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 1)) return SV_ERROR;
		// new scope for temporary local variables only
		{
			unsigned char ucOld = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w) & (~unData.abPlain[5]);
			unsigned char ucNew = unData.abPlain[4] & unData.abPlain[5];
			writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, ucOld | ucNew);
		}
		unData.abPlain[4] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w);
		break;

	case SV_WRITE_QUAD:
		if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 4)) return SV_ERROR;
		writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 0, unData.abPlain[4]);
		writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 1, unData.abPlain[5]);
		writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 2, unData.abPlain[6]);
		writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 3, unData.abPlain[7]);
		// fall through intended!
	case SV_READ_QUAD:
		if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 4)) return SV_ERROR;
		unData.abPlain[4] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 0);
		unData.abPlain[5] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 1);
		unData.abPlain[6] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 2);
		unData.abPlain[7] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w + 3);
		break;

	case SV_DISCOVER:
		DeferredSrcAddr = LnPacket->sv.src;
		DeferredProcessingRequired = 1;
		return SV_DEFERRED_PROCESSING_NEEDED;
		break;

	case SV_IDENTIFY:
		unData.stDecoded.unDestinationId.w = readSVNodeId();
		unData.stDecoded.unMfgIdDevIdOrSvAddress.b.hi = devId;
		unData.stDecoded.unMfgIdDevIdOrSvAddress.b.lo = mfgId;
		unData.stDecoded.unproductId.w = productId;
		unData.stDecoded.unSerialNumber.b.lo = readSVStorage(SV_ADDR_SERIAL_NUMBER_L);
		unData.stDecoded.unSerialNumber.b.hi = readSVStorage(SV_ADDR_SERIAL_NUMBER_H);
		break;

	case SV_CHANGE_ADDRESS:
		if ((mfgId != unData.stDecoded.unMfgIdDevIdOrSvAddress.b.lo) || (devId != unData.stDecoded.unMfgIdDevIdOrSvAddress.b.hi))
			return SV_OK; // not addressed
		if (productId != unData.stDecoded.unproductId.w)
			return SV_OK; // not addressed
		if (readSVStorage(SV_ADDR_SERIAL_NUMBER_L) != unData.stDecoded.unSerialNumber.b.lo)
			return SV_OK; // not addressed
		if (readSVStorage(SV_ADDR_SERIAL_NUMBER_H) != unData.stDecoded.unSerialNumber.b.hi)
			return SV_OK; // not addressed

		if (writeSVNodeId(unData.stDecoded.unDestinationId.w) != unData.stDecoded.unDestinationId.w)
		{
			LocoNet.sendLongAck(44);  // failed to change address in non-volatile memory (not implemented or failed to write)
			return SV_OK; // the LN reception was ok, we processed the message
		}
		break;

	case SV_RECONFIGURE:
		break;  // actual handling is done after sending out the reply

	default:
		LocoNet.sendLongAck(43); // not yet implemented
		return SV_ERROR;
	}

	encodePeerData(&LnPacket->px, unData.abPlain); // recycling the received packet

	LnPacket->sv.sv_cmd |= 0x40;    // flag the message as reply

	LN_STATUS lnStatus = LocoNet.send(LnPacket, LN_BACKOFF_INITIAL);

#ifdef DEBUG_SV
	Serial.print("LNSV Send Response - Status: ");
	Serial.println(lnStatus);   // report status value from send attempt
#endif

	if (lnStatus != LN_DONE) {
		// failed to send the SV reply message.  Send will NOT be re-tried.
		LocoNet.sendLongAck(44);  // indicate failure to send the reply
	}

	if (LnPacket->sv.sv_cmd == (SV_RECONFIGURE | 0x40))
	{
		if (notifySVReconfigure)
		{
			notifySVReconfigure();
		}
		wdt_enable(WDTO_15MS);  // prepare for reset
		while (1) {}            // stop and wait for watchdog to knock us out
	}

	return SV_OK;
}

SV_STATUS LocoNetSystemVariableClass::doDeferredProcessing(void)
{
	if (DeferredProcessingRequired)
	{
		lnMsg msg;
		SV_Addr_t unData;

		msg.sv.command = (byte)OPC_PEER_XFER;
		msg.sv.mesg_size = (byte)0x10;
		msg.sv.src = DeferredSrcAddr;
		msg.sv.sv_cmd = SV_DISCOVER | (byte)0x40;
		msg.sv.sv_type = (byte)0x02;
		msg.sv.svx1 = (byte)0x10;
		msg.sv.svx2 = (byte)0x10;

		unData.stDecoded.unDestinationId.w = readSVNodeId();
		unData.stDecoded.unMfgIdDevIdOrSvAddress.b.lo = mfgId;
		unData.stDecoded.unMfgIdDevIdOrSvAddress.b.hi = devId;
		unData.stDecoded.unproductId.w = productId;
		unData.stDecoded.unSerialNumber.b.lo = readSVStorage(SV_ADDR_SERIAL_NUMBER_L);
		unData.stDecoded.unSerialNumber.b.hi = readSVStorage(SV_ADDR_SERIAL_NUMBER_H);

		encodePeerData(&msg.px, unData.abPlain);


		/* Note that this operation intentionally uses a "make one attempt to
		   send to LocoNet" method here */
		if (sendLocoNetPacketTry(&msg, LN_BACKOFF_INITIAL + (unData.stDecoded.unSerialNumber.b.lo % (byte)10)) != LN_DONE)
			return SV_DEFERRED_PROCESSING_NEEDED;

		DeferredProcessingRequired = 0;
	}

	return SV_OK;
}

#endif // STM32F1


/*****************************************************************************
*	DESCRIPTION
*	This module provides functions that manage the LNCV-specifiv programming protocol
*
*****************************************************************************/

// Adresses for the 'SRC' part of an UhlenbrockMsg
#define LNCV_SRC_MASTER 0x00
#define LNCV_SRC_KPU 0x01
// KPU is, e.g., an IntelliBox
// 0x02 has no associated meaning
#define LNCV_SRC_TWINBOX_FRED 0x03
#define LNCV_SRC_IBSWITCH 0x04
#define LNCV_SRC_MODULE 0x05

// Adresses for the 'DSTL'/'DSTH' part of an UhlenbrockMsg
#define LNCV_BROADCAST_DSTL 0x00
#define LNCV_BROADCAST_DSTH 0x00
#define LNCV_INTELLIBOX_SPU_DSTL 'I'
#define LNCV_INTELLIBOX_SPU_DSTH 'B'
#define LNCV_INTELLIBOX_KPU_DSTL 'I'
#define LNCV_INTELLIBOX_KPU_DSTH 'K'
#define LNCV_TWINBOX_DSTH 'T'
// For TwinBox, DSTL can be anything from 0 to 15
#define LNCV_IBSWITCH_KPU_DSTL 'I'
#define LNCV_IBSWITCH_KPU_DSTH 'S'
#define LNCV_MODULE_DSTL 0x05
#define LNCV_MODULE_DSTH 0x00

// Request IDs
#define LNCV_REQID_CFGREAD 31
#define LNCV_REQID_CFGWRITE 32
#define LNCV_REQID_CFGREQUEST 33

// Flags for the 7th data Byte
#define LNCV_FLAG_PRON 0x80
#define LNCV_FLAG_PROFF 0x40
#define LNCV_FLAG_RO 0x01
// other flags are currently unused

//#define DEBUG_OUTPUT
#undef DEBUG_OUTPUT

#ifdef DEBUG_OUTPUT
//#define DEBUG(x) Serial.print(F(x))
#define DEBUG(x) Serial.print(x)
#else
#define DEBUG(x)
#endif

#ifdef DEBUG_OUTPUT
void printPacket(lnMsg* LnPacket) {
	Serial.print("LoconetPacket ");
	Serial.print(LnPacket->ub.command, HEX);
	Serial.print(" ");
	Serial.print(LnPacket->ub.mesg_size, HEX);
	Serial.print(" ");
	Serial.print(LnPacket->ub.SRC, HEX);
	Serial.print(" ");
	Serial.print(LnPacket->ub.DSTL, HEX);
	Serial.print(" ");
	Serial.print(LnPacket->ub.DSTH, HEX);
	Serial.print(" ");
	Serial.print(LnPacket->ub.ReqId, HEX);
	Serial.print(" ");
	Serial.print(LnPacket->ub.PXCT1, HEX);
	for (int i(0); i < 7; ++i) {
		Serial.print(" ");
		Serial.print(LnPacket->ub.payload.D[i], HEX);
	}
	Serial.print("\n");
}
#endif

uint8_t LocoNetCVClass::processLNCVMessage(lnMsg* LnPacket) {
	uint8_t ConsumedFlag(0);

	switch (LnPacket->sr.command) {
	case OPC_IMM_PACKET:
	case OPC_PEER_XFER:
		DEBUG("Possibly a LNCV message.");
		// Either of these message types may be a LNCV message
		// Sanity check: Message length, Verify addresses
		if (LnPacket->ub.mesg_size == 15 && LnPacket->ub.DSTL == LNCV_MODULE_DSTL && LnPacket->ub.DSTH == LNCV_MODULE_DSTH) {
			// It is a LNCV programming message
			computeBytesFromPXCT(LnPacket->ub);
#ifdef DEBUG_OUTPUT
			Serial.print("Message bytes: ");
			Serial.print(LnPacket->ub.ReqId);
			Serial.write(" ");
			Serial.print(LnPacket->ub.payload.data.deviceClass, HEX);
			Serial.write(" ");
			Serial.print(LnPacket->ub.payload.data.lncvNumber, HEX);
			Serial.write(" ");
			Serial.print(LnPacket->ub.payload.data.lncvValue, HEX);
			Serial.write("\n");
#endif

			lnMsg response;

			switch (LnPacket->ub.ReqId) {
			case LNCV_REQID_CFGREQUEST:
				if (LnPacket->ub.payload.data.deviceClass == 0xFFFF && LnPacket->ub.payload.data.lncvNumber == 0x0000 && LnPacket->ub.payload.data.lncvValue == 0xFFFF && (LnPacket->ub.payload.data.flags & LNCV_FLAG_PRON) != 0x00) {
					// This is a discover message
					DEBUG("LNCV discover: ");
					if (notifyLNCVdiscover) {
						DEBUG(" executing...");
						if (notifyLNCVdiscover(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue) == LNCV_LACK_OK) {
							makeLNCVresponse(response.ub, LnPacket->ub.SRC, LnPacket->ub.payload.data.deviceClass, 0x00, LnPacket->ub.payload.data.lncvValue, 0x00);
							LocoNet.send(&response);
						}
					}
#ifdef DEBUG_OUTPUT
					else { DEBUG(" NOT EXECUTING!"); }
#endif
				}
				else if (LnPacket->ub.payload.data.flags == 0x00) {
					// This can only be a read message
					DEBUG("LNCV read: ");
					if (notifyLNCVread) {
						DEBUG(" executing...");
						int8_t returnCode(notifyLNCVread(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvNumber, LnPacket->ub.payload.data.lncvValue));
						if (returnCode == LNCV_LACK_OK) {
							// return the read value
							makeLNCVresponse(response.ub, LnPacket->ub.SRC, LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvNumber, LnPacket->ub.payload.data.lncvValue, 0x00); // TODO: D7 was 0x80 here, but spec says that it is unused.
							LocoNet.send(&response);
							ConsumedFlag = 1;
						}
						else if (returnCode >= 0) {
							uint8_t old_opcode(0x7F & LnPacket->ub.command);
							LocoNet.send(OPC_LONG_ACK, old_opcode, returnCode);
							// return a nack
							ConsumedFlag = 1;
						}
					}
#ifdef DEBUG_OUTPUT
					else { DEBUG(" NOT EXECUTING!"); }
#endif
				}
				else {
					// Its a "control" message
					DEBUG("LNCV control: ");
					if ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PRON) != 0x00 && ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PROFF) != 0x00)) {
						DEBUG("Illegal, ignoring.");
						// Illegal message, no action.
					}
					else if ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PRON) != 0x00) {
						DEBUG("Programming Start, ");
						// LNCV PROGAMMING START
						// We'll skip the check whether D[2]/D[3] are 0x0000.
						if (notifyLNCVprogrammingStart) {
							DEBUG(" executing...");
							if (notifyLNCVprogrammingStart(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue) == LNCV_LACK_OK) {
								DEBUG("LNCV_LACK_OK ");
								DEBUG(LnPacket->ub.payload.data.deviceClass);
								DEBUG(" ");
								DEBUG(LnPacket->ub.payload.data.lncvValue);
								DEBUG("\n");
								makeLNCVresponse(response.ub, LnPacket->ub.SRC, LnPacket->ub.payload.data.deviceClass, 0x00, LnPacket->ub.payload.data.lncvValue, 0x80);
								delay(10); // for whatever reason, we need to delay, otherwise the message will not be sent.
#ifdef DEBUG_OUTPUT
								printPacket((lnMsg*)&response);
#endif
								LocoNet.send((lnMsg*)&response);
#ifdef DEBUG_OUTPUT
								LN_STATUS status = LocoNet.send((lnMsg*)&response);
								Serial.print(F("Return Code from Send: "));
								Serial.print(status, HEX);
								Serial.print("\n");
#endif
								ConsumedFlag = 1;
							} // not for us? then no reaction!
#ifdef DEBUG_OUTPUT
							else { DEBUG("Ignoring.\n"); }
#endif
						}
#ifdef DEBUG_OUTPUT
						else { DEBUG(" NOT EXECUTING!"); }
#endif

					}
					if ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PROFF) != 0x00) {
						// LNCV PROGRAMMING END
						if (notifyLNCVprogrammingStop) {
							notifyLNCVprogrammingStop(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue);
							ConsumedFlag = 1;
						}
					}
					// Read-Only mode not implmeneted.
				}

				break;
			case LNCV_REQID_CFGWRITE:
				if (notifyLNCVwrite) {
					// Negative return code indicates that we are not interested in this message.
					int8_t returnCode(notifyLNCVwrite(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvNumber, LnPacket->ub.payload.data.lncvValue));
					if (returnCode >= 0) {
						ConsumedFlag = 1;
						uint8_t old_opcode(0x7F & LnPacket->ub.command);
						LocoNet.send(OPC_LONG_ACK, old_opcode, returnCode);
					}
				}
				break;

			}

		}
		break;
#ifdef DEBUG_OUTPUT
	default:
		Serial.println("Not a LNCV message.");
#endif
	}

	return ConsumedFlag;
}

void LocoNetCVClass::makeLNCVresponse(UhlenbrockMsg& ub, uint8_t originalSource, uint16_t first, uint16_t second, uint16_t third, uint8_t last) {
	ub.command = OPC_PEER_XFER;
	ub.mesg_size = 15;
	ub.SRC = LNCV_SRC_MODULE;
	switch (originalSource) {
	case LNCV_SRC_KPU:
		// Only in case of SRC == 0x01 should something specific be done.
		ub.DSTL = LNCV_INTELLIBOX_KPU_DSTL;
		ub.DSTH = LNCV_INTELLIBOX_KPU_DSTH;
		break;
	default:
		ub.DSTL = originalSource;
		ub.DSTH = 0x00;
	}
	ub.ReqId = LNCV_REQID_CFGREAD;
	ub.PXCT1 = 0x00;
	ub.payload.data.deviceClass = first;
	ub.payload.data.lncvNumber = second;
	ub.payload.data.lncvValue = third;
	ub.payload.data.flags = last;
	computePXCTFromBytes(ub);
}


void LocoNetCVClass::computeBytesFromPXCT(UhlenbrockMsg& ub) {
	uint8_t mask(0x01);
	// Data has only 7 bytes, so we consider only 7 bits from PXCT1
	for (int i(0); i < 7; ++i) {
		if ((ub.PXCT1 & mask) != 0x00) {
			// Bit was set
			ub.payload.D[i] |= 0x80;
		}
		mask <<= 1;
	}
	ub.PXCT1 = 0x00;
}

void LocoNetCVClass::computePXCTFromBytes(UhlenbrockMsg& ub) {
	uint8_t mask(0x01);
	ub.PXCT1 = 0x00;
	// Data has only 7 bytes, so we consider only 7 bits from PXCT1
	for (int i(0); i < 7; ++i) {
		if ((ub.payload.D[i] & 0x80) != 0x00) {
			ub.PXCT1 |= mask; // add bit to PXCT1
			ub.payload.D[i] &= 0x7F;	// remove bit from data byte
		}
		mask <<= 1;
	}
}

uint16_t LocoNetCVClass::getAddress(uint8_t lower, uint8_t higher) {
	uint16_t result(higher);
	result <<= 8;
	result |= lower;
	return result;
}


