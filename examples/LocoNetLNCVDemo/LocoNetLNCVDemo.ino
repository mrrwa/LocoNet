#include <LocoNet.h>

/******************************************************************************
 *
 *  Copyright (C) 2013 Damian Philipp
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************
 *
 * DESCRIPTION
 *
 * This is a Demo for the LNCV functionality. A device running under this code
 * will be LNCV-programmable at Hardware Address 50010, initial module Address 1.
 * You can freely read and write LNCV values 0 through 15. Everything is stored
 * in RAM, thus all settings are lost after a reset of the Arduino.
 *
 * Use this code as an example on how to use LNCV programming.
 *
 ******************************************************************************/

#define LOCONET_TX_PIN 7

#define LNCV_COUNT 16

// Art.-Nr.: 50010
#define ARTNR 5001

//uint16_t moduleAddr;
uint16_t lncv[LNCV_COUNT];

lnMsg *LnPacket;

LocoNetCVClass lnCV;

boolean programmingMode;

void setup() {
	LocoNet.init(LOCONET_TX_PIN);
	Serial.begin(115200);
	Serial.print("Starting LNCV-test\n");
	lncv[0] = 1;
	for (int i(1); i < LNCV_COUNT; ++i) {
		lncv[i] = i;
	}
	commitLNCVUpdate();
	programmingMode = false;
}

void commitLNCVUpdate() {
	Serial.print("Module Address is now: ");
	Serial.print(lncv[0]);
	Serial.print("\n");
}

void loop() {
	LnPacket = LocoNet.receive();
	if (LnPacket) {
		uint8_t packetConsumed(LocoNet.processSwitchSensorMessage(LnPacket));
		if (packetConsumed == 0) {
			Serial.print("Loop ");
			Serial.print((int)LnPacket);
			dumpPacket(LnPacket->ub);
			packetConsumed = lnCV.processLNCVMessage(LnPacket);
			Serial.print("End Loop\n");
		}
	}
}

void dumpPacket(UhlenbrockMsg & ub) {
	Serial.print(" PKT: ");
	Serial.print(ub.command, HEX);
	Serial.print(" ");
	Serial.print(ub.mesg_size, HEX);
	Serial.print(" ");
	Serial.print(ub.SRC, HEX);
	Serial.print(" ");
	Serial.print(ub.DSTL, HEX);
	Serial.print(" ");
	Serial.print(ub.DSTH, HEX);
	Serial.print(" ");
	Serial.print(ub.ReqId, HEX);
	Serial.print(" ");
	Serial.print(ub.PXCT1, HEX);
	Serial.print(" ");
	for (int i(0); i < 8; ++i) {
		Serial.print(ub.D[i], HEX);
		Serial.print(" ");
	}
	Serial.write("\n");
}

	/**
	 * Notifies the code on the reception of a read request.
	 * Note that without application knowledge (i.e., art.-nr., module address
	 * and "Programming Mode" state), it is not possible to distinguish
	 * a read request from a programming start request message.
	 */
int8_t notifyLNCVread(uint16_t ArtNr, uint16_t lncvAddress, uint16_t,
		uint16_t & lncvValue) {
	Serial.print("Enter notifyLNCVread(");
	Serial.print(ArtNr, HEX);
	Serial.print(", ");
	Serial.print(lncvAddress, HEX);
	Serial.print(", ");
	Serial.print(", ");
	Serial.print(lncvValue, HEX);
	Serial.print(")");
	// Step 1: Can this be addressed to me?
	// All ReadRequests contain the ARTNR. For starting programming, we do not accept the broadcast address.
			if (programmingMode) {
				if (ArtNr == ARTNR) {
					if (lncvAddress < 16) {
						lncvValue = lncv[lncvAddress];
						Serial.print(" LNCV Value: ");
						Serial.print(lncvValue);
						Serial.print("\n");
						return LNCV_LACK_OK;
					} else {
						// Invalid LNCV address, request a NAXK
						return LNCV_LACK_ERROR_UNSUPPORTED;
					}
				} else {
					Serial.print("ArtNr invalid.\n");
					return -1;
				}
			} else {
				Serial.print("Ignoring Request.\n");
				return -1;
			}
		}

int8_t notifyLNCVprogrammingStart(uint16_t & ArtNr, uint16_t & ModuleAddress) {
	// Enter programming mode. If we already are in programming mode,
	// we simply send a response and nothing else happens.
	Serial.print("notifyLNCVProgrammingStart ");
	if (ArtNr == ARTNR) {
		Serial.print("artnrOK ");
		if (ModuleAddress == lncv[0]) {
			Serial.print("moduleUNI ENTERING PROGRAMMING MODE\n");
			programmingMode = true;
			return LNCV_LACK_OK;
		} else if (ModuleAddress == 0xFFFF) {
			Serial.print("moduleBC ENTERING PROGRAMMING MODE\n");
			ModuleAddress = lncv[0];
			return LNCV_LACK_OK;
		}
	}
	Serial.print("Ignoring Request.\n");
	return -1;
}

	/**
	 * Notifies the code on the reception of a write request
	 */
int8_t notifyLNCVwrite(uint16_t ArtNr, uint16_t lncvAddress,
		uint16_t lncvValue) {
	Serial.print("notifyLNCVwrite, ");
	//  dumpPacket(ub);
			if (!programmingMode) {
				Serial.print("not in Programming Mode.\n");
				return -1;
			}

			if (ArtNr == ARTNR) {
				Serial.print("Artnr OK, ");

				if (lncvAddress < 16) {
					lncv[lncvAddress] = lncvValue;
					return LNCV_LACK_OK;
				}
				else {
					return LNCV_LACK_ERROR_UNSUPPORTED;
				}

			}
			else {
				Serial.print("Artnr Invalid.\n");
				return -1;
			}
		}

	/**
	 * Notifies the code on the reception of a request to end programming mode
	 */
void notifyLNCVprogrammingStop(uint16_t ArtNr, uint16_t ModuleAddress) {
	Serial.print("notifyLNCVprogrammingStop ");
	if (programmingMode) {
		if (ArtNr == ARTNR && ModuleAddress == lncv[0]) {
			programmingMode = false;
			Serial.print("End Programing Mode.\n");
			commitLNCVUpdate();
		}
		else {
			if (ArtNr != ARTNR) {
				Serial.print("Wrong Artnr.\n");
				return;
			}
			if (ModuleAddress != lncv[0]) {
				Serial.print("Wrong Module Address.\n");
				return;
			}
		}
	}
	else {
		Serial.print("Ignoring Request.\n");
	}
}

