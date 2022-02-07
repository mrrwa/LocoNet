#include <LocoNet.h>

// LocoNet Packet Monitor
// Demonstrates the use of the:
//
//   LocoNet.processSwitchSensorMessage(LnPacket)
//
// function and examples of each of the notifyXXXXXXX user call-back functions

lnMsg        *LnPacket;

void setup() {
  // First initialize the LocoNet interface
  LocoNet.init();

  // Configure the serial port for 57600 baud
  Serial.begin(57600);
  Serial.println("LocoNet Monitor");
}

void loop() {
  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if ( LnPacket ) {
    // First print out the packet in HEX
    Serial.print("RX: ");
    uint8_t msgLen = getLnMsgSize(LnPacket);
    for (uint8_t x = 0; x < msgLen; x++)
    {
      uint8_t val = LnPacket->data[x];
      // Print a leading 0 if less than 16 to make 2 HEX digits
      if (val < 16)
        Serial.print('0');

      Serial.print(val, HEX);
      Serial.print(' ');
    }

    // If this packet was not a Switch or Sensor Message then print a new line
    if (!LocoNet.processSwitchSensorMessage(LnPacket)) {
      Serial.println();
    }
  }
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages
void notifySensor( uint16_t Address, uint8_t State ) {
  Serial.print("Sensor: ");
  Serial.print(Address, DEC);
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  Serial.print("Switch Request: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Output Report messages
void notifySwitchOutputsReport( uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
  Serial.print("Switch Outputs Report: ");
  Serial.print(Address, DEC);
  Serial.print(": Closed - ");
  Serial.print(ClosedOutput ? "On" : "Off");
  Serial.print(": Thrown - ");
  Serial.println(ThrownOutput ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Sensor Report messages
void notifySwitchReport( uint16_t Address, uint8_t State, uint8_t Sensor ) {
  Serial.print("Switch Sensor Report: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Sensor ? "Switch" : "Aux");
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch State messages
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  Serial.print("Switch State: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Power messages
void notifyPower(uint8_t State) {
  Serial.print("Layout Power State: ");
  Serial.println(State ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all MultiSensePower messages
void notifyMultiSensePower(uint8_t BoardID, uint8_t Subdistrict, uint8_t Mode, uint8_t Direction) {
  Serial.print("MultiSensePower: Board ID: ");
  Serial.print(BoardID, DEC);
  Serial.print(" Sub District: ");
  Serial.print(Subdistrict, DEC);
  Serial.print(" Mode: ");
  Serial.print(Mode, DEC);
  Serial.print(" Direction: ");
  Serial.println(Direction, DEC);
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all notifyMultiSenseTransponder messages
void notifyMultiSenseTransponder(uint16_t Address, uint8_t Zone, uint16_t LocoAddress, uint8_t Present) {
  Serial.print("MultiSenseTransponder: Address: ");
  Serial.print(Address, DEC);
  Serial.print(" Zone: ");
  Serial.print(Zone, DEC);
  Serial.print(" Loco Address: ");
  Serial.print(LocoAddress, DEC);
  Serial.print(" Present: ");
  Serial.println(Present, DEC);
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all LongAck messages

void notifyLongAck(uint8_t d1, uint8_t d2) {
  Serial.print("LongACK : Data Byte 1: ");
  Serial.print(d1, DEC);
  Serial.print(" Data Byte 2: ");
  Serial.println(d2, DEC);

}
