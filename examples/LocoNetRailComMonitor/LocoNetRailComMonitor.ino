#include <LocoNet.h>

// LocoNet RailCom Monitor
// Demonstrates the use of the:
//
// OPC_MULTI_SENSE transponder call-back
//
// Tested on a Uhlenbrock 68500 MARCo-Empfänger in Digitrax Mode
// LNCV 15 == 2 - Send ÜF Digitrax with Locomotive address and Block status (vacant/occupied) 

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
  if( LnPacket ) {
      // First print out the packet in HEX
    Serial.print("RX: ");
    uint8_t msgLen = getLnMsgSize(LnPacket); 
    for (uint8_t x = 0; x < msgLen; x++)
    {
      uint8_t val = LnPacket->data[x];
        // Print a leading 0 if less than 16 to make 2 HEX digits
      if(val < 16)
        Serial.print('0');
        
      Serial.print(val, HEX);
      Serial.print(' ');
    }
    
      // If this packet was not a Switch or Sensor Message then print a new line 
    if(!LocoNet.processSwitchSensorMessage(LnPacket)) {
      Serial.println();
    }
  }
}


  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for OPC_MULTI_SENSE 0xD0
void notifyMultiSenseTransponder( uint16_t Address, uint8_t Zone, uint16_t LocoAddress, uint8_t Present ) {
  Serial.print("Railcom Sensor ");
  Serial.print(Address);
  Serial.print(" reports ");
  Serial.print(Present? "present" : "absent");
  Serial.print(" of decoder address ");
  Serial.print(LocoAddress, DEC);
  Serial.print(" in zone ");
  Serial.println(Zone, HEX);
}
