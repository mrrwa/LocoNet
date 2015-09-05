#include <LocoNet.h>

#define  TX_PIN 7

LocoNetSystemVariableClass sv;
lnMsg       *LnPacket;
SV_STATUS   svStatus = SV_OK;
boolean     deferredProcessingNeeded = false;

void setup() {
  LocoNet.init(TX_PIN); 
  sv.init(13, 4, 1, 1);
  
  sv.writeSVStorage(SV_ADDR_NODE_ID_H, 1 );
  sv.writeSVStorage(SV_ADDR_NODE_ID_L, 0);
  
  sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, 0x56);
  sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_L, 0x78);
  
  Serial.begin(115200);
  Serial.println("LocoNet SV Programming Test");
}

void loop() {
  SV_STATUS svStatus;
  LnPacket = LocoNet.receive();
  if( LnPacket )
  {
      // First print out the packet in HEX
    Serial.print("\nRX: ");
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
    Serial.println();
    
    svStatus = sv.processMessage(LnPacket);
    Serial.print("LNSV processMessage - Status: ");
    Serial.println(svStatus);
    
    deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);
  }
  
  if(deferredProcessingNeeded)
    deferredProcessingNeeded = (sv.doDeferredProcessing() != SV_OK);
}

void notifySVChanged(uint16_t Offset){
  Serial.print("LNSV SV: ");
  Serial.print(Offset);
  Serial.print(" Changed to: ");
  Serial.println(sv.readSVStorage(Offset));
}

