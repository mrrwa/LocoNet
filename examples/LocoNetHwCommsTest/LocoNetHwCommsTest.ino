#include <LocoNet.h>

// LocoNet Hardware Communication Test sketch
//
// This sketch can be used to test the normal operation of the LocoNet Comms Hardware by printing out any received LocoNet messages
// and sending a Report Sensor message with an incrementing address every second and prints the send message status

// Define your LocoNet TX Pin below
#define  TX_PIN   7

// Uncomment the line below to enable periodicall sending an incrementing LocoNet Sensor Message
//#define SEND_TEST_MESSAGE_PERIOD_MS 1000

static   lnMsg        *LnPacket;

void setup()
{
  Serial.begin(115200);
  
  while (!Serial);  // Wait for the USB Serial Port to initialise 

  Serial.println("LocoNet Hardware Communication Test");
  
    // First initialize the LocoNet interface, specifying the TX Pin
  LocoNet.init(TX_PIN);
}

boolean isTime(unsigned long *timeMark, unsigned long timeInterval)
{
    unsigned long timeNow = millis();
    if ( timeNow - *timeMark >= timeInterval) {
        *timeMark = timeNow;
        return true;
    }    
    return false;
}

#ifdef SEND_TEST_MESSAGE_PERIOD_MS
unsigned long lastTxTime = 0;
uint16_t lastSensorAddr = 0;
#endif

void loop()
{  
    // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket )
  {
      // Get the length of the received packet
    uint8_t msgLen = getLnMsgSize( LnPacket ) ;
    Serial.print("\nRx: ");
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
    if (!LocoNet.processSwitchSensorMessage(LnPacket))
      Serial.println();
  }

#ifdef SEND_TEST_MESSAGE_PERIOD_MS
  // Send a Report Sensor Message every second with an incrementing address
  if( isTime(&lastTxTime, 1000))
  {
    if(++lastSensorAddr >= 1000)
      lastSensorAddr = 0;

    LN_STATUS lnStatus = LocoNet.reportSensor(lastSensorAddr, 1);

    Serial.print("Tx: Sensor: ");
    Serial.print(lastSensorAddr);
    Serial.print(" Status: ");
    Serial.println(LocoNet.getStatusStr(lnStatus));
  }
#endif  
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
