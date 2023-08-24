#include <LocoNet.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ESPmDNS.h>

// LocoNet.TCP Packet Monitor
// opens a TCP server socket at port 1001 for Loconet communication
// you may use a virtual serial port that forwards data to the TCP socket (e.g. VSP Manager https://tibbo.com/soi/software.html)
// or directly connect to LoconetServer.local:1001

// In Rocrail use Interface "LocoNet", Device "LocoNetServer.local:1001", Typ LocoBuffer, Uncheck CTS Flow

// Demonstrates the use of the:
//
//   LocoNet.processSwitchSensorMessage(LnPacket)
//
// function and examples of each of the notifyXXXXXXX user call-back functions

lnMsg        *LnPacket;
#if !defined(LOCONET_NO_EEPROM)
LocoNetSystemVariableClass LocoNetSV;
#endif
LocoNetFastClockClass LocoFCClass;
SV_STATUS SvStatus ;
bool powerIsOn = false;

// ESP32 WIFI 
WiFiManager wm; // global wm instance
WiFiServer locoNetServer;
WiFiClient locoNetClient;
bool isConnected = false;
String hostString;

// you may ping the server with DNSname 'LocoNetServer.local'
#define WIFI_DNS_NAME "LocoNetServer"

void handlePacket(lnMsg *LnPacket);

void setup() {
  Serial.begin(57600);
  Serial.println("LocoNet.TCP Monitor for ESP32");

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  wm.setConfigPortalBlocking(false);
  wm.setTitle(String(WIFI_DNS_NAME).c_str());

  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  //generate the accespoint name

  hostString = String(WIFI_getChipId(),HEX);
  hostString.toUpperCase();
  hostString = "MLL_" + hostString;
  wm.setHostname(hostString.c_str());

  bool connected = false;

  for (int i=0;i<2&& !connected;i++)
  {
    connected = wm.autoConnect(hostString.c_str());
  }
  setConnected(connected);

  if (connected)
  {
    Serial.println("connected");
  }
  else
  {
    Serial.println("local AP started");
  }

  // initialize the LocoNet interface
  LocoNet.init();
  locoNetServer.begin(1001);
}

void loop() {
    wm.process();

  if (!locoNetClient) locoNetClient = locoNetServer.available();  
  if (locoNetClient) {
    if (!locoNetClient.connected())
    {
      locoNetClient.stop();
      printf("LocoNetClient disconnected\n");
      return;
    }
  }

    while (locoNetClient.available()>0) {
      LocoNet.addToBuffer(locoNetClient.read());
    
    // Check for any received LocoNet packets
    lnMsg* LnPacket = LocoNet.receive() ;
    if ( LnPacket ) {
      handlePacket(LnPacket);
    }
  }
}

bool sendRawLocoNet(uint8_t val)
{
  return locoNetClient.write(val)==1;
}

void handlePacket(lnMsg *LnPacket)
{
  // first mirror the message
  LocoNet.send(LnPacket);

  // then print out the packet in HEX
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
  
#if !defined(LOCONET_NO_EEPROM)
  if( LocoNetSV.processMessage( LnPacket ) == SV_DEFERRED_PROCESSING_NEEDED)
    SvStatus = SV_DEFERRED_PROCESSING_NEEDED;

  if(SvStatus == SV_DEFERRED_PROCESSING_NEEDED)
    SvStatus = LocoNetSV.doDeferredProcessing();
#endif

  LocoFCClass.processMessage( LnPacket );

  if (LnPacket->sr.command==OPC_RQ_SL_DATA)
  {
    lnMsg SendPacket;
    SendPacket.data[ 0 ] = OPC_SL_RD_DATA;
    SendPacket.data[ 1 ] = 0x0e;
    SendPacket.data[ 2 ] = 3;  
    SendPacket.data[ 3 ] = GTRK_POWER;
    SendPacket.data[ 4 ] = 0;
    SendPacket.data[ 5 ] = 0;
    SendPacket.data[ 6 ] = 0;
    SendPacket.data[ 7 ] = 0;
    SendPacket.data[ 8 ] = 0;
    SendPacket.data[ 9 ] = 0;
    SendPacket.data[ 10 ] = 0;
    SendPacket.data[ 11 ] = 0;
    SendPacket.data[ 12 ] = 0;
    SendPacket.data[ 13 ] = 0;
    LocoNet.send( &SendPacket );
  }

  // Process the packet in case its a OPC_GPON
  LocoNet.processSwitchSensorMessage(LnPacket);
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

void setConnected(bool connected)
{
  if (connected==isConnected) return;   // connection state didn't change

  isConnected=connected;
  if (isConnected)
  {
    Serial.println("starting config portal");
    wm.startConfigPortal(hostString.c_str());
    Serial.println("starting config portal done");

    if(!MDNS.begin(WIFI_DNS_NAME)) {
       Serial.println("Error starting mDNS");
    }
  }
}
