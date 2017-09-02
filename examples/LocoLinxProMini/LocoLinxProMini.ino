// Arduino - Leonardo based USB LocoNet PC Interface Demo

#include <LocoNet.h>
#include <elapsedMillis.h>  // See http://playground.arduino.cc/Code/ElapsedMillis

elapsedMillis  txElapsedMillis;
elapsedMillis  rxElapsedMillis;

#define TX_LED  30
#define RX_LED  17
#define LED_ON  LOW
#define LED_OFF HIGH

#define VERSION 2

// Rx Pin on the LeoStick is D4
#define  TX_PIN 5

static   LnBuf        LnTxBuffer ;
static   lnMsg        *LnTxPacket;
static   lnMsg        *LnRxPacket;
static   lnMsg        *LnStatsPacket;
static   lnMsg        myStats;


  // Format a LocoBuffer II Status Response
void updateStats()
{
  
  memset(&myStats, 0, sizeof(peerXferMsg));
  
  myStats.data[ 0] = OPC_PEER_XFER;
  myStats.data[ 1] = 0x10;
  myStats.data[ 2] = 0x50;
  myStats.data[ 3] = 0x50;
  myStats.data[ 4] = 0x01;

  LnBufStats* pLnStats = LocoNet.getStats();
  long Errors = pLnStats->RxErrors + pLnStats->TxErrors;

  uint8_t myStatsData[8];

  myStatsData[0] = 0x00;
  myStatsData[1] = (Errors >> 16) & 0xFF;
  myStatsData[2] = (Errors >>  8) & 0xFF;
  myStatsData[3] = Errors & 0xFF;
  
  myStatsData[4] = VERSION;
  myStatsData[5] = 0x00;
  myStatsData[6] = (pLnStats->Collisions >> 8) & 0xFF ;
  myStatsData[7] = pLnStats->Collisions & 0xFF;

  encodePeerData(&myStats.px, myStatsData);

  uint8_t CheckSum = 0xFF ;

  for( uint8_t lnTxIndex = 0; lnTxIndex < sizeof(peerXferMsg) - 1; lnTxIndex++ ) {
    CheckSum ^= myStats.data[ lnTxIndex ] ;
  }

  myStats.data[sizeof(peerXferMsg) - 1] = CheckSum ; 

  LnStatsPacket = &myStats;
}

void setup()
{
    // Configure the serial port for 57600 baud, even though the baud rate is ignored for USB interfaces
  Serial.begin(57600);
  
  while (!Serial); // Wait for USB Serial port to connect - needed for native USB  
  
    // First initialize the LocoNet interface, specifying the TX Pin
  LocoNet.init(TX_PIN);
  
    // Initialize a LocoNet packet buffer to buffer bytes from the PC 
  initLnBuf(&LnTxBuffer) ;

    // Set this to NULL so we know we don't have an unsent LocoNet packet 
  LnRxPacket = NULL;
  LnTxPacket = NULL;
  LnStatsPacket = NULL;

  pinMode(RX_LED, OUTPUT);
  digitalWrite(RX_LED, LED_OFF);

  pinMode(TX_LED, OUTPUT);
  digitalWrite(TX_LED, LED_OFF);
}

void loop()
{  
    // Before we check for a new LocoNet packet, make sure we haven't already got a previously unset packet
  if(LnRxPacket == NULL)
  {
    if(LnStatsPacket)
    {
      LnRxPacket = LnStatsPacket;
      LnStatsPacket = NULL;
    }
    
    else
      LnRxPacket = LocoNet.receive() ;
  }

  if( LnRxPacket )
  {
    digitalWrite(RX_LED, LED_ON);
    rxElapsedMillis = 0;
    
      // Get the length of the received packet
    uint8_t Length = getLnMsgSize( LnRxPacket ) ;

    uint8_t USBWriteBufferFree = Serial.availableForWrite();
    if( USBWriteBufferFree >= Length)
    {
      Serial.write((uint8_t*)LnRxPacket, Length);
      LnRxPacket = NULL;
    }
  }

    // Before we check for a new LocoNet TX packet, make sure we haven't already got a previously unset packet
  if(LnTxPacket == NULL)
  {
    int charWaiting;
    
      // Check to see if there are any bytes from the PC
    while( (charWaiting = Serial.available()) && (LnTxPacket == NULL) )
    {
        // Read the byte
      uint8_t inByte = Serial.read() & 0xFF;
      
        // Add it to the buffer
      addByteLnBuf( &LnTxBuffer, inByte ) ;
      
        // Check to see if we have received a complete packet yet
      LnTxPacket = recvLnMsg( &LnTxBuffer ) ;
    }
  }
    
    // Send the received packet from the PC to the LocoNet
  if(LnTxPacket )
  {
      // Check for a Request for LocoNet Stats
    if(LnTxPacket->data[0] == OPC_BUSY)
    {
      LnTxPacket = NULL;
      updateStats();
    }
    
    else if(LocoNet.send( LnTxPacket ) == LN_DONE)
    {
      LnTxPacket = NULL;
      
      digitalWrite(TX_LED, LED_ON);
      txElapsedMillis = 0;
    }
  }

  if(rxElapsedMillis > 50)
    digitalWrite(RX_LED, LED_OFF);

  if(txElapsedMillis > 50)
    digitalWrite(TX_LED, LED_OFF);
}
