// Arduino - AVR ATmega32U4 based USB LocoNet PC Interface Demo

#if !defined(ARDUINO_ARCH_AVR) and !defined(ARDUINO_MEGA_32U4)
#error "This sketch only supports the AVR ATmega32U4 processor as it relies on specific USB Port functionality"
#endif 

#include <LocoNet.h>

// There are a number of ATmega32U4 boards available but each may have different pin numbers for on-baord LEDs.
//
// This example sketch was developed using the Sparkfun Arduino Pro Mico (see: https://www.sparkfun.com/products/12640) 
// and a small LocoNet interface shield designed by John Plocher specifically to match up with the Arduino Pro Micro footprint
// that can be found here: http://www.spcoast.com/wiki/index.php/Core-Locobuffer

// The LocoNet Tx/Rx traffic can be indicated using some LEDs. The default values below are for the Sparkfun Arduino Pro Micro board.
// If your board doesn't have LEDs or you don't want to use them, comment out the two #define lines below
#define TX_LED  30
#define RX_LED  17

#if defined(TX_LED) or defined(RX_LED)
// If you're using LEDs to indicate LocoNet traffic then define the HIGH/LOW state to turn the LEDs ON/OFF in the two lines below
#define LED_ON  LOW
#define LED_OFF HIGH

// These elapsedMillis timers are used to turn off the Tx/Rx LEDs after a short delay, so they're only needed when the LEDs are used
#include <elapsedMillis.h>  // See http://playground.arduino.cc/Code/ElapsedMillis
elapsedMillis  txElapsedMillis;
elapsedMillis  rxElapsedMillis;

#endif

#define VERSION 2

// The Rx Pin on the ATmega32U4 always uses the ICP input so that is fixed and enabled in the Library.
// However the Tx pin can be any other available pin on the board.
// For the Sparkfun Arduino Pro Micro and the Core-Locobuffer shield it uses Digital pin 7

#define  TX_PIN 7

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

#ifdef RX_LED
  pinMode(RX_LED, OUTPUT);
  digitalWrite(RX_LED, LED_OFF);
#endif

#ifdef TX_LED
  pinMode(TX_LED, OUTPUT);
  digitalWrite(TX_LED, LED_OFF);
#endif
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
#ifdef RX_LED
    digitalWrite(RX_LED, LED_ON);
    rxElapsedMillis = 0;
#endif    
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
      
#ifdef TX_LED
      digitalWrite(TX_LED, LED_ON);
      txElapsedMillis = 0;
#endif      
    }
  }

#ifdef RX_LED
  if(rxElapsedMillis > 50)
    digitalWrite(RX_LED, LED_OFF);
#endif

#ifdef TX_LED
  if(txElapsedMillis > 50)
    digitalWrite(TX_LED, LED_OFF);
#endif    
}
