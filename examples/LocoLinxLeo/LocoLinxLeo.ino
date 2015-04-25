// Arduino - Leonardo based USB LocoNet PC Interface Demo

#include <LocoNet.h>
#include <elapsedMillis.h>  // See http://playground.arduino.cc/Code/ElapsedMillis

elapsedMillis  txElapsedMillis;
elapsedMillis  rxElapsedMillis;

#define TX_LED 13
#define RX_LED  9

#define  TX_PIN 5

static   LnBuf        LnTxBuffer ;
static   lnMsg        *LnPacket;

void setup()
{
    // Configure the serial port for 57600 baud, even though the baud rate is ignored for USB interfaces
  Serial.begin(57600);  
  
    // First initialize the LocoNet interface, specifying the TX Pin
  LocoNet.init(TX_PIN);
  
    // Initialize a LocoNet packet buffer to buffer bytes from the PC 
  initLnBuf(&LnTxBuffer) ;

  pinMode(RX_LED, OUTPUT);
  digitalWrite(RX_LED, LOW);

  pinMode(TX_LED, OUTPUT);
  digitalWrite(TX_LED, LOW);
}

void loop()
{  
    // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket )
  {
    digitalWrite(RX_LED, HIGH);
    rxElapsedMillis = 0;
    
      // Get the length of the received packet
    uint8_t Length = getLnMsgSize( LnPacket ) ;

      // Send the received packet out byte by byte to the PC
    for( uint8_t Index = 0; Index < Length; Index++ )
      Serial.write(LnPacket->data[ Index ]);
  }

    // Check to see if there are any bytes from the PC
  if(int charWaiting = Serial.available())
  {
      // Read the byte
    uint8_t inByte = Serial.read() & 0xFF;
    
      // Add it to the buffer
    addByteLnBuf( &LnTxBuffer, inByte ) ;
    
      // Check to see if we have received a complete packet yet
    LnPacket = recvLnMsg( &LnTxBuffer ) ;
    if(LnPacket )
        // Send the received packet from the PC to the LocoNet
      LocoNet.send( LnPacket ) ;

      digitalWrite(TX_LED, HIGH);
      txElapsedMillis = 0;
  }

  if(rxElapsedMillis > 50)
    digitalWrite(RX_LED, LOW);

  if(txElapsedMillis > 50)
    digitalWrite(TX_LED, LOW);
}
