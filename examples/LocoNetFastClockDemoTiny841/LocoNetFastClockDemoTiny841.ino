// LocoNet FastClock Demo

#include <LocoNet.h>

LocoNetFastClockClass  FastClock ;

static   lnMsg    *LnPacket;
unsigned long     LastFastClockTick;

boolean isTime(unsigned long *timeMark, unsigned long timeInterval) {
  unsigned long timeNow = millis();
  if ( timeNow - *timeMark >= timeInterval) {
    *timeMark += timeInterval;
    return true;
  }    
  return false;
}

void setup() {
  Serial1.begin(115200);
  Serial1.println("LocoNet Fast Clock Demo");

  // Initialize the LocoNet interface
  LocoNet.init(4);

  // Initialize the Fast Clock
  FastClock.init(0, 1, 1);

  // Poll the Current Time from the Command Station
  FastClock.poll();
}

void loop() {  
  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket ) {
//    Serial1.print('|');
    FastClock.processMessage(LnPacket);
  }

  if(isTime(&LastFastClockTick, 67)) {
    FastClock.process66msActions(); 
  }
}

void notifyFastClock( uint8_t Rate, uint8_t Day, uint8_t Hour, uint8_t Minute, uint8_t Sync ) {
  Serial1.print("Rate: "); Serial1.print(Rate, DEC);
  Serial1.print(" Day: "); Serial1.print(Day, DEC);
  Serial1.print(" Hour: "); Serial1.print(Hour, DEC);
  Serial1.print(" Min: "); Serial1.print(Minute, DEC);
  Serial1.print(" Sync: "); Serial1.println(Sync, DEC);
}

void notifyFastClockFracMins( uint16_t FracMins ) {
//  Serial1.print('.');
}

