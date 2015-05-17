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
	Serial.begin(57600);
	Serial.println("LocoNet Fast Clock Demo");

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
		// Serial.print('|');
		FastClock.processMessage(LnPacket);
	}

	if(isTime(&LastFastClockTick, 67)) {
	    FastClock.process66msActions(); 
	}
}

void notifyFastClock( uint8_t Rate, uint8_t Day, uint8_t Hour, uint8_t Minute, uint8_t Sync ) {
	Serial.print("Rate: "); Serial.print(Rate, DEC);
	Serial.print(" Day: "); Serial.print(Day, DEC);
	Serial.print(" Hour: "); Serial.print(Hour, DEC);
	Serial.print(" Min: "); Serial.print(Minute, DEC);
	Serial.print(" Sync: "); Serial.println(Sync, DEC);
}

void notifyFastClockFracMins( uint16_t FracMins ) {
    //  Serial.print('.');
}

