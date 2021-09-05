/*****************************************************************************

 Title :   LocoNet Throttle F9-F28 tester
 Author:   Enrico Mattioli <@>
 Date:     5-Sep-2021
 Target:   ATMega328 Arduino

 DESCRIPTION
  This project is intended to check the correct woking of the F9-F28 function support.
  The function will first enable functions F1-F28, one after another with a pause of DELAY_MILLIS.
  Then all functions will be disabled in reverse order.
  When done, all functions will be reset using LocoNetThrottleClass::resetAllFunctions().
  Loco addr is LOCO_ADDR.
  You may also set functions using an external throttle and check if the changes are correctly register (see Serial Terminal)

*****************************************************************************/


#include <LocoNet.h>

uint16_t          LocoAddr ;

LocoNetThrottleClass  Throttle;
lnMsg                 *RxPacket ;
uint32_t              LastThrottleTimerTick;

#define LOCO_ADDR 3
#define DELAY_MILLIS 1000

boolean isTime(unsigned long *timeMark, unsigned long timeInterval)
{
    unsigned long timeNow = millis();
    if ( timeNow - *timeMark >= timeInterval) {
        *timeMark = timeNow;
        return true;
    }    
    return false;
}

void setup()
{
  // First initialize the LocoNet interface
  LocoNet.init(7);

  // Configure the serial port for 57600 baud
  Serial.begin(115200);

  // throttle
  Throttle.init(0, 0, 0);
  Throttle.stealAddress(LOCO_ADDR);
  Throttle.setAddress(LOCO_ADDR);
  Throttle.acquireAddress();
}

void loop()
{  
  // Check for any received LocoNet packets
  RxPacket = LocoNet.receive() ;

  // process packets
  if( RxPacket )
  {
    digitalWrite(13, LOW);
    
    if( !LocoNet.processSwitchSensorMessage(RxPacket) )
      Throttle.processMessage(RxPacket) ;
  }

  // 100ms updates
  if(isTime(&LastThrottleTimerTick, 100))
  {
    Throttle.process100msActions() ;
    digitalWrite(13, HIGH);
  }

  function_set_unset();
}

void function_set_unset()
{
  static byte fn=0;
  static bool count_up = true;
  static unsigned long nextMillis=0;

  if(millis()>nextMillis)
  {
      if (fn==0)
      {
          Throttle.resetAllFunctions();
          count_up = true;
      }
      else if (fn>28)
      {
          count_up = false;
      }
      else
      {
          Throttle.setFunction(fn, count_up?1:0);
      }

      fn += count_up?1:-1;
      
      nextMillis=millis()+DELAY_MILLIS;
  }
}

// Throttle notify Call-back functions
void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t Slot )
{
  Serial.print("Address: ");Serial.println(Address, DEC);
}
void notifyThrottleSpeed( uint8_t UserData, TH_STATE State, uint8_t Speed )
{
  Serial.print("Speed: ");Serial.println(Speed, DEC);
}
void notifyThrottleDirection( uint8_t UserData, TH_STATE State, uint8_t Direction )
{
  Serial.print("Direction: ");Serial.println(Direction, DEC);
}
void notifyThrottleFunction( uint8_t UserData, uint8_t Function, uint8_t Value )
{
  Serial.print("Function: ");Serial.print(Function, DEC);Serial.print(" = ");Serial.println(Value, DEC);
}
void notifyThrottleSlotStatus( uint8_t UserData, uint8_t Status )
{
  Serial.print("Throttle slot status: ");Serial.println(Status, DEC);
}
void notifyThrottleSpeedSteps( uint8_t UserData, TH_SPEED_STEPS SpeedSteps )
{
  Serial.print("Speed steps: ");Serial.println(Throttle.getSpeedStepStr(SpeedSteps));
}
void notifyThrottleError( uint8_t UserData, TH_ERROR Error )
{
  Serial.print("Throttle error: ");Serial.println(Throttle.getErrorStr(Error));
}
void notifyThrottleState( uint8_t UserData, TH_STATE PrevState, TH_STATE State )
{
  Serial.print("Throttle state: ");Serial.println(Throttle.getStateStr(State));
}