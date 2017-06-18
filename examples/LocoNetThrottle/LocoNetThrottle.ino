/****************************************************************************
    Copyright (C) 2002,2003,2004 Alex Shepherd

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

 Title :   LocoNet Serial Terminal Throttle
 Author:   Alex Shepherd <kiwi64ajs@gmail.com>
 Date:     9-Mar-2017
 Target:   ATMega328 Arduino

 DESCRIPTION
  This project is a LocoNet throttle which uses a Serial Terminal (VT100 Emulation) 
  to Display the status and uses the keyboard to interact and perform control actions.

  The BasicTerm library can be found here: https://github.com/nottwo/BasicTerm

*****************************************************************************/

#include <LocoNet.h>
#include <BasicTerm.h>
#include <ctype.h>
#include <EEPROM.h>

  // EEPROM storage locations for State
#define EE_LAST_TH_STATE 0  // EEPROM Offset of  8-Bit Throttle State 
#define EE_LAST_TH_SLOT  1  // EEPROM Offset of  8-Bit Throttle Slot Number
#define EE_LAST_TH_ADDR  2  // EEPROM Offset of 16-Bit Throttle Loco Address 
#define EE_LAST_TH_IDX   4  // EEPROM Offset of 16-Bit Throttle IDX (Should be Unique Throttle ID)

#define DEFAULT_THROTTLE_IDX 0x3FF0

#define RECALL_BUFFER_SIZE 8

typedef struct
{
  uint8_t         lastSlot;
  uint16_t        lastLocoAddr ;
  uint16_t        lastThrottleIDX ;
  TH_STATE        lastState;
  TH_SPEED_STEPS  lastSpeedSteps;
  int16_t         recallBuffer[RECALL_BUFFER_SIZE];
} STORED_STATE;
STORED_STATE ss;

uint16_t          LocoAddr ;
uint8_t           recallIndex;

LocoNetThrottleClass  Throttle ;
lnMsg                 *RxPacket ;
uint32_t              LastThrottleTimerTick;

BasicTerm Term(&Serial);

void DrawStaticText(void)
{
  Term.show_cursor(0);
  Term.cls();
  Term.position(0,0);
  Term.println(F("LocoNet Throttle Library Demonstration Version: 3"));
  Term.println();
  Term.print(F("Address    : ")); Term.println(LocoAddr);
  Term.println();
  Term.println(F("Speed      :"));
  Term.println();
  Term.println(F("Direction  :"));
  Term.println();
  Term.println(F("Functions  : 0 1 2 3 4 5 6 7 8"));
//                        1         2         3             
//              0123456789012345678901234567890  
  Term.println();
  Term.println(F("Status     :"));
  Term.println();
  Term.println(F("Error      :"));
  Term.println();
  Term.println(F("Last Key   :"));
  Term.println();
  Term.println(F("Resumed    :"));
  Term.println();
  Term.print(F("Speed Steps: "));
  Term.print(Throttle.getSpeedStepStr(ss.lastSpeedSteps));
  Term.println("     ");
  Term.println();
  Term.print(F("Recall Buffer:"));
  printRecallBuffer();
  Term.println();
  Term.println();
  Term.println(F("Keys: A           - Acquire previously Dispatched Loco Address"));
  Term.println(F("Keys: D           - Dispatch Loco Address"));
  Term.println(F("Keys: <Enter>     - Select Loco Address"));
  Term.println(F("Keys: S           - Steal Loco Address"));
  Term.println(F("Keys: I           - Idle Loco,    (Keep Address,  Slot Not Active)"));
  Term.println(F("Keys: R           - Release Loco, (Keep Address,  Slot stays Active)"));
  Term.println(F("Keys: F           - Free Loco,    (Clear Address, Slot not Active)"));
  Term.println(F("Keys: Z           - Force Loco Address to be Freed"));
  Term.println(F("Keys: Arrow LEFT  - Reduce Speed"));
  Term.println(F("Keys: Arrow RIGHT - Increase Speed"));
  Term.println(F("Keys: Arrow UP    - Forward"));
  Term.println(F("Keys: Arrow Down  - Reverse"));
  Term.println(F("Keys: T           - Toggle Direction"));
  Term.println(F("Keys: <SP>        - Stop"));
  Term.println(F("Keys: [ ]         - Release current Loco and select Address from Recall Buffer"));
  Term.println(F("Keys: P           - Select Next Speed Step Mode"));
  Term.println(F("Keys: 0..8        - When IN_USE - Toggle Functions 0..8"));
  Term.println(F("Keys: 0..9, <BS>  - When not IN_USE - Edit Address"));
}

void printRecallBuffer(void)
{
  Term.position(20, 15);
  for(uint8_t i = 0; i < RECALL_BUFFER_SIZE; i++)
  {
    if(ss.recallBuffer[i] != -1)
    {
      if( ss.recallBuffer[i] == LocoAddr)
        Term.set_attribute(BT_BOLD);
        
      Term.print(ss.recallBuffer[i]);
      Term.print(' ');

      if( ss.recallBuffer[i] == LocoAddr)
        Term.set_attribute(BT_NORMAL);
    }
  }
}

void insertRecallBuffer(int16_t Address)
{
  memmove(&ss.recallBuffer[1], &ss.recallBuffer[0], (RECALL_BUFFER_SIZE - 1) * 2);
  ss.recallBuffer[0] = Address;
  EEPROM.put(0,ss) ;
  recallIndex = 0;
}

uint8_t posRecallBuffer(int16_t Address)
{
  uint8_t i;
  for(i = 0; i < RECALL_BUFFER_SIZE; i++)
    if(ss.recallBuffer[i] == Address)
      break;
      
  return i;    
}

void printAddress(uint16_t Address)
{
  Term.position(2,13);
  Term.print(Address);
  Term.print("     "); // Erase any extra chars
}

void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t Slot )
{
  if(State == TH_ST_IN_USE)
  {
    ss.lastState = State;
    ss.lastLocoAddr = Address;
    ss.lastSlot = Slot;

      // Check if the Address is in the Recall Buffer, if not insert it
    if(posRecallBuffer(Address) == RECALL_BUFFER_SIZE)
    {
      insertRecallBuffer(Address);
      printRecallBuffer();
    }
    EEPROM.put(0,ss);
  }
  printAddress(Address);
};

void notifyThrottleSpeed( uint8_t UserData, TH_STATE State, uint8_t Speed )
{
  Term.position(4,13);
  Term.print(Speed);
  Term.print("     "); // Erase any extra chars
};

void notifyThrottleDirection( uint8_t UserData, TH_STATE State, uint8_t Direction )
{
  Term.position(6,13);
  Term.print(Direction ? "Reverse" : "Forward");
};

void notifyThrottleFunction( uint8_t UserData, uint8_t Function, uint8_t Value )
{
  Term.position(8,13 + (Function * 2) );
  if(Value)
    Term.set_attribute(BT_BOLD);
    
  Term.print(Function, DEC);

  if(Value)
    Term.set_attribute(BT_NORMAL);
};

void notifyThrottleSlotStatus( uint8_t UserData, uint8_t Status ){};

void notifyThrottleState( uint8_t UserData, TH_STATE PrevState, TH_STATE State )
{
  if(State <= TH_ST_RELEASE) // We really only want to store non active or final states to EEPROM
  {
    ss.lastState = State;  
    EEPROM.put(0,ss);
  }
  
  Term.position(10, 13);
  Term.print(State, DEC);
  Term.print(' ');
  Term.print(Throttle.getStateStr(State));
  Term.print("                   ");
}

void notifyThrottleError( uint8_t UserData, TH_ERROR Error )
{
  Term.position(12, 13);
  Term.print(Error, DEC);
  Term.print(' ');
  Term.print(Throttle.getErrorStr(Error));
  Term.print("                   ");
}

void notifyThrottleSpeedSteps( uint8_t UserData, TH_SPEED_STEPS SpeedSteps )
{
  Term.position(18, 13);
  Term.print(Throttle.getSpeedStepStr(SpeedSteps));Term.print("     ");
}

void setup()
{
  // First initialize the LocoNet interface
  LocoNet.init(7);

  // Configure the serial port for 57600 baud
  Serial.begin(115200);
  Term.init();

// Uncomment to force Reset of EEPROM Values
//  EEPROM.write(0,0xFF);
  
  EEPROM.get(0, ss);
  if(ss.lastSlot == 0xFF) // EEPROM Default Erased State or a Valid slot? 
  {
    ss.lastLocoAddr = 0;
    ss.lastThrottleIDX = DEFAULT_THROTTLE_IDX;
    ss.lastState = TH_ST_FREE;
    ss.lastSpeedSteps = TH_SP_ST_128_ADV;
    for(uint8_t i = 0; i < RECALL_BUFFER_SIZE; i++) 
      ss.recallBuffer[i] = -1;
    EEPROM.put(0,ss) ;
  }

  LocoAddr = ss.lastLocoAddr;

  DrawStaticText();
  
  Throttle.init(0, 0, ss.lastThrottleIDX);
  Throttle.setSpeedSteps(ss.lastSpeedSteps);

  Term.position(16, 13);
  Term.print( (ss.lastState == TH_ST_IN_USE) ? F("Yes") : F("No "));
  Term.print(F("  Last Slot: ")); Term.print(ss.lastSlot);
  Term.print(F("  Last State: ")); Term.print(ss.lastState);
  Term.print(F("  Last Address: ")); Term.print(ss.lastLocoAddr);
  Term.print(F("  Last IDX: ")); Term.print(ss.lastThrottleIDX,HEX);
  Term.print(F("  Last Speed Step Mode: ")); Term.print(Throttle.getSpeedStepStr(ss.lastSpeedSteps));
  Term.print("     ");

  if(ss.lastState == TH_ST_IN_USE)
  {
    LocoAddr = ss.lastLocoAddr;
    Throttle.resumeAddress(ss.lastLocoAddr, ss.lastSlot);
  }
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

void loop()
{  
  // Check for any received LocoNet packets
  RxPacket = LocoNet.receive() ;
  if( RxPacket )
  {
    digitalWrite(13, LOW);
    
    if( !LocoNet.processSwitchSensorMessage(RxPacket) )
      Throttle.processMessage(RxPacket) ; 
  }
  
  if( Term.available())
  {
    int16_t inChar = Term.get_key();
    if(inChar < 127)
      inChar = toupper(inChar);
      
    Term.position(14,13);
    Term.print(inChar);
    switch(inChar){
      case 13 : if(Throttle.getState() != TH_ST_IN_USE)
                  Throttle.setAddress(LocoAddr);
                break;
                
      case 'A': if(Throttle.getState() != TH_ST_IN_USE)
                  Throttle.acquireAddress();
                break;
                
      case 'D': if(Throttle.getState() == TH_ST_IN_USE)
                  Throttle.dispatchAddress();
                else
                  Throttle.dispatchAddress(LocoAddr);
                  DrawStaticText();
                break;
                
      case 'S': if(Throttle.getState() != TH_ST_IN_USE)
                  Throttle.stealAddress(LocoAddr);
                break;
                
      case 'F': if(Throttle.getState() == TH_ST_IN_USE)
                {
                  Throttle.freeAddress();
                  DrawStaticText();
                }
                break;
                
      case 'I': if(Throttle.getState() == TH_ST_IN_USE)
                {
                  Throttle.idleAddress();
                  DrawStaticText();
                }
                break;
                
      case 'R': if(Throttle.getState() == TH_ST_IN_USE)
                {
                  Throttle.releaseAddress(); 
                  DrawStaticText();
                }
                break;

      case 'Z': if(Throttle.getState() <= TH_ST_RELEASE)
                {
                  Throttle.freeAddressForce(LocoAddr);
                  DrawStaticText();
                }
                break;
                
      case BT_KEY_UP :
                if(Throttle.getState() == TH_ST_IN_USE)
                  Throttle.setDirection(0); 
                break;
                
      case BT_KEY_DOWN :
                if(Throttle.getState() == TH_ST_IN_USE)
                  Throttle.setDirection(1); 
                break;
                
      case 'P': switch(ss.lastSpeedSteps)
                {
                case TH_SP_ST_14:      // 010=14 step MODE
                  ss.lastSpeedSteps = TH_SP_ST_28;
                  break;

                case TH_SP_ST_28:    // 000=28 step/ 3 BYTE PKT regular mode
                  ss.lastSpeedSteps = TH_SP_ST_28_TRI;
                  break;
     
                case TH_SP_ST_28_TRI:  // 001=28 step. Generate Trinary packets for this Mobile ADR
                  ss.lastSpeedSteps = TH_SP_ST_28_ADV;
                  break;

                case TH_SP_ST_28_ADV:  // 100=28 Step decoder ,Allow Advanced DCC consisting
                  ss.lastSpeedSteps = TH_SP_ST_128;
                  break;
  
                case TH_SP_ST_128:     // 011=send 128 speed mode packets
                  ss.lastSpeedSteps = TH_SP_ST_128_ADV;
                  break;
    
                case TH_SP_ST_128_ADV: // 111=128 Step decoder, Allow Advanced DCC consisting
                default:
                  ss.lastSpeedSteps = TH_SP_ST_14;
                  break;
                }
                
                Throttle.setSpeedSteps(ss.lastSpeedSteps);
                EEPROM.put(0,ss);

                break;
                  
      case 'T': if(Throttle.getState() == TH_ST_IN_USE)
                  Throttle.setDirection(!Throttle.getDirection());
                break;

      case BT_KEY_LEFT :
                if((Throttle.getState() == TH_ST_IN_USE) && (Throttle.getSpeed() > 0 ))
                  Throttle.setSpeed(Throttle.getSpeed() - 1);
                break;
                
      case BT_KEY_RIGHT :
                if((Throttle.getState() == TH_ST_IN_USE) && (Throttle.getSpeed() < 127 ))
                  Throttle.setSpeed(Throttle.getSpeed() + 1);
                break;
                
      case ' ': if(Throttle.getState() == TH_ST_IN_USE)
                  Throttle.setSpeed(0);
                break;

      case '[': if(recallIndex > 0)
                {
                  recallIndex--;
                
                  if(Throttle.getState() == TH_ST_IN_USE)
                    Throttle.releaseAddress();

                  LocoAddr = ss.recallBuffer[recallIndex];

                  DrawStaticText();
                }
                break;

      case ']': if(recallIndex < (RECALL_BUFFER_SIZE - 1))
                {
                  recallIndex++;
                
                  if(Throttle.getState() == TH_ST_IN_USE)
                    Throttle.releaseAddress();

                  LocoAddr = ss.recallBuffer[recallIndex];

                  DrawStaticText();
                }
                break;

      default:  if(Throttle.getState() <= TH_ST_RELEASE) 
                {
                  if( (inChar >= '0') && (inChar <= '9') && (LocoAddr < 999) )
                  {
                    LocoAddr *= 10;
                    LocoAddr += inChar - '0';
                  }
                  else if(inChar == 8)
                    LocoAddr /= 10;
                    
                  printAddress(LocoAddr);
                }
                else if( (inChar >= '0') && (inChar <= '8'))
                  Throttle.setFunction( inChar - '0', !Throttle.getFunction(inChar - '0'));
                break;
    }
  }
  
  if(isTime(&LastThrottleTimerTick, 100))
  {
    Throttle.process100msActions() ; 
    digitalWrite(13, HIGH);
  }
}
