/*
 * LocoNet Layout Commander
 *
 * A standalone controller which will generate LocoNet commands when a button is pressed to:
 *   Turn track power on and off, with a LED to indicate state.
 *   Generate a master E-Stop, with a LED to indicate state.
 *   Clear all slots in an attached command station
 *
 * RX and TX Loconet
 *   Hardcoded to use ICP pin 8 (Pin PINB bit PB0) for LocoNet input and a user define'd pin for output/transmit
 * 
 * 3 pushbuttons and associated LEDs plus a Loconet Shield
 *
 * Copyright (c) 2012, 2014 John Plocher, released under the terms of the MIT License (MIT)
 */

#include <LocoNet.h>

#define PowerOnPin      2    // Buttons connected to these pins
#define PowerOffPin     3    // GPON GPOFF and ESTOP as well as Clear All Slots
#define EStopPin        4
#define SlotClearPin    5

#define LNtxPin         6    // LocoNet Transmit pin (LocoShield uses pin7)

#define PowerOnLEDPin  11    // Assume this pin has a LED+resistor attached...
#define PowerOffLEDPin 12    // Assume this pin has a LED+resistor attached...
#define EStopLEDPin    13    // Assume this pin has a LED+resistor attached...

// Button press state
// Current
int GPonButton,    GPonButton1,   GPonButton2;
int GPoffButton,   GPoffButton1,  GPoffButton2;
int EStopButton,   EStopButton1,  EStopButton2;
int ClearButton,   ClearButton1,  ClearButton2;

// Last state processed - helps us ensure we don't repeat commands while a button is held down
int lastGPon  = -1;
int lastGPoff = -1;
int lastEStop = -1;
int lastClear = -1;

int ClearIt   =  0;           // Should we clear slots when we get a slot status packet?
    
void setup(){
    pinMode(PowerOnPin,     INPUT);   
    pinMode(PowerOffPin,    INPUT);   
    pinMode(SlotClearPin,   INPUT);   
    pinMode(EStopPin,       INPUT); 
    
    pinMode(EStopLEDPin,    OUTPUT); 
    pinMode(PowerOnLEDPin,  OUTPUT); 
    pinMode(PowerOffLEDPin, OUTPUT); 
    
    digitalWrite(PowerOnLEDPin,  0);   // Power is in an unknown state
    digitalWrite(PowerOffLEDPin, 0);   // 
    digitalWrite(EStopLEDPin,    0);   // and not estopped
 
    // Configure the serial Pin for 57600 baud
    Serial.begin(57600);
    Serial.println("LocoNet Controller");     
    
    // initialize the LocoNet interface
    LocoNet.init(LNtxPin);
}                   

void setLNTurnout(int address, byte dir) {
    sendOPC_SW_REQ( address - 1, dir, 1 );
    sendOPC_SW_REQ( address - 1, dir, 0 );
}

void sendOPC_SW_REQ(int address, byte dir, byte on) {
    lnMsg SendPacket ;
    
    int sw2 = 0x00;
    if (dir) sw2 |= B00100000;
    if (on)  sw2 |= B00010000;
    sw2 |= (address >> 7) & 0x0F;
    
    SendPacket.data[ 0 ] = OPC_SW_REQ ;
    SendPacket.data[ 1 ] = address & 0x7F ;
    SendPacket.data[ 2 ] = sw2 ;
    
    LocoNet.send( &SendPacket );
}

void sendOPC_INPUT_REP(int address, byte on) {
        lnMsg SendPacket;
        
        SendPacket.data[ 0 ] = OPC_INPUT_REP;  
        SendPacket.data[ 1 ] = address & 0x7F;  // turnout address
        int in2 = B01000000;
        if (on)  in2 |= B00010000;
        in2 |= (address >> 7) & 0x0F;
        SendPacket.data[ 2 ] = in2;            // sw2 contains direction, on/off and hi nibble of address

        LocoNet.send( &SendPacket ) ;
}

void sendOPC_GP(byte on) {
        lnMsg SendPacket;
        if (on) {
            SendPacket.data[ 0 ] = OPC_GPON;  
        } else {
            SendPacket.data[ 0 ] = OPC_GPOFF;  
        }
        LocoNet.send( &SendPacket ) ;
}

void sendOPC_IDLE() {
        lnMsg SendPacket;
        SendPacket.data[ 0 ] = OPC_IDLE;  
        LocoNet.send( &SendPacket ) ;
}

void send3bytePacket(int opcode, int slot, int spd) {
        lnMsg SendPacket;

        SendPacket.data[ 0 ] = opcode;
        SendPacket.data[ 1 ] = slot;
        SendPacket.data[ 2 ] = spd;  
        LocoNet.send( &SendPacket );
}
void sendOPC_LOCO_SPD(int slot, int spd) {
        send3bytePacket(OPC_LOCO_SPD,slot,spd);
}

void sendOPC_LOCO_DIRF(int slot, int dirf) {
        send3bytePacket(OPC_LOCO_DIRF,slot,dirf);
}

void sendOPC_LOCO_SND(int slot, int snd) {
        send3bytePacket(OPC_LOCO_SND,slot,snd);
}

void sendOPC_SLOT_STAT1(int slot, int stat) {
        send3bytePacket(OPC_SLOT_STAT1,slot,stat);
}

void sendOPC_RQ_SL_DATA(int slot) {
        send3bytePacket(OPC_RQ_SL_DATA,slot,0);
}

void processIncomingLoconetCommand(lnMsg* LnPacket) {
    if( LnPacket )      {   
        //LocoNet.processSwitchSensorMessage(LnPacket);
        unsigned char opcode = (int)LnPacket->sz.command;
        if (opcode == OPC_GPON)  {     
            Serial.println("Power ON");     
            digitalWrite(PowerOnLEDPin, 1);  
            digitalWrite(PowerOffLEDPin, 0);
            digitalWrite(EStopLEDPin, 0);
        } else if (opcode == OPC_GPOFF) {
            Serial.println("Power OFF");     
            digitalWrite(PowerOnLEDPin, 0);
            digitalWrite(PowerOffLEDPin, 1);
        } else if (opcode == OPC_IDLE) {
            digitalWrite(EStopLEDPin, 1);
            Serial.println("EStop!"); 
        } else if (opcode == OPC_SL_RD_DATA) {
            if (ClearIt) {
                int slot = LnPacket->sd.slot;
                int stat = LnPacket->sd.stat;
                Serial.print("Clear Slot:"); Serial.print(slot); Serial.print(":");  Serial.println(stat);
                if (stat != 0) {
                   sendOPC_LOCO_SPD(slot,  0);   // speed 0
                   sendOPC_LOCO_DIRF(slot, 0);   // F0-4 off, Fwd
                   sendOPC_LOCO_SND(slot,  0);   // F5-8 off
                    // Don't need to turn off F9 and above because they should go away when track power is turned off...
                   sendOPC_SLOT_STAT1(slot, 0);
                }
            }
        } else {
            // ignore the message...
        }
    }
}

void loop() {  
    // Check for any received LocoNet packets
    while (lnMsg *LnPacket = LocoNet.receive() ) {
        processIncomingLoconetCommand( LnPacket );
    }
    // Debounce logic:
    // ...Check for any buttons pushed, delay, read again...
    GPonButton1  = digitalRead(PowerOnPin);
    GPoffButton1 = digitalRead(PowerOffPin);
    EStopButton1 = digitalRead(EStopPin);
    ClearButton1 = digitalRead(SlotClearPin);
    delay(5);
    GPonButton2  = digitalRead(PowerOnPin);
    GPoffButton2 = digitalRead(PowerOffPin);
    EStopButton2 = digitalRead(EStopPin);
    ClearButton2 = digitalRead(SlotClearPin);

    // ...identical readings mean we have a good result
    if (GPonButton1  == GPonButton2)  { GPonButton  = GPonButton1  ? 0 : 1; }
    if (GPoffButton1 == GPoffButton2) { GPoffButton = GPoffButton1 ? 0 : 1; }
    if (EStopButton1 == EStopButton2) { EStopButton = EStopButton1 ? 0 : 1; }
    if (ClearButton1 == ClearButton2) { ClearButton = ClearButton1 ? 0 : 1; }

                
    if (lastGPon == -1) {
        // need to initialize things the first time thru to ensure buttons don't all fire...
        lastGPon = GPonButton;
        lastGPoff = GPoffButton;
        lastEStop = EStopButton;
        lastClear = ClearButton;
    } else {
        // See if anything has changed since last time thru...

        if (GPonButton != lastGPon) {                // GP_ON
            lastGPon = GPonButton;
            if (GPonButton) {
                sendOPC_GP(1);
            }
            ClearIt = 0;
                
        }
        if (GPoffButton != lastGPoff) {              // GP_OFF
            lastGPoff = GPoffButton;
            if (GPoffButton) {
                sendOPC_GP(0);
            }
            ClearIt = 0;
        }
        if (EStopButton != lastEStop) {              // E_STOP
            lastEStop = EStopButton;
            if (EStopButton) {
                sendOPC_IDLE();
            }
            ClearIt = 0;
        }
        if (ClearButton != lastClear) {              // Clear all Slots
            lastClear = ClearButton;
            if (ClearButton) {
                ClearIt = 1;
                // query all the slots, let the handler clear things
                for (int slot = 0; slot < 120; slot++) {
                    sendOPC_RQ_SL_DATA(slot);
                }
                ClearIt = 0;  // race condition?
            }
        }     
    }
}
