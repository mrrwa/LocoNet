
/*
   LocoNet Arduino Layout Controler using Serial Terminal 
   LocoNetLayoutControlPanel_SerialTerminal.ino

   A standalone controller which will generate LocoNet commands from a computer when key 'o', 'f', 'c' or 'e' is pressed
     Turn on track power with 'o' key
     Turn off track power with 'f' key
     Clear all decoder slots with 'c' key
     Stop all engines with 's' key

   RX and TX Loconet
     Hardcoded to use ICP pin 8 (port PINB bit PB0) for LocoNet input and a user define'd pin for output/transmit

   Copyright (c) 2012, 2014 John Plocher, released under the terms of the MIT License (MIT)

   Modified by Alex Shephard at Al Silverstein request

   Change from eStop command to Loco Speed command by ALS on 11/9/21

*/

#include <LocoNet.h>


#define LNtxPort         7    // LocoNet Transmit pin (LocoShield uses pin7)

int MaxSlot = 120;

typedef enum
{
  CMD_DONE,
  CMD_SPEED_ZERO,
  CMD_SLOT_CLEAR
} CMD_STATE;

CMD_STATE CommandState = CMD_DONE;

String ValidCommands = "ofcis";

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
  send3bytePacket(OPC_LOCO_SPD, slot, spd);
}

void sendOPC_LOCO_DIRF(int slot, int dirf) {
  send3bytePacket(OPC_LOCO_DIRF, slot, dirf);
}

void sendOPC_LOCO_SND(int slot, int snd) {
  send3bytePacket(OPC_LOCO_SND, slot, snd);
}

void sendOPC_SLOT_STAT1(int slot, int stat) {
  send3bytePacket(OPC_SLOT_STAT1, slot, stat);
}

void sendOPC_RQ_SL_DATA(int slot) {
  send3bytePacket(OPC_RQ_SL_DATA, slot, 0);
}

void processIncomingLoconetCommand(lnMsg* LnPacket) {
  if ( LnPacket )
  {
    switch(LnPacket->sz.command)
    {
      case OPC_GPON:
        Serial.println("Power ON");
        break;

      case OPC_GPOFF:
        Serial.println("Power OFF");
        break;

      case OPC_IDLE:
        Serial.println("EStop!");
        break;

      case OPC_SL_RD_DATA:
        if (CommandState == CMD_SLOT_CLEAR)
        {
          if(LnPacket->sd.stat & LOCO_IN_USE)
          {
            Serial.print("Clear Slot: "); Serial.print(LnPacket->sd.slot); Serial.print("  Status: ");  Serial.println(LnPacket->sd.stat, HEX);
            LnPacket->sd.command = OPC_WR_SL_DATA;
            LnPacket->sd.stat = 0;
            LnPacket->sd.adr = 0;
            LnPacket->sd.spd = 0;
            LnPacket->sd.dirf = 0;
            LnPacket->sd.ss2 = 0;
            LnPacket->sd.adr2 = 0;
            LnPacket->sd.snd = 0;
            LnPacket->sd.id1 = 0;
            LnPacket->sd.id2 = 0;
            LocoNet.send(LnPacket);
          }
        }
        
        else if(CommandState == CMD_SPEED_ZERO)
        {
          if((LnPacket->sd.stat & LOCO_IN_USE) && (LnPacket->sd.spd))
            sendOPC_LOCO_SPD(LnPacket->sd.slot, 0);
        }
        
        if(LnPacket->sd.slot >= MaxSlot)
          CommandState = CMD_DONE;
        break;

      case OPC_LOCO_SPD:
        Serial.print("Set Slot: ");   Serial.print(LnPacket->lsp.slot);
        Serial.print("  : Speed: ");  Serial.println(LnPacket->lsp.spd);
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nLocoNet Controller");

  // initialize the LocoNet interface
  LocoNet.init(LNtxPort);
}

char inChar = 0;
lnMsg sendPacket;

void loop() {
  // Check for any received LocoNet packets
  while (lnMsg *LnPacket = LocoNet.receive() )
  {
    processIncomingLoconetCommand( LnPacket );
  }

  if (Serial.available())
  {
    inChar = Serial.read();

    if (ValidCommands.indexOf(inChar) >= 0)
    {
      Serial.print("Command: "); Serial.println(inChar);

      if(inChar == 'o') // Track Power On
      {
        sendOPC_GP(1);
      }

      else if(inChar == 'f')  // Track Power Off
      {
        sendOPC_GP(0);
      }

      else if(inChar == 'i')  // Idle
      {
        sendOPC_IDLE();
      }

      else
      {
        if(inChar == 's')
          CommandState = CMD_SPEED_ZERO;

        else if(inChar == 'c')
          CommandState = CMD_SLOT_CLEAR;

        for (int slot = 0; slot <= MaxSlot; slot++)
        {
          sendOPC_RQ_SL_DATA(slot);

          while (lnMsg *LnPacket = LocoNet.receive() )
            processIncomingLoconetCommand( LnPacket );
        }
      }
    }
  }
}
