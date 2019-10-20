/****************************************************************************
    Copyright (C) 2013 Alex Shepherd

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

 Title :   LocoNet Block Occupancy Detector
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     7-Apr-2013
 Software: Arduio
 Target:   AtMega168

 DESCRIPTION
    This project is a LocoNet Block Occupancy Detector
	
*****************************************************************************/
#include "vcDebounce.h"
#include <LocoNet.h>
#include <avr/eeprom.h>

#define FL_REPORT_STATE_ON_POWER      0x01  // Report Sensor States on Power-Up
#define FL_REPORT_STATE_ON_OPC_GPON   0x02  // Report Sensor States on OPC_GPON
#define FL_ENABLE_LN_BOOTLOADER       0x80  // Enable LocoNet IPL Bootloader

#define MANUFACTURER_ID   13    // DIY Manufacturer Id
#define DEVELOPER_ID      4     // EmbeddedLocoNet
#define PRODUCT_ID        1     // LocoBOD-16
#define VERSION 1

// suppress bogus warning when compiling with gcc 4.3
#if (__GNUC__ == 4 && __GNUC_MINOR__ == 3)
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif 

typedef struct
{
  uint16_t  SVNodeId;               // SV 3 & 4
  uint16_t  SVSerialNumber;         // SV 5 & 6
  uint8_t   Flags;                  // SV 7 Config Flags Report the state of all sensors when this device is powered-up
  uint8_t   DebouncePeriod;         // SV 8 Debounce Period in 1..255 ms
  uint8_t   UnoccupiedReportDelay;  // SV 9 Delay (x 100ms) 0..25500 ms before sending a Block Unoccupied Message
  uint16_t  SensorAddresses[16];    // LocoNet Sensor Addresses for Sensors
} EE_SV_REC ;

EE_SV_REC eeSVRec __attribute__((section(".eeprom"))) = 
{ 
  0x0001, // SV 3 & 4 Node Id
  0x0001, // SV 5 & 6 Serial Number
  (FL_REPORT_STATE_ON_POWER | FL_REPORT_STATE_ON_OPC_GPON ),      // SV 7 Flags 
  10,     // SV 8 Debounce Period
  2,      // SV 9 Delay before sending a Block Unoccupied Message x 100ms
  {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16} // LocoNet Sensor Addresses
} ;

#define PORTB_USED_INPUTS 0x3E
#define PORTC_USED_INPUTS 0x3F
#define PORTD_USED_INPUTS_H 0xE0
#define PORTD_USED_INPUTS_L 0x0C

#define SV_ADDR_TO_EEPROM_ADDR_OFFSET (3 - offsetof( EE_SV_REC, SVSerialNumber ))
#define SV_ADDR_MAX    (sizeof( EE_SV_REC) - 2 + SV_ADDR_SW_VERSION )

#define NUM_SENSORS 16

typedef enum
{
  SS_UNOCCUPIED = 0,
  SS_OCCUPIED,
} SENSOR_STATE ;

typedef struct sensor_rec_t
{
  SENSOR_STATE  State ;
  uint8_t          DelayCount ;
  uint8_t          Queued ;
  uint8_t          Next ;
} SENSOR_REC ;

#define QUEUE_END 255

SENSOR_REC Sensors[NUM_SENSORS];
uint8_t ReportQueueHead ;
uint8_t PowerOnReportCount ;
//FIXME TimerAction ProcessSensorsTimer ;
//FIXME TimerAction ProcessUnoccupiedTimer ;

uint8_t PortBSamples, PortBLastStates, PortCSamples, PortCLastStates, PortDSamples, PortDLastStates ;
VC_DEBOUNCE_STATE PortBDebounceStates, PortCDebounceStates, PortDDebounceStates ;
uint16_t LastSensorStates ;

lnMsg *LnPacket ;
LN_STATUS Status ;
SV_STATUS SvStatus ;

LocoNetSystemVariableClass LocoNetSV;

void initReportQueue( void )
{
  uint8_t i ;
  
  for( i = 0; i < NUM_SENSORS; i++ )
    Sensors[i].Queued = 0 ;
    Sensors[i].Next = QUEUE_END ;
    
  ReportQueueHead = QUEUE_END ;
}

void addReportQueue( uint8_t addIndex )
{
#ifdef DEBUG_PRINT
  _printf("a%d\r", addIndex ) ;
#endif  
  uint8_t i ;
  
  if( ReportQueueHead == QUEUE_END )
    ReportQueueHead = addIndex ;

  else
  {
    for( i = ReportQueueHead; Sensors[ i ].Next != QUEUE_END; i = Sensors[ i ].Next )
      ;
      
    Sensors[ i ].Next = addIndex ;
  }

  Sensors[ addIndex ].Queued = 1 ;
  Sensors[ addIndex ].Next = QUEUE_END ;
}

void addReportQueueAll( void )
{
#ifdef DEBUG_PRINT
  _printf("aA\r" ) ;
#endif  
  uint8_t i ;
  
  for( i = 0; i < NUM_SENSORS; i++ )
    if( !Sensors[ i ].Queued )
      addReportQueue( i ) ;
}

void removeReportQueue( uint8_t removeIndex )
{
#ifdef DEBUG_PRINT
  _printf("r%d\r", removeIndex ) ;
#endif  
  uint8_t i ;
  
  if( ReportQueueHead == removeIndex )
    ReportQueueHead = Sensors[ removeIndex ].Next ;

  else
  {
    for( i = ReportQueueHead; Sensors[ i ].Next != removeIndex; i = Sensors[ i ].Next )
      ;
      
    Sensors[ i ].Next = Sensors[ removeIndex ].Next ;
  }

  Sensors[ removeIndex ].Queued = 0 ;
  Sensors[ removeIndex ].Next = QUEUE_END ;
}

void ProcessSensorState( uint8_t SensorIndex, uint8_t Value )
{
#ifdef DEBUG_PRINT
//  _printf("ProcessSensorState: %d %d\r", SensorIndex, Value ) ;
#endif  
  if( Value )
  {
      // If the sensors was previously Unoccupied then make it occupied
    if( Sensors[SensorIndex].State != SS_OCCUPIED )
    {
      Sensors[SensorIndex].State = SS_OCCUPIED ;

        // If the delay counter was counting down then just stop it
      if( Sensors[SensorIndex].DelayCount )
        Sensors[SensorIndex].DelayCount = 0 ;
        // If it's NOT queued then add it to the Report Queue
      else if( !Sensors[SensorIndex].Queued )
        addReportQueue(SensorIndex);
    }
  }
  else
  {
    if( Sensors[SensorIndex].State != SS_UNOCCUPIED )
    {
      Sensors[SensorIndex].State = SS_UNOCCUPIED ;
      
      Sensors[SensorIndex].DelayCount = eeprom_read_byte( &eeSVRec.UnoccupiedReportDelay ) ;
      if( ( Sensors[SensorIndex].DelayCount == 0 ) && !Sensors[SensorIndex].Queued )
        addReportQueue(SensorIndex);
    }
  }
}

uint8_t ProcessUnoccupiedTimers(void)
{
  uint8_t SensorIndex ;
  
  for( SensorIndex = 0; SensorIndex < NUM_SENSORS; SensorIndex++ )
    if( Sensors[SensorIndex].DelayCount )
    {
      Sensors[SensorIndex].DelayCount--;
      if( ( Sensors[SensorIndex].DelayCount == 0 ) && !Sensors[SensorIndex].Queued )
        addReportQueue(SensorIndex);
    }

  return 1 ;
}

LN_STATUS ReportSensor( uint8_t SensorIndex )
{
  LN_STATUS LnStatus ;
  
  uint16_t Address = eeprom_read_word( &eeSVRec.SensorAddresses[SensorIndex] ) ;

  LnStatus = LocoNet.reportSensor( Address, Sensors[ SensorIndex ].State == SS_OCCUPIED ) ;
#ifdef DEBUG_PRINT
//  _printf("RS%d-%d-%d\r", SensorIndex, Sensors[ SensorIndex ].State, LnStatus ) ;
#endif  
  return LnStatus ;
}

uint8_t ProcessSensors(void)
{
  uint8_t PortBStates, PortCStates, PortDStates, SensorIndex ;
  uint16_t SensorStates, SensorChanges, SensorMask ;
  
  PortBStates = vcDebounce( &PortBDebounceStates, PortBSamples ) ; 
  PortCStates = vcDebounce( &PortCDebounceStates, PortCSamples ) ; 
  PortDStates = vcDebounce( &PortDDebounceStates, PortDSamples ) ; 
  
#ifdef DEBUG_PRINT
//  _printf("ProcessSensors: PortStates: B: %02X-%02X  C: %02X-%02X  D: %02X-%02X\r", PortBSamples, PortBStates, PortCSamples, PortCStates, PortDSamples, PortDStates ) ;
#endif

    // Check to see if any of the ports have changed state. If necessary recompute all Sensor states 
  if( ( PortBLastStates != PortBStates ) || ( PortCLastStates != PortCStates ) || ( PortDLastStates != PortDStates ) )
  {
    SensorStates  = ( PortDStates & PORTD_USED_INPUTS_H ) >> 5 ;  // Sensors  0..2  from PD5..PD7
    SensorStates |= ( PortBStates & PORTB_USED_INPUTS ) << 2 ;    // Sensors  3..7  from PB1..PB5
    SensorStates |= ( PortCStates & PORTC_USED_INPUTS ) << 8 ;    // Sensors  8..13 from PC0..PC5
    SensorStates |= ( PortDStates & PORTD_USED_INPUTS_L ) << 12 ; // Sensors 14..15 from PD4..PD5
    
#ifdef DEBUG_PRINT
    _printf("PS%04X\r", SensorStates ) ;
#endif

      // Check for changed Sensor states
    SensorChanges = LastSensorStates ^ SensorStates ;
    if( SensorChanges )
    {
#ifdef DEBUG_PRINT
      _printf("PC%04X\r", SensorChanges ) ;
#endif
      for( SensorIndex = 0; SensorIndex < 16; SensorIndex++ )
      {
        SensorMask = 1 << SensorIndex ;
        if( SensorMask & SensorChanges )
          ProcessSensorState( SensorIndex, ( SensorMask & SensorStates ) != 0 ) ;
      }

      LastSensorStates = SensorStates ;
    }
  }

  PortBLastStates = PortBStates ;
  PortCLastStates = PortCStates ;
  PortDLastStates = PortDStates ;

  PortBSamples = 0 ;
  PortCSamples = 0 ;
  PortDSamples = 0 ;
  
  if( PowerOnReportCount )
  {
    if( --PowerOnReportCount == 0 )
      addReportQueueAll() ;
  }
  
  return eeprom_read_byte( &eeSVRec.DebouncePeriod) ; // return the timer period for subsequent cycles
}

void setup(void)
{
    // All ports default to being Inputs so we need to Enable Pull-Up Resistors
    // on the inputs we are using for BOD's  
  PORTB |= PORTB_USED_INPUTS ; // Also enable a pull-up on the ICP pin.
  PORTC |= PORTC_USED_INPUTS ;
  PORTD |= PORTD_USED_INPUTS_H | PORTD_USED_INPUTS_L ;

#ifdef DEBUG_PRINT
  DDRD |= 2 ;
  _printf("LocoBOD-16\r");
#endif

  LocoNet.init(6);
  LocoNetSV.init(MANUFACTURER_ID, DEVELOPER_ID, PRODUCT_ID, VERSION);

  initReportQueue() ;
  
  if( eeprom_read_byte( &eeSVRec.Flags ) & FL_REPORT_STATE_ON_POWER )
    PowerOnReportCount = 4 ;
}

void notifyPower( uint8_t State )
{
  if(State && (eeprom_read_byte( &eeSVRec.Flags ) & FL_REPORT_STATE_ON_OPC_GPON ) )
    addReportQueueAll();
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

unsigned long ticksProcessSensors;
unsigned long ticksProcessUnoccupiedTimers;

void loop(void)
{
  LnPacket = LocoNet.receive() ;
  if( LnPacket )
  {
    if( LocoNetSV.processMessage( LnPacket ) == SV_DEFERRED_PROCESSING_NEEDED)
      SvStatus = SV_DEFERRED_PROCESSING_NEEDED;

    if(SvStatus == SV_DEFERRED_PROCESSING_NEEDED)
      SvStatus = LocoNetSV.doDeferredProcessing();
      
      // Process the packet in case its a OPC_GPON
    LocoNet.processSwitchSensorMessage(LnPacket);
    
      // Sample the Inputs and OR the states with the previous samples 
    PortBSamples |= ~PINB ;
    PortCSamples |= ~PINC ;
    PortDSamples |= ~PIND ;
    
    if( isTime(&ticksProcessSensors, eeprom_read_byte( &eeSVRec.DebouncePeriod)))
      ProcessSensors();
      
    if( isTime(&ticksProcessUnoccupiedTimers, 100))
      ProcessUnoccupiedTimers();
    
      // If there is a sensor in the ReportQueue then try and report it's state to LocoNet
      // and then remove it from the Queue and loop around again
    if( ReportQueueHead != QUEUE_END )
      if( ReportSensor( ReportQueueHead ) == LN_DONE )
        removeReportQueue( ReportQueueHead ) ;
  }
}
