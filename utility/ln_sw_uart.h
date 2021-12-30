/****************************************************************************
	Copyright (C) 2002 Alex Shepherd

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

 Title :   LocoNet Software UART Access library (ln_sw_uart.h)
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     13-Aug-2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device

 DESCRIPTION
  Basic routines for interfacing to the LocoNet via any output pin and
  either the Analog Comparator pins or the Input Capture pin

  The receiver uses the Timer1 Input Capture Register and Interrupt to detect
  the Start Bit and then the Compare A Register for timing the subsequest
  bit times.

  The Transmitter uses just the Compare A Register for timing all bit times

 USAGE
  See the C include ln_interface.h file for a description of each function

*****************************************************************************/

#ifndef _LN_SW_UART_INCLUDED
#define _LN_SW_UART_INCLUDED

#ifdef ESP8266
#  include <Arduino.h>
#elif !defined(STM32F1)
#  include <avr/io.h>
#  include <avr/interrupt.h>
#endif

#include <string.h>

//added code to distinguish between inverted and non-inverted output 2020-03-28 Hans Tanner
#ifdef LN_SW_UART_TX_INVERTED 									//normally output is driven via NPN, so it is inverted
#  ifndef LN_SW_UART_SET_TX_LOW                               // putting a 1 to the pin to switch on NPN transistor
#    ifdef ESP8266
#      define LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT) lnLastTxBit = HIGH; digitalWrite(LN_TX_PORT, HIGH)    // to pull down LN line to drive low level
#    else
#      define LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT) LN_TX_PORT |= (1 << LN_TX_BIT)    // to pull down LN line to drive low level
#    endif
#  endif

#  ifndef LN_SW_UART_SET_TX_HIGH                              // putting a 0 to the pin to switch off NPN transistor
#    ifdef ESP8266
#      define LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT) lnLastTxBit = LOW; digitalWrite(LN_TX_PORT, LOW)    // master pull up will take care of high LN level
#    else
#      define LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT) LN_TX_PORT &= ~(1 << LN_TX_BIT)  // master pull up will take care of high LN level
#    endif
#  endif
#else //non-inverted output logic
#  ifndef LN_SW_UART_SET_TX_LOW    
#    ifdef ESP8266
#      define LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT) lnLastTxBit = LOW; digitalWrite(LN_TX_PORT, LOW)     // to pull down LN line to drive low level
#    else                           
#      define LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT) LN_TX_PORT &= ~(1 << LN_TX_BIT)   // to pull down LN line to drive low level
#    endif
#  endif

#  ifndef LN_SW_UART_SET_TX_HIGH 
#    ifdef ESP8266
#      define LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT) lnLastTxBit = HIGH; digitalWrite(LN_TX_PORT, HIGH)   // master pull up will take care of high LN level
#    else                             
#      define LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT) LN_TX_PORT |= (1 << LN_TX_BIT)   // master pull up will take care of high LN level
#    endif
#  endif
#endif

#if defined(STM32F1)
//Clear StartBit Interrupt flag
#  define LN_CLEAR_START_BIT_FLAG() (exti_reset_request(EXTI14))
//Enable StartBit Interrupt
#  define LN_ENABLE_START_BIT_INTERRUPT() (exti_enable_request(EXTI14))
//Disable StartBit Interrupt
#  define LN_DISABLE_START_BIT_INTERRUPT() (exti_disable_request(EXTI14))

// Clear Timer Interrupt Flag
#  define LN_CLEAR_TIMER_FLAG() (timer_clear_flag(TIM2, TIM_SR_CC1IF))

// Enable Timer Compare Interrupt
#  define LN_ENABLE_TIMER_INTERRUPT() (timer_enable_irq(TIM2, TIM_DIER_CC1IE))
// Disable Timer Compare Interrupt
#  define LN_DISABLE_TIMER_INTERRUPT() (timer_disable_irq(TIM2, TIM_DIER_CC1IE))
#elif !defined(ESP8266)
//Clear StartBit Interrupt flag
#  define LN_CLEAR_START_BIT_FLAG() (sbi( LN_SB_INT_STATUS_REG, LN_SB_INT_STATUS_BIT ))
//Enable StartBit Interrupt
#  define LN_ENABLE_START_BIT_INTERRUPT() (sbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ))
//Disable StartBit Interrupt
#  define LN_DISABLE_START_BIT_INTERRUPT() (cbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ))

// Clear Timer Interrupt Flag
#  define LN_CLEAR_TIMER_FLAG() (sbi(LN_TMR_INT_STATUS_REG, LN_TMR_INT_STATUS_BIT))

// Enable Timer Compare Interrupt
#  define LN_ENABLE_TIMER_INTERRUPT() (sbi(LN_TMR_INT_ENABLE_REG, LN_TMR_INT_ENABLE_BIT))
// Disable Timer Compare Interrupt
#  define LN_DISABLE_TIMER_INTERRUPT() (cbi( LN_TMR_INT_ENABLE_REG, LN_TMR_INT_ENABLE_BIT ))
#endif

// For now we will simply check that TX and RX ARE NOT THE SAME as our circuit
// requires the TX signal to be INVERTED.  If they are THE SAME then we have a 
// Collision.  
// Define LN_SW_UART_TX_INVERTED in your board header if your circuit doesn't
// (for example) use a NPN between TX pin and the Loconet port...

#ifdef ESP8266
#  define IS_LN_COLLISION() isLocoNetCollision()
#else
#  ifdef LN_SW_UART_TX_INVERTED
#    ifdef LN_SW_UART_RX_INVERTED
#      define IS_LN_COLLISION()	(((LN_TX_PORT >> LN_TX_BIT) & 0x01) != ((LN_RX_PORT >> LN_RX_BIT) & 0x01))
#    else
#      define IS_LN_COLLISION()	(((LN_TX_PORT >> LN_TX_BIT) & 0x01) == ((LN_RX_PORT >> LN_RX_BIT) & 0x01))
#    endif
#  else
#    ifdef LN_SW_UART_RX_INVERTED
#      define IS_LN_COLLISION()	(((LN_TX_PORT >> LN_TX_BIT) & 0x01) == ((LN_RX_PORT >> LN_RX_BIT) & 0x01))
#    else
#      define IS_LN_COLLISION()	(((LN_TX_PORT >> LN_TX_BIT) & 0x01) != ((LN_RX_PORT >> LN_RX_BIT) & 0x01))
#    endif
#  endif
#endif

#define LN_ST_IDLE            0   // net is free for anyone to start transmission
#define LN_ST_CD_BACKOFF      1   // timer interrupt is counting backoff bits
#define LN_ST_TX_COLLISION    2   // just sending break after creating a collision
#define LN_ST_TX              3   // transmitting a packet
#define LN_ST_RX              4   // receiving bytes

#define LN_COLLISION_TICKS 15
#define LN_TX_RETRIES_MAX  25

// The Start Bit period is a full bit period + half of the next bit period
// so that the bit is sampled in middle of the bit
//#define LN_TIMER_RX_START_PERIOD    LN_BIT_PERIOD + (LN_BIT_PERIOD / 2)
#define LN_TIMER_RX_START_PERIOD    LN_BIT_PERIOD / 2 // Read Startbit to make sure we are not started by a glitch
#define LN_TIMER_RX_RELOAD_PERIOD   LN_BIT_PERIOD 
#define LN_TIMER_TX_RELOAD_PERIOD   LN_BIT_PERIOD 

// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// LN_TIMER_TX_RELOAD_ADJUST is a value for an error correction. This is needed for 
// every start of a byte. The first bit is to long. Therefore we need to reduce the 
// reload value of the bittimer.
// The following value depences highly on used compiler, optimizationlevel and hardware.
// Define the value in sysdef.h. This is very project specific.
// For the FREDI hard- and software it is nearly a quarter of a LN_BIT_PERIOD.
// Olaf Funke, 19th October 2007
// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#ifndef LN_TIMER_TX_RELOAD_ADJUST
#  define LN_TIMER_TX_RELOAD_ADJUST   0
// #error detect value by oszilloscope
#endif

// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void initLocoNetHardware(LnBuf* RxBuffer);
void setTxPortAndPin(LnPortAddrType newTxPort, uint8_t newTxPin);
LN_STATUS sendLocoNetPacketTry(lnMsg* TxData, unsigned char ucPrioDelay);
#ifdef ESP8266
void ICACHE_RAM_ATTR ln_esp8266_pin_isr();
void ICACHE_RAM_ATTR ln_esp8266_timer1_isr();
#endif

#endif
