#ifndef _LN_CONFIG_H_INCLUDED
#define _LN_CONFIG_H_INCLUDED

/*
 ****************************************************************************
 * Copyright (C) 2004 Alex Shepherd
 * 
 * Portions Copyright (C) Digitrax Inc.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 ****************************************************************************
 * 2014/10/10 - Tom Knox
 *
 * Changes made to allow library to run with Mega 2560 (and similar) processors
 *
 * To differentiate between UNO and the 2560 I added #ifdef-s with defines
 * the 2560 has that UNO doesn't.  These are all commented with my initials (twk)
 *
 * Thanks to the neatness and modularity of Alex, all the changes are isolated
 * to just the ln_config.h (this) file!
 *
 ****************************************************************************
 * 2014/11/10 - John Plocher
 *
 * refactored to put all the MEGA/UNO changes together in one conditional block
 * to make it easier to port to other chipsets
 *
 ****************************************************************************
 */

// jmp
// figure out what board we are building for
// TODO:  Add support for Leo and others...
#ifdef PINL	//      For the Mega 2560 (should work with 1280, etc)
#define _LNET_USE_MEGA
#else		//	For the UNO:
#define _LNET_USE_UNO
#endif

// Common defines
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#undef  LN_SW_UART_TX_NON_INVERTED  // Normal is to be inverted...
#define LN_BIT_PERIOD               (F_CPU / 16666)
#define LN_TMR_PRESCALER              1
#define LN_TIMER_TX_RELOAD_ADJUST   106 //  14,4 us delay borrowed from FREDI sysdef.h
#define LN_TX_RETRIES_MAX            25


// *****************************************************************************
// *                                                              Arduino MEGA *
// *                            The RX port *MUST BE* the ICP5 pin             *
// *                            (port PINL bit PL1, Arduino pin 48 on a 2560)  *
// *****************************************************************************
#if defined( _LNET_USE_MEGA ) // twk

#define LN_RX_PORT  PINL
#define LN_RX_DDR   DDRL
#define LN_RX_BIT   PL1

// From sysdef.h:
#define LN_SB_SIGNAL          TIMER5_CAPT_vect
#define LN_SB_INT_ENABLE_REG  TIMSK5
#define LN_SB_INT_ENABLE_BIT  ICIE5
#define LN_SB_INT_STATUS_REG  TIFR5
#define LN_SB_INT_STATUS_BIT  ICF5
#define LN_TMR_SIGNAL         TIMER5_COMPA_vect
#define LN_TMR_INT_ENABLE_REG TIMSK5
#define LN_TMR_INT_STATUS_REG TIFR5
#define LN_TMR_INT_ENABLE_BIT OCIE5A
#define LN_TMR_INT_STATUS_BIT OCF5A
#define LN_TMR_INP_CAPT_REG   ICR5      // [BA040319] added defines for:
#define LN_TMR_OUTP_CAPT_REG  OCR5A     // ICR1, OCR1A, TCNT1, TCCR1B
#define LN_TMR_COUNT_REG      TCNT5     // and replaced their occurence in
#define LN_TMR_CONTROL_REG    TCCR5B    // the code.
#define LN_INIT_COMPARATOR() { TCCR5A = 0; TCCR5B = 0x01; }    // no prescaler, normal mode

// *****************************************************************************
// *                                                               Arduino UNO *
// *                              The RX port *MUST BE* the ICP pin            *
// *                              (port PINB bit PB0, Arduino pin 8 on a '168) *
// *****************************************************************************
#elif defined( _LNET_USE_UNO )

#if defined(__AVR_ATmega32U4__)

#define LN_RX_PORT  PIND
#define LN_RX_DDR   DDRD
#define LN_RX_BIT   PORTD4

#else

#define LN_RX_PORT  PINB
#define LN_RX_DDR   DDRB
#ifdef PB0	// bug/missing defines (some hardware core's ioXX.h files define one or the other)
#define LN_RX_BIT   PB0
#else
#define LN_RX_BIT   PORTB0
#endif
#endif

// From sysdef.h:
#define LN_SB_SIGNAL          TIMER1_CAPT_vect
#define LN_SB_INT_ENABLE_REG  TIMSK1
#define LN_SB_INT_ENABLE_BIT  ICIE1
#define LN_SB_INT_STATUS_REG  TIFR1
#define LN_SB_INT_STATUS_BIT  ICF1
#define LN_TMR_SIGNAL         TIMER1_COMPA_vect
#define LN_TMR_INT_ENABLE_REG TIMSK1
#define LN_TMR_INT_STATUS_REG TIFR1
#define LN_TMR_INT_ENABLE_BIT OCIE1A
#define LN_TMR_INT_STATUS_BIT OCF1A
#define LN_TMR_INP_CAPT_REG   ICR1      // [BA040319] added defines for:
#define LN_TMR_OUTP_CAPT_REG  OCR1A     // ICR1, OCR1A, TCNT1, TCCR1B
#define LN_TMR_COUNT_REG      TCNT1     // and replaced their occurence in
#define LN_TMR_CONTROL_REG    TCCR1B    // the code.
#define LN_INIT_COMPARATOR() { TCCR1A = 0; TCCR1B = 0x01; }    // no prescaler, normal mode

// *****************************************************************************
// *                                                       Arduino --UNKNOWN-- *
// *****************************************************************************
#else
#error "No Arduino Board/Processor selected"

#define LN_SB_INT_ENABLE_REG  TIMSK
#define LN_SB_INT_ENABLE_BIT  TICIE1
#define LN_SB_INT_STATUS_REG  TIFR
#define LN_SB_INT_STATUS_BIT  ICF1
#define LN_TMR_INT_ENABLE_REG TIMSK
#define LN_TMR_INT_STATUS_REG TIFR

#endif	// board type
#endif	// include file

