/****************************************************************************
 * Copyright (C) 2002 Alex Shepherd
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
 *****************************************************************************
 *
 * Title :   LocoNet Software UART Access library (ln_sw_uart.c)
 * Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 * Date:     13-Aug-2002
 * Software:  AVR-GCC with AVR-AS
 * Target:    any AVR device
 *
 * DESCRIPTION
 * Basic routines for interfacing to the LocoNet via any output pin and
 * either the Analog Comparator pins or the Input Capture pin
 *
 * The receiver uses the Timer1 Input Capture Register and Interrupt to detect
 * the Start Bit and then the Compare A Register for timing the subsequest
 * bit times.
 *
 * The Transmitter uses just the Compare A Register for timing all bit times
 *
 * USAGE
 * See the C include ln_interface.h file for a description of each function
 *
 *****************************************************************************/

#if defined(ESP8266)
#  include <Arduino.h>
 // The Arduino standard GPIO routines are not enough,
 // must use some from the Espressif SDK as well
extern "C" {
#  include "gpio.h"
}
#elif defined(STM32F1)
#  include <libopencm3/cm3/nvic.h>
#  include <libopencm3/stm32/exti.h>
#  include <libopencm3/stm32/gpio.h>
#  include <libopencm3/stm32/rcc.h>
#  include <libopencm3/stm32/timer.h>
#else
#  include <avr/io.h>
#  include <avr/interrupt.h>
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#  include "Arduino.h"
#else
#  include "WProgram.h"
#endif

#include <string.h>
#include "ln_config.h"
#include "LocoNet.h"
#include "ln_buf.h"    
#include "ln_sw_uart.h"    

volatile uint8_t  lnState;
volatile uint8_t  lnBitCount;
volatile uint8_t  lnCurrentByte;
volatile LnCompareTargetType lnCompareTarget;

bool checkStartBit = false;

LnBuf* lnRxBuffer;
volatile lnMsg* volatile lnTxData;
volatile uint8_t  lnTxIndex;
volatile uint8_t  lnTxLength;
volatile uint8_t  lnTxSuccess;   // this boolean flag as a message from timer interrupt to send function
#ifdef ESP8266
volatile uint8_t  lnLastTxBit;
#endif

#ifndef ESP8266
volatile uint8_t* txPort;
#else
LnPortAddrType txPort;
#endif
uint8_t           txPin;

#ifndef ESP8266
#  define LN_TX_PORT *txPort
#  define LN_TX_BIT  txPin
#else
#  define LN_TX_PORT txPin
#  define LN_TX_BIT
#endif

void setTxPortAndPin(LnPortAddrType newTxPort, uint8_t newTxPin)
{
#ifndef ESP8266  
	txPort = newTxPort;
#else
	// Mute unused parameter warning
	(void)(newTxPort);
#endif
	txPin = newTxPin;
}

#ifdef ESP8266
bool ICACHE_RAM_ATTR isLocoNetCollision()
{
#ifdef LN_SW_UART_TX_INVERTED
#  ifdef LN_SW_UART_RX_INVERTED
	bool result = (lnLastTxBit != digitalRead(LN_RX_PORT));
#  else
	bool result = (lnLastTxBit == digitalRead(LN_RX_PORT));
#  endif
#else
#  ifdef LN_SW_UART_RX_INVERTED
	bool result = (lnLastTxBit == digitalRead(LN_RX_PORT));
#  else
	bool result = (lnLastTxBit != digitalRead(LN_RX_PORT));
#  endif
#endif

	return result;
}
#endif

#if defined(STM32F1)
#  define bit_is_set(PORT, PIN) (((PORT >> PIN) & 0x01) != 0)
#  define bit_is_clear(PORT, PIN) (((PORT >> PIN) & 0x01) == 0)
#endif

/**************************************************************************
 *
 * Start Bit Interrupt Routine
 *
 * DESCRIPTION
 * This routine is executed when a falling edge on the incoming serial
 * signal is detected. It disables further interrupts and enables
 * timer interrupts (bit-timer) because the UART must now receive the
 * incoming data.
 *
 **************************************************************************/

#if defined(ESP8266)
void ICACHE_RAM_ATTR ln_esp8266_pin_isr()
#else
ISR(LN_SB_SIGNAL)
#endif
{
#if defined(STM32F1)
	// Check if it really was EXTI14 that triggered this interrupt.
	if (!exti_get_flag_status(EXTI14)) {
		// Ignore any interrupt that is not EXTI14.
		return;
	}
#endif

#if defined(ESP8266)
	// Disable the pin interrupt
	detachInterrupt(digitalPinToInterrupt(LN_RX_PORT));

	// Attach timer interrupt handler and restart timer
	timer1_attachInterrupt(ln_esp8266_timer1_isr);
	timer1_write(LN_TIMER_RX_START_PERIOD);
#else
	// Disable the Input Comparator Interrupt
	LN_CLEAR_START_BIT_FLAG();
	LN_DISABLE_START_BIT_INTERRUPT();

#  if defined(STM32F1)
	lnCompareTarget = timer_get_counter(TIM2) + LN_TIMER_RX_START_PERIOD;
	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, lnCompareTarget);
#  else  // Get the Current Timer1 Count and Add the offset for the Compare target
	lnCompareTarget = LN_TMR_INP_CAPT_REG + LN_TIMER_RX_START_PERIOD;
	LN_TMR_OUTP_CAPT_REG = lnCompareTarget;
#  endif

	// Clear the current Compare interrupt status bit and enable the Compare interrupt
	LN_CLEAR_TIMER_FLAG();
	LN_ENABLE_TIMER_INTERRUPT();
#endif

	// Set the State to indicate that we have begun to Receive
	lnState = LN_ST_RX;

	// Reset the bit counter so that on first increment it is on 0
	lnBitCount = 0;
	
	// Next Bit ist Startbit
	checkStartBit=true;


#if defined(ESP8266)
	// Must clear this bit in the interrupt register,
	// it gets set even when interrupts are disabled
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << LN_RX_PORT);
#endif
}

/**************************************************************************
 *
 * Timer Tick Interrupt
 *
 * DESCRIPTION
 * This routine coordinates the transmition and reception of bits. This
 * routine is automatically executed at a rate equal to the baud-rate. When
 * transmitting, this routine shifts the bits and sends it. When receiving,
 * it samples the bit and shifts it into the buffer.
 *
 **************************************************************************/
#if defined(ESP8266)
void ICACHE_RAM_ATTR ln_esp8266_timer1_isr()
#else
ISR(LN_TMR_SIGNAL)     /* signal handler for timer0 overflow */
#endif
{
	// Advance the Compare Target by a bit period
#if defined(ESP8266)
	timer1_write(LN_TIMER_RX_RELOAD_PERIOD);
#else
	LN_CLEAR_TIMER_FLAG();
	lnCompareTarget += LN_TIMER_RX_RELOAD_PERIOD;
#  if defined(STM32F1)
	timer_set_oc_value(TIM2, TIM_OC1, lnCompareTarget);
#  else
	LN_TMR_OUTP_CAPT_REG = lnCompareTarget;
#  endif
#endif

	// Check if there is really a start bit or just a glitch
	if (checkStartBit) {
		checkStartBit = false;
#ifdef LN_SW_UART_RX_INVERTED
		if (bit_is_clear(LN_RX_PORT, LN_RX_BIT)) {
#else
		if (bit_is_set(LN_RX_PORT, LN_RX_BIT)) {
#endif
		  lnState = LN_ST_CD_BACKOFF;
#if defined(ESP8266)
		   // Enable the pin interrupt
#ifdef LN_SW_UART_RX_INVERTED  
		   attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, RISING);
#else
		   attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, FALLING);
#endif
#else
		   // Clear the Start Bit Interrupt Status Flag and Enable ready to 
		   // detect the next Start Bit
		   LN_CLEAR_START_BIT_FLAG();
		   LN_ENABLE_START_BIT_INTERRUPT();
#endif
		}
		return;
	}
	
	lnBitCount++;                // Increment bit_counter

	if (lnState == LN_ST_RX) {  // Are we in RX mode
		if (lnBitCount < 9) {     // Are we in the Stop Bits phase
			lnCurrentByte >>= 1;
#ifdef LN_SW_UART_RX_INVERTED
			if (bit_is_clear(LN_RX_PORT, LN_RX_BIT)) {
#else		
			if (bit_is_set(LN_RX_PORT, LN_RX_BIT)) {
#endif
				lnCurrentByte |= 0x80;
			}
			return;
		}

#if defined(ESP8266)
		// Enable the pin interrupt
#ifdef LN_SW_UART_RX_INVERTED  
		attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, RISING);
#else
		attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, FALLING);
#endif
#else
		// Clear the Start Bit Interrupt Status Flag and Enable ready to 
		// detect the next Start Bit
		LN_CLEAR_START_BIT_FLAG();
		LN_ENABLE_START_BIT_INTERRUPT();
#endif

		// If the Stop bit is not Set then we have a Framing Error
#ifdef LN_SW_UART_RX_INVERTED  
		if (bit_is_set(LN_RX_PORT, LN_RX_BIT)) {
#else
		if (bit_is_clear(LN_RX_PORT, LN_RX_BIT)) {
#endif		
			// ERROR_LED_ON();
			lnRxBuffer->Stats.RxErrors++;
		}
		else { // Put the received byte in the buffer
			addByteLnBuf(lnRxBuffer, lnCurrentByte);
			if (notifyLnByteReceived != 0) {
				notifyLnByteReceived();
			}
		}

		lnBitCount = 0;
		lnState = LN_ST_CD_BACKOFF;
	}

	if (lnState == LN_ST_TX) {   // Are we in the TX State
		// To get to this point we have already begun the TX cycle so we need to 
		// first check for a Collision. 
		if (IS_LN_COLLISION()) {			 // Collision?
			lnBitCount = 0;
			lnState = LN_ST_TX_COLLISION;
			// ERROR_LED_ON();
		}
		else if (lnBitCount < 9) {   			 // Send each Bit
			if (lnCurrentByte & 0x01) {
				LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT);
			}
			else {
				LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT);
			}
			lnCurrentByte >>= 1;
		}
		else if (lnBitCount == 9) {   		 // Generate stop-bit
			LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT);
		}
		else if (++lnTxIndex < lnTxLength) {  // Any more bytes in buffer
			// Setup for the next byte
			lnBitCount = 0;
			lnCurrentByte = lnTxData->data[lnTxIndex];

			// Begin the Start Bit
			LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT);
#if defined(ESP8266)
			timer1_write(LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST);
#else
			// Get the Current Timer1 Count and Add the offset for the Compare target
			// added adjustment value for bugfix (Olaf Funke)
#  if defined(STM32F1)
			lnCompareTarget = timer_get_counter(TIM2) + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST;
			timer_set_oc_value(TIM2, TIM_OC1, lnCompareTarget);
#  else
			lnCompareTarget = LN_TMR_COUNT_REG + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST;
			LN_TMR_OUTP_CAPT_REG = lnCompareTarget;
#  endif
#endif
		}
		else {
			// Successfully Sent all bytes in the buffer
			// so set the Packet Status to Done
			lnTxSuccess = 1;

			// Now copy the TX Packet into the RX Buffer
#if (defined(LN_TX_ECHO) && (LN_TX_ECHO) != 0)
			addMsgLnBuf(lnRxBuffer, lnTxData);
			if (notifyLnByteReceived != 0) {
				notifyLnByteReceived();
			}
#endif

			// Begin CD Backoff state
			lnBitCount = 0;
			lnState = LN_ST_CD_BACKOFF;
		}
	}

	// Note we may have got here from a failed TX cycle, if so BitCount will be 0
	if (lnState == LN_ST_TX_COLLISION) {
		if (lnBitCount == 0) {
			// Pull the TX Line low to indicate Collision
			LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT);
			// ERROR_LED_ON();
		}
		else if (lnBitCount >= LN_COLLISION_TICKS) {
			// Release the TX Line
			LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT);
			// ERROR_LED_OFF();

			lnBitCount = 0;
			lnState = LN_ST_CD_BACKOFF;

			lnRxBuffer->Stats.Collisions++;
		}
	}

	if (lnState == LN_ST_CD_BACKOFF) {
		if (lnBitCount == 0) {
			// Even though we are waiting, other nodes may try and transmit early
			// so Clear the Start Bit Interrupt Status Flag and Enable ready to 
			// detect the next Start Bit
#if defined(ESP8266)
#ifdef LN_SW_UART_RX_INVERTED  
			attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, RISING);
#else
			attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, FALLING);
#endif
#else
			LN_CLEAR_START_BIT_FLAG();
			LN_ENABLE_START_BIT_INTERRUPT();
#endif
		}
		else if (lnBitCount >= LN_BACKOFF_MAX) {
			// declare network to free after maximum backoff delay
			lnState = LN_ST_IDLE;
#if defined(ESP8266)
			timer1_detachInterrupt();
#else
			LN_DISABLE_TIMER_INTERRUPT();
#endif
		}
	}
}


void initLocoNetHardware(LnBuf * RxBuffer)
{
	lnRxBuffer = RxBuffer;

	// Set the RX line to Input
#if defined(ESP8266)
	pinMode(LN_RX_PORT, INPUT);
#elif defined(STM32F1)
	pinMode(LN_RX_PIN_NAME, INPUT);
#else
	cbi(LN_RX_DDR, LN_RX_BIT);
#endif

	// Set the TX line to Inactive
	LN_SW_UART_SET_TX_HIGH(LN_TX_PORT, LN_TX_BIT);

#if defined(ESP8266)
	timer1_detachInterrupt();
	timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);	//remove divider to make timing more accurate
#elif defined(STM32F1)
	// === Setup the timer ===

	// Enable TIM2 clock. 
	rcc_periph_clock_enable(RCC_TIM2);

	// Enable TIM2 interrupt.
	nvic_set_priority(NVIC_TIM2_IRQ, LN_TMR_ISR_PRIO);
	nvic_enable_irq(NVIC_TIM2_IRQ);

	// Reset TIM2 peripheral to defaults.
	rcc_periph_reset_pulse(RST_TIM2);

	// Timer global mode:
	//  - No divider
	//  - Alignment edge
	//  - Direction up
	//  (These are actually default values after reset above, so this call
	//  is strictly unnecessary, but demos the api for alternative settings)
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	// Please take note that the clock source for STM32 timers
	// might not be the raw APB1/APB2 clocks.  In various conditions they
	// are doubled.  See the Reference Manual for full details!
	// In our case, TIM2 on APB1 is running at double frequency, so this
	// sets the prescaler to have the timer run at 5kHz
	//
	//timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));

	// Disable preload.
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	// count full range, as we'll update compare value continuously
	timer_set_period(TIM2, 65535);

	// Set the initual output compare value for OC1.
	//timer_set_oc_value(TIM2, TIM_OC1, lnBitTime);

	// Counter enable.
	timer_enable_counter(TIM2);

	// Enable Channel 1 compare interrupt to recalculate compare values
	//timer_enable_irq(TIM2, TIM_DIER_CC1IE);

  // Setup level change interrupt
	exti_select_source(EXTI14, GPIOB);
#  ifdef LN_SW_UART_RX_INVERTED  
	exti_set_trigger(EXTI14, EXTI_TRIGGER_RISING);  // Experiment: Use both
#  else
	exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);  // Experiment: Use both
#  endif

	lnState = LN_ST_IDLE;

	exti_reset_request(EXTI14);
	exti_enable_request(EXTI14);
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);

#else
#  ifdef LN_INIT_COMPARATOR
	LN_INIT_COMPARATOR();
#  else
	// First Enable the Analog Comparitor Power, 
	// Set the mode to Falling Edge
	// Enable Analog Comparator to Trigger the Input Capture unit
	// ACSR = (1<<ACI) | (1<<ACIS1) | (1<<ACIC) ;

	// Turn off the Analog Comparator
	ACSR = 1 << ACD;
	// The noise canceler is enabled by setting the Input Capture Noise Canceler (ICNCn) bit in 
	// Timer/Counter Control Register B (TCCRnB). When enabled the noise canceler introduces addi- 
	// tional four system clock cycles of delay from a change applied to the input, to the update of the 
	// ICRn Register. The noise canceler uses the system clock and is therefore not affected by the 
	// prescaler.
	TCCR1B |= (1 << ICNC1);    		// Enable Noise Canceler 
#  endif
#endif

	lnState = LN_ST_IDLE;

#if defined(ESP8266)
#  ifdef LN_SW_UART_RX_INVERTED  
	attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, RISING);
#  else
	attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, FALLING);
#  endif
#else
	//Clear StartBit Interrupt flag
	LN_CLEAR_START_BIT_FLAG();
	//Enable StartBit Interrupt
	LN_ENABLE_START_BIT_INTERRUPT();

	//Set rising edge for StartBit if signal is inverted
#  ifdef LN_SW_UART_RX_INVERTED  
	sbi(LN_SB_EDGE_CFG_REG, LN_SB_EDGE_BIT);
#  endif

	// Set Timer Clock Source 
	LN_TMR_CONTROL_REG = (LN_TMR_CONTROL_REG & 0xF8) | LN_TMR_PRESCALER;
#endif // STM32F1 
}


LN_STATUS sendLocoNetPacketTry(lnMsg * TxData, unsigned char ucPrioDelay)
{
	uint8_t  CheckSum;
	uint8_t  CheckLength;

	lnTxLength = getLnMsgSize(TxData);

	// First calculate the checksum as it may not have been done
	CheckLength = lnTxLength - 1;
	CheckSum = 0xFF;

	for (lnTxIndex = 0; lnTxIndex < CheckLength; lnTxIndex++) {
		CheckSum ^= TxData->data[lnTxIndex];
	}

	TxData->data[CheckLength] = CheckSum;

	// clip maximum prio delay
	if (ucPrioDelay > LN_BACKOFF_MAX) {
		ucPrioDelay = LN_BACKOFF_MAX;
	}
	// if priority delay was waited now, declare net as free for this try
	noInterrupts();  // disabling interrupt to avoid confusion by ISR changing lnState while we want to do it
	if (lnState == LN_ST_CD_BACKOFF) {
		if (lnBitCount >= ucPrioDelay) {	// Likely we don't want to wait as long as
			lnState = LN_ST_IDLE;			// the timer ISR waits its maximum delay.

#if defined(ESP8266)
			timer1_detachInterrupt();
#else
			LN_DISABLE_TIMER_INTERRUPT();
#endif
		}
	}
	interrupts();  // a delayed start bit interrupt will happen now,
	// a delayed timer interrupt was stalled


	// If the Network is not Idle, don't start the packet
	if (lnState == LN_ST_CD_BACKOFF) {
		if (lnBitCount < LN_CARRIER_TICKS) {  // in carrier detect timer?
			return LN_CD_BACKOFF;
		}
		else {
			return LN_PRIO_BACKOFF;
		}
	}

	if (lnState != LN_ST_IDLE) {
		return LN_NETWORK_BUSY;  // neither idle nor backoff -> busy
	}
	// We need to do this with interrupts off.
	// The last time we check for free net until sending our start bit
	// must be as short as possible, not interrupted.
	noInterrupts();

#if defined(ESP8266)
	// Before we do anything else - Disable StartBit Interrupt
	detachInterrupt(digitalPinToInterrupt(LN_RX_PORT));
#  ifdef LN_SW_UART_RX_INVERTED  
	if (bit_is_set(LN_RX_PORT, LN_RX_BIT)) {
#  else
	if (bit_is_clear(LN_RX_PORT, LN_RX_BIT)) {
#  endif
#else
	// Before we do anything else - Disable StartBit Interrupt
	LN_DISABLE_START_BIT_INTERRUPT();
#  if defined(STM32F1)
	if (exti_get_flag_status(EXTI14)) {
#  else
	if (bit_is_set(LN_SB_INT_STATUS_REG, LN_SB_INT_STATUS_BIT)) {
#  endif
#endif
		// first we disabled it, than before sending the start bit, we found out
		// that somebody was faster by examining the start bit interrupt request flag
#if defined(ESP8266)
#  ifdef LN_SW_UART_RX_INVERTED  
		attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, RISING);
#  else
		attachInterrupt(digitalPinToInterrupt(LN_RX_PORT), ln_esp8266_pin_isr, FALLING);
#  endif
#else
		LN_ENABLE_START_BIT_INTERRUPT();
#endif

		interrupts();  // receive now what our rival is sending
		return LN_NETWORK_BUSY;
	}

	LN_SW_UART_SET_TX_LOW(LN_TX_PORT, LN_TX_BIT);        // Begin the Start Bit

#if !defined(ESP8266)
  // Get the Current Timer1 Count and Add the offset for the Compare target
  // added adjustment value for bugfix (Olaf Funke)
#  if defined(STM32F1)
	lnCompareTarget = timer_get_counter(TIM2) + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST;
	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, lnCompareTarget);
#  else
	lnCompareTarget = LN_TMR_COUNT_REG + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST;
	LN_TMR_OUTP_CAPT_REG = lnCompareTarget;
#  endif
#endif

	interrupts();  // Interrupts back on ...

	lnTxData = TxData;
	lnTxIndex = 0;
	lnTxSuccess = 0;

	// Load the first Byte
	lnCurrentByte = TxData->data[0];

	// Set the State to Transmit
	lnState = LN_ST_TX;

	// Reset the bit counter
	lnBitCount = 0;

#if defined(ESP8266)
	timer1_attachInterrupt(ln_esp8266_timer1_isr);
	timer1_write(LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST);
#else
	// Clear the current Compare interrupt status bit and enable the Compare interrupt
	LN_CLEAR_TIMER_FLAG();
	LN_ENABLE_TIMER_INTERRUPT();
#endif

	while (lnState == LN_ST_TX) {
		// now busy wait until the interrupts do the rest
	}
	if (lnTxSuccess) {
		lnRxBuffer->Stats.TxPackets++;
		return LN_DONE;
	}
	if (lnState == LN_ST_TX_COLLISION) {
		return LN_COLLISION;
	}
	return LN_UNKNOWN_ERROR; // everything else is an error
}
