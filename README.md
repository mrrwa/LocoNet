# Overview

The EmbededLocoNet Library provides functions that manage the
sending and receiving of LocoNet packets.

As bytes are received from the LocoNet by an interrupt driven
handler, they are stored in a circular FIFO buffer.
The `LocoNet.receive()` method returns the head packet from the
buffer, or `NULL` if no valid packets have arrived.

When packets are sent successfully, by default they are also appended to the
circular receive buffer so they can be handled in the same manner
as if they had been received from another device.

It maintains statistics about the transmission and reception
of packets.  Also, it discards any invalid packets it receives,
incrementing an RxErrors statistic accordingly.

# Hardware Resource Dependencies

This library uses `TIMER1` & `ICP` (UNO) and/or `TIMER5` and `ICP5` (MEGA)
resources and associated interrupt handler hooks. On the STM32,
it uses `TIM2` and the `EXTI10`-`15` interrupts handler hooks.
On the ESP8266, the implementation uses `TIMER1` and associated
interrupt handler hooks.

Of the AVR "mega" and "tiny" range of microcontrollers, this library only works on those that
have an Input Capture Unit (ICP) associated with a 16-Bit Timer/Counter.

It's known to work with:
- UNO (ATmega328)
- MEGA (ATmega2560)
- Leonardo, LeoStick, Arduino Pro Micro (ATmega32U4)
- Various AVRTiny Boards (ATTiny84, ATTiny84A, ATTiny841)
- Blue Pill (STM32F1 w/ libopencm3 and Arduino Stubs)
- NodeMCU v1.0 (ESP8266 core for Arduino)

As of 2020-03-28 - Hans Tanner added the capability to change the polarity
of the Rx and Tx electrical signal (depending on your circuitry) in the utility/ln_config.h file.
The is controlled using the two #defines `LN_SW_UART_RX_INVERTED` and `LN_SW_UART_TX_INVERTED`  


## Contributing

Update the version in two places:

1. `library.properties` - used by the Arduino IDE 
   via https://github.com/arduino/library-registry#adding-a-library-to-library-manager
2. `library.json` - used by PlatformIO, see https://docs.platformio.org/en/latest/manifests/library-json/index.html

## Releasing

Arduino IDE: add a tag or GitHub Release matching the version number in `library.properties`.
See [the documentation](https://github.com/arduino/library-registry/blob/main/FAQ.md#how-can-i-publish-a-new-release-once-my-library-is-in-the-list).

For PlatformIO, see
[documentation about publishing](https://docs.platformio.org/en/latest/librarymanager/creating.html#publishing).

## IMPORTANT

Some message formats used in this code are Copyright Digitrax,
Inc.  and/or Elektronik GmbH and are used with permission as part
of the MRRwA project.  That permission does not extend to uses in
other software products.  See the LICENSE file for more information.


