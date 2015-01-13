/*
 *  Loconet Control Panel - simple demo
 *  2014 John Plocher - Public Domain
 *
 *  Circuit:
 *    Loconet on pins 8 (RX on UNO) and 7 (TX) (via a LocoShield or similar)
 *    Push Buttons on pins 5 and 6 (making a connection to ground when pressed)
 *    LED on pin 13 (lit when NORMAL)
 *
 *  Code:
 *    Use the two pushbuttons to control a turnout somewhere else on the layout
 *    by way of a Loconet layout control bus.
 *        press the "N" button and genetate a loconet "set turnout to Normal" message,
 *        press the "D" button and generate "set turnout to Diverging"
 *        make sure the LED is on when the turnout is Normal, even if someone else controls
 *          the turnout with a throttle or JMRI...
 *
 *    The turnout number is arbitrary and should be changed to match your layout...
 */

#include <LocoNet.h>

// #define LN_TX_PIN     47  // MEGA
// #define LN_TX_PIN     6   // UNO
#define LN_TX_PIN     7		// LocoShield
//      LN_RX_PIN     Hardcoded in library for UNO and MEGA.  Will not work with Leonardo (yet)

#define LED_PIN      13
#define N_BUTTON_PIN  6
#define D_BUTTON_PIN  5
#define LN_TURNOUT   25  // change to match your layout...

// Loconet turnout position definitions to make code easier to read
#define TURNOUT_NORMAL     1  // aka closed
#define TURNOUT_DIVERGING  0  // thrown

lnMsg  *LnPacket;          // pointer to a received LNet packet
int     nbit1, nbit2;      // used to debounce "N" pushbutton
int     dbit1, dbit2;      // used to debounce "D" pushbutton
int     led_state;         // on (1) or off (0)
int     nbutton, dbutton;  // on (1) or off (0)
int     last_nbutton, last_dbutton;  // only want to generate one loconet message per button press


// Construct a Loconet packet that requests a turnout to set/change its state
void sendOPC_SW_REQ(int address, byte dir, byte on) {
    lnMsg SendPacket ;
    
    int sw2 = 0x00;
    if (dir == TURNOUT_NORMAL) { sw2 |= B00100000; }
    if (on) { sw2 |= B00010000; }
    sw2 |= (address >> 7) & 0x0F;
    
    SendPacket.data[ 0 ] = OPC_SW_REQ ;
    SendPacket.data[ 1 ] = address & 0x7F ;
    SendPacket.data[ 2 ] = sw2 ;
    
    LocoNet.send( &SendPacket );
}

// Some turnout decoders (DS54?) can use solenoids, this code emulates the digitrax 
// throttles in toggling the "power" bit to cause a pulse
void setLNTurnout(int address, byte dir) {
    sendOPC_SW_REQ(address - 1, dir, 1);
    sendOPC_SW_REQ(address - 1, dir, 0);
}

void setup() {
    pinMode(LED_PIN,      OUTPUT); 
    pinMode(N_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(D_BUTTON_PIN, INPUT_PULLUP); 
    digitalWrite(LED_PIN, LOW);
    LocoNet.init(LN_TX_PIN);
    led_state = 0;
    last_nbutton = last_dbutton = -1;
}

void loop() {  
    
    // read the various input bits
    nbit1 = digitalRead(N_BUTTON_PIN);
    dbit1 = digitalRead(D_BUTTON_PIN);
    delay(10);  // wait for mechanical switch bounces to subside
    nbit2 = digitalRead(N_BUTTON_PIN);
    dbit2 = digitalRead(D_BUTTON_PIN);

    // clean them up, interpret them and make decisions    
    
    // identical readings mean we have a good result
    if (nbit1 == nbit2) {        // Was "N"ormal pushbutton pressed?
        if (nbit1 == LOW) {
            nbutton = 1;
        } else {
            nbutton = 0;
        }
    }
    if (dbit1 == dbit2) {        // Was "D"iverging pushbutton pressed?
        if (dbit1 == LOW) {
            dbutton = 1;
        } else {
            dbutton = 0;
        }
    } else {  // no valid actions to perform
        nbutton = 0;
        dbutton = 0;
    }
    
    
    // now that we have clean inputs, implement the "business logic" that
    // you want - in this case, the buttons should control a turnout...
    if ((nbutton == 1) && (dbutton == 1)) {
        // ERROR - someone is leaning on the fascia and pressing both buttons...
        //         ignore it.
    } else if ((nbutton == 1) && (nbutton == last_nbutton)) {
        // Already processed this N button push, wait for the bozo to take their thumb off the button!
    } else if ((dbutton == 1) && (dbutton == last_dbutton)) {
        // Already processed this D button push, wait for the bozo to take their other thumb off the button!
    } else if (nbutton == 1) {
        // Set the turnout to Normal
        led_state = 1;
        setLNTurnout(LN_TURNOUT, TURNOUT_NORMAL);
    } else if (dbutton == 1) {
        // Set the turnout to Diverging
        led_state = 0;
        setLNTurnout(LN_TURNOUT, TURNOUT_DIVERGING);
    } else {
        // both must be 0 - nothing pressed, nothing to do
    }
    last_nbutton = nbutton;  // remember the state of the
    last_dbutton = dbutton;  // buttons for next time...


    // done with our "local" logic, now we need to check for any loconet packets that might
    // influence our control panel, such as when someone else throws the turnout from a throttle.
    // We want to catch these events so the LED follows the turnout's true state
    
    // Check for any received LocoNet packets
    LnPacket = LocoNet.receive() ;
    if( LnPacket ) {   
        LocoNet.processSwitchSensorMessage(LnPacket);
        // this function will call the specially named functions below...
    }
    
    // Finally, update the LED to match the turnout position
    digitalWrite(LED_PIN, led_state);
}

// Callbacks from LocoNet.processSwitchSensorMessage() ...
// We tie into the ones connected to turnouts so we can capture anything
// that can change (or indicatea change to) a turnout's position.

void notifySensor( uint16_t Address, uint8_t State )
{ /* ignore, not a turnout related action */ }

void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{ if (Address == LN_TURNOUT) { led_state = Direction; } }

void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{ if (Address == LN_TURNOUT) { led_state = Direction; } }

void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{ if (Address == LN_TURNOUT) { led_state = Direction; } }

