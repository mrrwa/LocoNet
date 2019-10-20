/*
 * This debounce routine comes from Scott Dattalo <scott@dattalo.com> 
 * http://www.dattalo.com/technical/software/pic/debounce.html
 *
 * Modified by Alex Shepherd to pass in state as a parameter
 *
 */

#include "vcDebounce.h"

uint8_t vcDebounce( VC_DEBOUNCE_STATE *state, uint8_t new_sample )
{
  uint8_t delta;

  delta = new_sample ^ state->debounced_state;   //Find all of the changes

  state->clock_A ^= state->clock_B;                     //Increment the counters
  state->clock_B  = ~state->clock_B;

  state->clock_A &= delta;                       //Reset the counters if no changes
  state->clock_B &= delta;                       //were detected.

      //Preserve the state of those bits that are being filtered and simultaneously
      //clear the states of those bits that are already filtered.
  state->debounced_state &= (state->clock_A | state->clock_B);

      //Re-write the bits that are already filtered.
  state->debounced_state |= (~(state->clock_A | state->clock_B) & new_sample);

  return state->debounced_state ;
}
