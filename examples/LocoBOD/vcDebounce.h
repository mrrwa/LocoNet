/*
 * This debounce routine comes from Scott Dattalo <scott@dattalo.com> 
 * http://www.dattalo.com/technical/software/pic/debounce.html
 *
 * Modified by Alex Shepherd to pass in state as a parameter
 *
 */

#include <stdint.h>

typedef struct
{
  uint8_t clock_A;
  uint8_t clock_B;
  uint8_t debounced_state;
} VC_DEBOUNCE_STATE ;

#if defined (__cplusplus)
	extern "C" {
#endif

uint8_t vcDebounce( VC_DEBOUNCE_STATE *state, uint8_t new_sample ) ;

#if defined (__cplusplus)
}
#endif
