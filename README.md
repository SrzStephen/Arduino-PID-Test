# Arduino-PID-Test
Testing using two PT100 thermistors + LM358 to drive a constant current source
Very rough as it was just a test. In the end instead of trying to guess the output of the wheatstone bridge, I Just did a linear regression on the op amps output for an input of temprerature, making the whole "do all the maths" method kinda dumb.

NOTE: I've made this non-private in case anyone needs to check that i've done something with an arduino other than making some LEDS flash.
