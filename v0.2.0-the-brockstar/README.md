# Version 2: Clear Bass w/ Code Rewrite

## Summary

This is the code for the clear bass created from acrylic. This features the large clear "bass-like" cutout.

This is the iteration that was given to Dean Brock as "The Brockstar", as we had to troubleshoot technical difficulties on our actual "Brockstar" iteration.

Code was updated to be a little more legible, however most changes in this code were just deleting extraneous unused variables/code and renaming a number of variables.



## Notable Hardware

- Teensy 3.2
- BNO-055
- Powered by either DC power supply or USB power
- Ethernet connection between bow and bass body



## Changes

- Added ethernet jack to connect the bass body to the bow
- Added DC power supply
- Cut out of acrylic
- Added Neopixel LEDs to the entire bow and entire bass body
- Renamed L1/L2/L3 to L_nut, L_bridge, and L_between to improve clarity
- LEDs are running using the FastLED library
- Bug fix: Changed the logic of if statements that led to a bowed note being played while in the plucking position.



## Notes

The FastLED library has worse performance than the Neopixel library for updating the colors of the neopixel strips in terms of latency and in bugs. This code is essentially a worse version of the v3-brockstar code, but it is uploaded just for proper documentation.



