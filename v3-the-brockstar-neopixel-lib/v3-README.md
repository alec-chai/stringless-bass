# Version 3: The Brockstar

## Summary

This is the code for the clear bass created from acrylic that was originally intended to be "The Brockstar." This features the large clear "bass-like" cutout.

This is the iteration that was intended to be given to Dean Brock as "The Brockstar", but we delivered Version 2 to him as we had too many technical difficulties to troubleshoot on our actual "Brockstar" iteration.

Code was updated to be a little more legible, however most changes in this code were just deleting extraneous unused variables/code and renaming a number of variables.



## Notable Hardware

- Teensy 3.2
- BNO-055
- Powered by either DC power supply or USB power
- Ethernet connection between bow and bass body



## Changes

- Using the Neopixel library to control LEDs on the entire bow and entire bass body

- Body is cut out of four different layers. From top to bottom: 1/4 inch, 1/4 inch (hollow), 1/2 inch (hollow), 1/4 inch

  - Cavity inside of the bass body is now 3/4 in thick.

  

## Notes

- The FastLED library has worse performance than the Neopixel library for updating the colors of the neopixel strips in terms of latency and in bugs. In this version of the code, all LED changes are done through the neopixel library.

- This iteration's BNO-055 did not function at all without 4.7 kohm pullup resistors connecting the SDA/SCL lines to ground. Once the pullup resistors were added, the BNO functions quite well. 

- On this iteration, the orientation of the BNO was flipped 180 degrees about the Y-axis, leading to issues with the quaternion code that converts to Euler angles. This was remedied by flipping the sign of the q0prime and q3prime quaternions as:

  - 'q0prime = -quat.w();'
  - 'q3prime = -quat.z();'

  Another suitable solution was to flip the signs of q1 and q2. 

- The pluck velocity code also needed to change in sign, so the code was updated to 'plucksig = -gyro.z();'



