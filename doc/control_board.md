##SMALdog2 Control Board

SMALdog2 will likely be using an Intel NUC as the onboard PC. Major things to
still be decided:

 * Choice of IMU
 * Will servos be TTL or RS-485?

##Specifications

 * STM32F407 w/ ethernet connection to PC.
 * ACS711 50A current sensor on main input current.
 * ACS711 25A current sensor on each leg, PC power connection
 * IMU: TBD
 * Physical runstop: MX-64s are terrifying, e-stop needs to physically cut the
   power to them.
 * Voltage measurement: on both the main input, and on the motor side of the
   runstop.
 * Auxillary 12V power headers for Hokuyo laser, NUC computer.
 * Dual battery inputs, possibly real circuitry so that it is save to hotswap
   the batteries.

##Notes

 * May need to use an ethernet PHY that has a spare connection, in order to
   interface to newer ethernet-based Hokuyo lasers.
 * Newer NUCs claim to be 12-24V compatible, however, will they be happy on
   11.1V lipo system? If not, we'll need a step up converter. Someone on the
   Trossen forums noted they are using this with their NUC:
      http://www.amazon.com/gp/product/B008FLE7PA
 * If foot sensors come back to main board analog inputs, will need to provide
   headers for them.
