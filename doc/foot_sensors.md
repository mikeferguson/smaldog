## Foot Sensor Notes

### Design Explorations

Previous "switch" feet proved problematic. Ideally, would have a non-contact
sensor.

TI makes the LDC1000, which can allegedly measure the displacement of a spring.
However, it appears that twist/etc can also change the inductance.

One of the design notes for the AS5045/5048 magnetic encoder chip is to use it
as a pushbutton, as you can get a fairly nice digital reading of the magnetic
field strength. This is a bit expensive for a set of foot sensors, but has the
nice property of being designed for something like 1mm of travel. Might be
simpler to just use an [analog hall effect sensor](http://www.digikey.com/product-detail/en/SS39ET/480-3845-1-ND/2839726
).

The FC22 load cells are 10lbf rating, with ratiometric output (designed for 5V).
If they will operate on 3.3V, this would give something like 200-600 steps with
the 12-bit ADC of the STM32. Issues to be tested/solved:

 * [ ] Does the sensor run on 3.3V?
 * [ ] How far can we route that analog signal without it getting messed up?
 * [ ] Mechanical packaging.
