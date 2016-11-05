# TeensyCore3-FRDM-K64F
Eclipse project for building TeensyCore3 for the [NXP FRDM-K64F board](http://www.nxp.com/products/software-and-tools/hardware-development-tools/freedom-development-boards/freedom-development-platform-for-kinetis-k64-k63-and-k24-mcus:FRDM-K64F),
an ARM Cortex-M4 development board.

This will likely not work for you, since there is a ton of setup that you have to do first. At some point I will put together
instructions, but they will be extremely fragile because that's what you get with software.

I got a lot of the setup instructions from [here](http://gnuarmeclipse.github.io/), and then I hacked [TeensyCore3](https://github.com/PaulStoffregen/cores/tree/master/teensy3)
to get just the minimal functionality working on the FRDM-K64F.

I redirected Arduino pin 0 to the green LED, and got USB serial working, but that is all for now.
