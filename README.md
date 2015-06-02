# alfred-fw

I have tried to package up as many of the necessary files from the RFduino repo into this repo in a nicely structured way.

There are 4 main directories:
* src: This is where our code goes. It also contains some Arduino "library" code as well.
* include: This is where the headers go. I have included the necessary headers from the RFduino repo
* lib: RFduino archived libraries
* core: Arduino core code. This is the stuff Arduino gives to us for free to make our lives easier!

# Toolchain

You will need the gcc-arm-none-eabi toolchain for compiling & linking and the RFDLoader to program the RFduino.
The RFduino repo contains a tarballed MAC OS X version of the gcc-arm-none-eabi toolchain which I have tested and confirmed to work.
You can find other versions here: https://launchpad.net/gcc-arm-embedded

The repo also contains the RFDLoader tool for every platform. You may have to chmod +x it to get it to execute.

# Build

I have created a nice Makefile that makes this process simple:

`make`

`make upload`

Done! Just make sure the toolchain and RFDLoader is in the path and are running it in a UNIX like environment.

# Code

As for the code itself, the most important code provided to us is in the RFduinoBLE.cpp and Wire.cpp files. These are actually just
C++ wrappers around the C libRFduinoBLE and Arduino serial interfaces respectively. They are easier to use than the C interface.
The RFduinoBLE class is essentially all we need to handle bluetooth communication (btw, if you have a Macbook, download the Hardware IO tools
for Xcode to sniff BLE packets. It's what I was using to test the RFduino). The Wire class provides an interface for I2C communication.
There are also functions in wiring_analog.h and wiring_digital.h that provide analog/digital access to the GPIOs. 

