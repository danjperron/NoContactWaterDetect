#!/usr/bin/env python

import sys
import minimalmodbus


if len(sys.argv) == 3:
   #Get the current SlaveAddress
   SlaveAddress = int(sys.argv[1])
   NewAddress   = int(sys.argv[2])
else:
   print "Usage: {} Current_Address New_Address".format(sys.argv[0])
   exit(1)

print "Current Address is {}".format(SlaveAddress)
print "New Address will be {}",format(NewAddress)

instrument = minimalmodbus.Instrument('/dev/ttyAMA0',SlaveAddress)
instrument.serial.baudrate= 9600
instrument.serial.timeout=0.1
instrument.debug = True

instrument.write_register(2,NewAddress,0,6);

