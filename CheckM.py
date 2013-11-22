#!/usr/bin/env python
import minimalmodbus
import time


FirstModule=1
LastModule=4

ModuleRange = range(FirstModule,LastModule+1,1)

instruments = []
#Set modules
for i in ModuleRange:
  instrument = minimalmodbus.Instrument('/dev/ttyAMA0',i)
  instrument.serial.baudrate=9600
  instrument.serial.timeout=0.05
  instruments.append(instrument)



for n in range(10):
  for instrument in instruments:
    print 'F[{}]: {} \t'.format(instrument.address,instrument.read_register(0,0,4)),
  print " "
