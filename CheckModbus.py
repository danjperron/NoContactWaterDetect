#!/usr/bin/env python
import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/dev/ttyAMA0',1)
instrument.serial.baudrate= 9600


print "Cailbration for Air   Frequency : ", instrument.read_register(0,0)
print "Calibration for Water Frequency : ", instrument.read_register(1,0)


for i in range(10):
  print "F:", instrument.read_register(0,0,4)
  time.sleep(0.5)
