#!/usr/bin/env python
import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/dev/ttyAMA0',127)
instrument.serial.baudrate= 9600


print "Cailbration for Air   Frequency : ", instrument.read_register(0,0)
print "Calibration for Water Frequency : ", instrument.read_register(1,0)


while(True):
  print "F:", instrument.read_register(0,0,4)
  time.sleep(0.2)
