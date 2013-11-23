Capacitive sensor to detect water in pipe using a small cpu PIC12F1840



- WaterDetect.c   Source code in C to be compile via MPLAB (x).
- WaterDetect.hex Compile code to be burn into cpu using Raspberry Pi burnLVP or pickit

Schematic

- NoContactWaterDetectPic12F1840Schema.png      module schematic
- NoContactWaterDetectRpiInterfaceSchema.png    Raspberry Pi interface.
- NoContactWaterDetect.jpg                      Picture of the prototype

MODBUS Version

- modbuslayout.png  Raspberry Pi Modbus layout with water detect module
- rs485switch.c	  Source code to interface Raspberry Pi with RS-485 module
- rs485switch.hex  Hex file version of the compile source code
- RS485Switch.png  Raspberry Pi interface schematic

- WaterDetectModbus.c   Source code of water detect module
- CRC16.c			   CRC 16 bit algorithm source code
- WaterDetectModbus.hex Hex file of the compile source code
- WaterDetectRs485.png  Pic Water Detect module schematic

- CheckM.py        Python code which display how to read frequency of 4 modules.
- CheckModbus.py   Python code to read one module.
- TestSlave127.py  Python code to check cpu after program burn.
- SlaveAddress.py  Python code to change module slave address.

