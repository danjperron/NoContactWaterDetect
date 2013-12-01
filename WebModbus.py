# Imports
import webiopi
import minimalmodbus

# Retrieve GPIO lib
GPIO = webiopi.GPIO

# -------------------------------------------------- #
# Macro definition part                              #
# -------------------------------------------------- #


#Macro to read Modbus register
def ModbusReadRegister(SlaveAddress, Function, Register):
   instrument = minimalmodbus.Instrument('/dev/ttyAMA0',int(SlaveAddress))
   instrument.serial.baudrate = 9600
   instrument.timeout = 0.03
   value = instrument.read_register(int(Register),0,int(Function))
   return str(value)

def ModbusWriteRegister(SlaveAddress, Function, Register, value):
   instrument = minimalmodbus.Instrument('/dev/ttyAMA0',int(SlaveAddress))
   instrument.serial.baudrate = 9600
   instrument.timeout = 0.05
   instrument.write_register(int(Register),int(value),0,int(Function))



# -------------------------------------------------- #
# Initialization part                                #
# -------------------------------------------------- #

# Setup GPIOs

# -------------------------------------------------- #
# Main server part                                   #
# -------------------------------------------------- #

# Instantiate the server on the port 8000, it starts immediately in its own thread
#server = webiopi.Server(port=8000, login="", password="")
server = webiopi.Server(port=8000)

# or     webiopi.Server(port=8000, passwdfile="/etc/webiopi/passwd")

# Register the macros so you can call it with Javascript and/or REST API
server.addMacro(ModbusReadRegister)
server.addMacro(ModbusWriteRegister)

# -------------------------------------------------- #
# Loop execution part                                #
# -------------------------------------------------- #

# Run our loop until CTRL-C is pressed or SIGTERM received
webiopi.runLoop()

# If no specific loop is needed and defined above, just use 
# webiopi.runLoop()
# here instead

# -------------------------------------------------- #
# Termination part                                   #
# -------------------------------------------------- #

# Cleanly stop the server
server.stop()

# Reset GPIO functions
