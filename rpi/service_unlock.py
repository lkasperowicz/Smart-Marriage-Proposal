import serial
from time import sleep

PROPOSAL_INTERFACE = "/dev/ttyUSB0"

proposal_interface = serial.Serial(
        port=PROPOSAL_INTERFACE,
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS)

#startup delay
sleep(2)

proposal_interface.write(b"unlock\n")