import zaber.serial as zs
import time

"""
small program to communitcate with zabor motor. Decent documentation at
http://www.zaber.com/support/docs/api/core-python/0.8.1/binary.html/
Other motor functions can be performed using the send function with 
an instruction command number. These can be located at 
https://www.zaber.com/wiki/Manuals/Binary_Protocol_Manual#Quick_Command_Reference

"""


#open port for communication
port = zs.BinarySerial("COM11", timeout = 20, inter_char_timeout = 0.05)
#connect to specific device 
device = zs.BinaryDevice(port, 1)

def slice_routine(start_pos, end_pos, num_steps):
    device.move_abs(start_pos)
    step_size = (end_pos - start_pos) // num_steps
    for i in range(num_steps):
    	device.move_rel(step_size)
    	frog = device.send(60,0)
    	print(frog)
#find location of motor on rails - uses send functions with quick command number
reply = device.send(60,0)
print reply.data
# device.home()
device.move_rel(0)
reply = device.send(60,0)
print reply.data
slice_routine(9000, 90000, 10)

#close port so other applications can access motor
port.close()