# This script is just here to easily modify parameters on the dynamixels
from dynamixel_control import Dynamixel
from time import sleep
# https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/

def update_control_table(address, byte_len, value):
    for id in range(4):
        dc.add_parameter(id = id, address = address, byte_length = byte_len, value = value)
    dc.send_parameters()


dc = Dynamixel(port = '/dev/ttyUSB0')

dc.add_dynamixel(type="XL-330", ID_number=0, calibration=[1023,2048,3073])
dc.add_dynamixel(type="XL-330", ID_number=1, calibration=[1023,2048,3073])
dc.add_dynamixel(type="XL-330", ID_number=2, calibration=[1023,2048,3073]) 
dc.add_dynamixel(type="XL-330", ID_number=3, calibration=[1023,2048,3073])
dc.set_speed(80)
dc.setup_all()
dc.update_PID(1000,400,2000)

update_control_table(11, 1, 5)
dc.reboot_dynamixel()
sleep(2)
for id in range(4):
    dc.enable_torque(id, True)
sleep(1)
start_position = [-1.2, 0, .8, 0]
dc.go_to_position_all(start_position)
# for id in range(4):
    # Loop through all of the dynamixels
print("Sleeping")
sleep(3)
dc.groupBulkRead.clearParam()
for i in range(4):
    dc.groupBulkRead.addParam(i, 70, 1)
    
dc.groupBulkRead.rxPacket()
for id in range(4):
    # Saves position read in each Dxl object
    temp = dc.groupBulkRead.getData(id, 70, 1)
    print(temp)
