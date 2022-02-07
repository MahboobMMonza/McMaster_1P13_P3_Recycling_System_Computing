from library_qlabs import QuanserInteractiveLabs
from quanser.common import GenericError
import math

import struct
import os
        
        
######################### MODULAR CONTAINER CLASS #########################

class QLabsQArm:

    # Define class-level variables   
    container_size = 0
    class_id = 0       # What device type is this?
    device_number = 0   # Increment if there are more than one of the same device ID
    device_function = 0 # Command/reponse
    payload = bytearray()
    
       
    ID_QARM = 10
    
    # Initilize class
    def __init__(self):

       return
       
    def spawn(self, qlabs, device_num, location, rotation, configuration=0, wait_for_confirmation=True):
        return qlabs.spawn(device_num, self.ID_QARM, location[0], location[1], location[2], rotation[0], rotation[1], rotation[2], 1.0, 1.0, 1.0, configuration, wait_for_confirmation)
   
    def spawnDegrees(self, qlabs, device_num, location, rotation, configuration=0, wait_for_confirmation=True):
        rotation[0] = rotation[0]/180*math.pi
        rotation[1] = rotation[1]/180*math.pi
        rotation[2] = rotation[2]/180*math.pi 
        return qlabs.spawn(device_num, self.ID_QARM, location[0], location[1], location[2], rotation[0], rotation[1], rotation[2], 1.0, 1.0, 1.0, configuration, wait_for_confirmation)
 
    def startRTModel(self, device_num=0, QLabs_hostname='localhost'):
        cmd_string="quarc_run -D -r -t tcpip://localhost:17000 QArm_Spawn.rt-linux_pi_3 -uri tcpip://localhost:17002 -hostname {} -devicenum {}".format(QLabs_hostname, device_num)
        os.system(cmd_string)
        return cmd_string
        
    def terminateRTModel(self):
        cmd_string="quarc_run -q -Q -t tcpip://localhost:17000 QArm_Spawn.rt-linux_pi_3".format()
        os.system(cmd_string)
        return cmd_string
