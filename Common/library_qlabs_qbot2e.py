from library_qlabs import QuanserInteractiveLabs, CommModularContainer
from quanser.common import GenericError
import math

import struct
import os
        
        
######################### MODULAR CONTAINER CLASS #########################

class QLabsQBot2e:

       
    ID_QBOT = 20
    FCN_QBOT_REQUEST_GLOBAL_TRANSFORM = 3
    FCN_QBOT_GLOBAL_TRANSFORM_RESPONSE = 4
    FCN_QBOT_COMMAND_AND_REQUEST_STATE = 10
    FCN_QBOT_COMMAND_AND_REQUEST_STATE_RESPONSE = 11
    FCN_QBOT_POSSESS = 20
    FCN_QBOT_POSSESS_ACK = 21
    
    
    VIEWPOINT_RGB = 0
    VIEWPOINT_DEPTH = 1
    VIEWPOINT_TRAILING = 2
    
    # Initialize class
    def __init__(self):

       return
       
    def spawn(self, qlabs, deviceNumber, location, rotation, configuration=0, waitForConfirmation=True):
        return qlabs.spawn(deviceNumber, self.ID_QBOT, location[0], location[1], location[2]+0.1, rotation[0], rotation[1], rotation[2], 1.0, 1.0, 1.0, configuration, waitForConfirmation)
   
    def spawnDegrees(self, qlabs, deviceNumber, location, rotation, configuration=0, waitForConfirmation=True):
        rotation[0] = rotation[0]/180*math.pi
        rotation[1] = rotation[1]/180*math.pi
        rotation[2] = rotation[2]/180*math.pi    
    
        return qlabs.spawn(deviceNumber, self.ID_QBOT, location[0], location[1], location[2]+0.1, rotation[0], rotation[1], rotation[2], 1.0, 1.0, 1.0, configuration, waitForConfirmation)
   
   
    def possess(self, qlabs, deviceNumber, camera):
        c = CommModularContainer()
        c.classID = self.ID_QBOT
        c.deviceNumber = deviceNumber
        c.deviceFunction = self.FCN_QBOT_POSSESS
        c.payload = bytearray(struct.pack(">B", camera))
        c.containerSize = c.BASE_CONTAINER_SIZE + len(c.payload)
        
        qlabs.flushReceive()  
        
        if (qlabs.sendContainer(c)):
            c = qlabs.waitForContainer(self.ID_QBOT, deviceNumber, self.FCN_QBOT_POSSESS_ACK)
                    
            return True
        else:
            return False

    def requestGlobalPosition(self, qlabs, deviceNumber):
        c = CommModularContainer()
        c.classID = self.ID_QBOT
        c.deviceNumber = deviceNumber
        c.deviceFunction = self.FCN_QBOT_REQUEST_GLOBAL_TRANSFORM
        c.payload = bytearray(struct.pack(">B", 0))
        c.containerSize = c.BASE_CONTAINER_SIZE + len(c.payload)
        
        qlabs.flushReceive()  
        
        if (qlabs.sendContainer(c)):
            c = qlabs.waitForContainer(self.ID_QBOT, deviceNumber, self.FCN_QBOT_GLOBAL_TRANSFORM_RESPONSE)
            Location = struct.unpack(">fffffffff", c.payload)
            
            return Location[0:3]
        else:
            return [0,0,0]
        
    def startRTModel(self, device_num=0, QLabs_hostname='localhost'):
        cmd_string="quarc_run -D -r -t tcpip://localhost:17000 QBot2e_Spawn.rt-linux_pi_3 -uri tcpip://localhost:17003 -hostname {} -devicenum {}".format(QLabs_hostname, device_num)
        os.system(cmd_string)
        return cmd_string
        
    def terminateRTModel(self):
        cmd_string="quarc_run -q -Q -t tcpip://localhost:17000 QBot2e_Spawn.rt-linux_pi_3"
        os.system(cmd_string)
        return cmd_string
