# This code represents the library that students will reference to run the Q-Arm
# simulation for Project 2.
#
# Current code is set up to run on a raspberry pi but simple modification are required to
# allow for the code to run on a computer. A couple libraries will need to be installed
# here in order to prevent the students from having to install them.
#
# **** Note: After installing all the libraries on computer for a computer test run, an
# error was generated. So far, guaranteed execution is only when a Raspberry Pi is used.**** 

# Items to mention to students
# 1. Rotation limits base +/- 175 deg, shoulder +/- 90 deg, elbow +90 -80 deg, wrist +/-170, gripper 0(open)-1(close)
# 2. For the autoclaves, 0 = closed >0 = open

# Import all required libraries
import numpy as np

import time
import math
import struct
import os
import keyboard
import cv2
import random
import sys
sys.path.append('../Common/')
# Quanser Equipment Library
from McMaster_HIL_lib import *

# General Project Environment Configuration Libraries
from library_qlabs import QuanserInteractiveLabs
from library_qlabs_free_camera import QLabsFreeCamera

# P0 Environment Configuration Libraries
from library_qlabs_spline_line import QLabsSplineLine
from library_qlabs_qbot2e import QLabsQBot2e
import library_qlabs_utilities as QLabsUtils

# P2A and P2B Environment Configuration Libraries
from library_qlabs_qarm import QLabsQArm
from library_qlabs_delivery_tube import QLabsDeliveryTube
from library_qlabs_basic_shape import QLabsBasicShape
from library_qlabs_autoclave import QLabsAutoclave
#from library_qlabs_shredder import QLabsShredder
from library_qlabs_widget import QLabsWidget

# P3 Environment Configuration Libraries
# (Excluding previously imported libraries: QArm, QBot, Widget, Spline, Utils, and BasicShape)
from library_qlabs_qbot_hopper import QLabsQBotHopper
from library_qlabs_srv02 import QLabsSRV02
from library_qlabs_bottle_table import QLabsBottleTableAttachment, QLabsBottleTableSupport, QLabsBottleTableSensorTowerShort, QLabsBottleTableSensorTowerTall
from library_qlabs_weigh_scale import QLabsWeighScale


class configure_environment:
    def __init__(self, project_identifier, ip_address, hardware = False, config_info = None):
        self.project_identifier = str(project_identifier)
        self.device_ip_address = str(ip_address)
        self.config_info = config_info
        
        self.QLabsHostname = self.device_ip_address

        self.QLabs = QuanserInteractiveLabs()
        
        print("Connecting to QLabs...")
        self.QLabs.open("tcpip://{}:18000".format(self.QLabsHostname))


        if hardware == False:
            print("Working in the simulated environment.")

            # destroy all spawned actors to reset the scene
            print("Deleting current spawned actors...")
            self.QLabs.destroyAllSpawnedWidgets()
            self.QLabs.destroyAllSpawnedActors()
        
            if self.project_identifier == 'P0':
                self.p0_environment()
                print("Environment Configured")
            elif self.project_identifier == 'P2A':
                self.p2a_environment()
                print("Environment Configured")
            elif self.project_identifier == 'P2B':
                self.p2b_environment()
                print("Environment Configured")
            elif self.project_identifier == 'P3A':
                self.p3a_environment()
                print("Environment Configured")
            elif self.project_identifier == 'P3B':
                self.p3b_environment()
                print("Environment Configured")
            else:
                print("Project Identifier entry is not valid. Enter P0, P2A, P2B, P3A, or P3B.")
            

        else:
            print("Working in the physical environment. No configuration required.")

    #NOT USED - Could be implemented next year
    def change_bin_properties(self,deviceNumberStart,color,roughness,metallic):
        for i in range(4):
            QLabsBasicShape().setMaterialProperties(self.QLabs, deviceNumberStart+i, color, roughness, metallic, waitForConfirmation)

    def p0_environment(self):
        self.QLabs.setTitleString("McMaster Project 0")

        print("Spawning the P0 environment...")
        camera_location = [1.038, -1.084, 1.143]
        camera_rotation = [0, 0.647, 2.489]

        QLabsFreeCamera().spawn(self.QLabs, 1, camera_location, camera_rotation)
        QLabsFreeCamera().possess(self.QLabs,1) # This is what makes this camera default

        Arena1x = 0
        Arena1y = 0
        wall_color = [0.4, 0.2, 0.0] # Orangish color

        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1000, centerLocation=[Arena1x, Arena1y, 0], yaw=0, xSize=1.219, ySize=1.829, zHeight=0.2, wallThickness=0.025, floorThickness=0.01, wallColor=wall_color, floorColor=[0.05, 0.05, 0.05])
        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1010, centerLocation=[Arena1x, Arena1y, 0], yaw=0, xSize=0.152, ySize=0.406, zHeight=0.2, wallThickness=0.025, floorThickness=0.01, wallColor=wall_color, floorColor=[0.05, 0.05, 0.05])

        # Spawns circle in the center
        line_color = [0.3, 0.3, 0]
        line_width = .686 # m
        line_height = 1.143 # m
        corner_radius = line_width/2.0
        
        QLabsUtils.spawnSplineRoundedRectangleFromCenter(self.QLabs, 1, centerLocation=[Arena1x, Arena1y, 0.015], rotation=[0, 0, 0], cornerRadius=corner_radius, xWidth=line_width, yLength=line_height, lineWidth=1.3, color=line_color, waitForConfirmation=True)

        bot_location_x = line_width/(2.0) # in meters
        QLabsQBot2e().spawnDegrees(self.QLabs, 0, [bot_location_x, 0, 0.0], [0, 0, 90])
        #QLabsQBot2e().possess(self.QLabs, 0, 2) # This makes the Qbot camera default.

        #print("Stopping any open RT models...")
        #self.QLabs.terminateRTModels() # NEED to terminate all models individually.
        print("Stopping any open RT models...")
        QLabsQBot2e().terminateRTModel()
        QLabsQArm().terminateRTModel()
        QLabsSRV02().terminateRTModel()

        print("Starting QBot realtime model...")
        QLabsQBot2e().startRTModel(0, self.QLabsHostname)

        time.sleep(1)
        self.QLabs.close()

    def p2a_environment(self):
        self.QLabs.setTitleString("McMaster Project 2A")

        print("Spawning the P2A environment...")

        QLabsFreeCamera().spawn(self.QLabs, 2, [-0.787, 0.964, 0.724], [-0, 0.372, -0.732])
        QLabsQArm().spawn(self.QLabs, 0, [0, 0, 0], [0, 0, 0])



        QLabsBasicShape().spawn(self.QLabs, 0, [0.44, 0.05, 0.125], [0, 0, 0], [0.25, 0.25, 0.25], QLabsBasicShape().SHAPE_CYLINDER)
        QLabsBasicShape().setPhysicsProperties(self.QLabs, 0, 1, 1, 0.1, True)
        QLabsBasicShape().spawn(self.QLabs, 1, [0.2, -0.43, 0.15], [0, 0, 0], [0.25, 0.25, 0.3], QLabsBasicShape().SHAPE_CYLINDER)
        QLabsBasicShape().setPhysicsProperties(self.QLabs, 0, 1, 1, 0.1, True)
        QLabsBasicShape().spawn(self.QLabs, 2, [-0.31, -0.4, 0.18], [0, 0, 0], [0.25, 0.25, 0.35], QLabsBasicShape().SHAPE_CYLINDER)
        QLabsBasicShape().setPhysicsProperties(self.QLabs, 0, 1, 1, 0.1, True)

        QLabsFreeCamera().possess(self.QLabs, 2)

        time.sleep(1)

        QLabsDeliveryTube().spawn(self.QLabs, 1, [0.35, .35, 0], [0, 0, 0], QLabsDeliveryTube().CONFIG_HOVER)
        for count in range(10,25, 1):
            QLabsDeliveryTube().setHeight(self.QLabs, 1, count/100)
            
        QLabsDeliveryTube().spawnBlock(self.QLabs, 1, QLabsDeliveryTube().BLOCK_CYLINDER, 1.0, 0.0, [0, 1, 0])

        QLabsDeliveryTube().spawn(self.QLabs, 0, [0, 0.66, 0], [0, 0, 0], QLabsDeliveryTube().CONFIG_HOVER)
        for count in range(10,25, 1):
            QLabsDeliveryTube().setHeight(self.QLabs, 0, count/100)
            
        QLabsDeliveryTube().spawnBlock(self.QLabs, 0, QLabsDeliveryTube().BLOCK_CUBE, 1.0, 0.0, [1, 0, 0])


        QLabsDeliveryTube().spawn(self.QLabs, 2, [-0.25, 0.5, 0], [0, 0, 0], QLabsDeliveryTube().CONFIG_HOVER)
        for count in range(10,40, 1):
            QLabsDeliveryTube().setHeight(self.QLabs, 2, count/100)

        QLabsDeliveryTube().spawnBlock(self.QLabs, 2, QLabsDeliveryTube().BLOCK_GEOSPHERE, 1.0, 0.0, [0, 0, 1])

        #print("Stopping any open RT models...")
        #self.QLabs.terminateRTModels() # NEED to terminate all models individually.
        print("Stopping any open RT models...")
        QLabsQBot2e().terminateRTModel()
        QLabsQArm().terminateRTModel()
        QLabsSRV02().terminateRTModel()

        print("Starting QArm realtime model...")
        QLabsQArm().startRTModel(0, self.QLabsHostname)

        time.sleep(1)

        #QLabs.destroy_all_spawned_widgets()

        self.QLabs.close()

    def p2b_environment(self):
        self.QLabs.setTitleString("McMaster Project 2B")

        print("Spawning the P2B environment...")
        #-0, 0.606, -0.712
        #[-0.787, 0.564, 1.0], [0, 0.666, -1.077])
       
        QLabsFreeCamera().spawn(self.QLabs, 3,[-0.506, 0.875, 1], [-0, 0.795, -1.045])
        QLabsFreeCamera().possess(self.QLabs, 3)
        
        QLabsQArm().spawn(self.QLabs, 0, [0, 0, 0], [0, 0, 0])

        # For later: Verify whether 0.52 is to the center of the box or to the front.
        QLabsAutoclave().spawnDegrees(self.QLabs, 0, [0, -0.52, 0], [0, 0, 180], QLabsAutoclave().RED)
        QLabsAutoclave().spawnDegrees(self.QLabs, 1, [0.52, 0, 0], [0, 0, -90], QLabsAutoclave().GREEN)
        QLabsAutoclave().spawnDegrees(self.QLabs, 2, [-0.52, 0, 0], [0, 0, 90], QLabsAutoclave().BLUE)

        #print("Stopping any open RT models...")
        #self.QLabs.terminateRTModels() # NEED to terminate all models individually.
        print("Stopping any open RT models...")
        QLabsQBot2e().terminateRTModel()
        QLabsQArm().terminateRTModel()
        QLabsSRV02().terminateRTModel()

        print("Starting QArm realtime model...")
        QLabsQArm().startRTModel(0, self.QLabsHostname)

        time.sleep(1)
        
        self.QLabs.close()
    
    def p3a_environment(self):
        self.QLabs.setTitleString("McMaster Project 3A")
        print("Spawning the P3A environment...")
        QLabsFreeCamera().spawn(self.QLabs, 4,[2.5,-1.270,1.05],[-0, 1.002, 2.702])
        QLabsFreeCamera().possess(self.QLabs, 4)

        self.p3_environment()
    
    def p3b_environment(self):
        self.QLabs.setTitleString("McMaster Project 3B")
        print("Spawning the P3B environment...")
        QLabsFreeCamera().spawn(self.QLabs, 5, [3.086, -1.27, 1.1], [0.0, 0.643, 2.617])
        QLabsFreeCamera().possess(self.QLabs, 5)

        self.p3_environment()

    # DO NOT INCLUDE IN DOCUMENTATION - USED TO SIMPLIFY THE TWO ENVIRONMENT CALLS ABOVE.
    def p3_environment(self):
        if self.config_info[2] == None: # Could be put in their corresponding environment. Consider doing later.
            print("Using default configuration information")
            bin1_offset,bin2_offset,bin3_offset,bin4_offset = 0.16,0.16,0.16,0.16
            bin1_color,bin2_color,bin3_color,bin4_color = [1.0, 0, 0],[0.0, 1.0, 0.0],[0, 1.0, 1.0],[0.1, 0.1, 0.1] #r,g,b,black
            [short_tower_angle,tall_tower_angle,drop_tube_angle] = self.config_info[0] # in degrees. Positive angles are clockwise
            
        else:
            print("Using custom configuration information")
            [bin1_offset,bin2_offset,bin3_offset,bin4_offset] = self.config_info[2][0]
            [bin1_color,bin2_color,bin3_color,bin4_color] = self.config_info[2][1]
            [short_tower_angle,tall_tower_angle,drop_tube_angle] = self.config_info[0]
            
        #self.QLabs.setTitleString("McMaster Project 3")
  
        print("Spawning the environment...")



        tableCenterX = 2.1
        tableCenterY = -0.80
        
        QLabsQArm().spawnDegrees(self.QLabs, 0, [tableCenterX, 0, 0], [0, 0, 180])
        wall_color = [1, 1, 1] # White color
        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1000, centerLocation=[tableCenterX,0,0], yaw=0, xSize=0.65, ySize=2.5, zHeight=0.2, wallThickness=0.05, floorThickness=0.0, wallColor=wall_color)
        
        QLabsWeighScale().spawn(self.QLabs, 0, [tableCenterX, -tableCenterY, 0], [0, 0, 0], False)
        
        QLabsSRV02().spawn(self.QLabs, 0, [tableCenterX, tableCenterY, 0], [0, 0, 0], False)
        QLabsBottleTableSupport().spawnAndParentWithRelativeTransform(self.QLabs, 0, [0,0, 0], [0, 0, 0], QLabsSRV02().ID_SRV02, 0, 0, False)

        QLabsBottleTableAttachment().spawnAndParentWithRelativeTransform(self.QLabs, 0, [0,0, 0], [0, 0, 0], QLabsSRV02().ID_SRV02, 0, 1, False)
        QLabsBottleTableSensorTowerShort().spawnAndParentWithRelativeTransformDegrees(self.QLabs, 0, [0,0,0], [0,0,90-short_tower_angle], QLabsSRV02().ID_SRV02, 0, 0, waitForConfirmation=True)
        QLabsBottleTableSensorTowerTall().spawnAndParentWithRelativeTransformDegrees(self.QLabs, 0, [0,0,0], [0,0,90-tall_tower_angle], QLabsSRV02().ID_SRV02, 0, 0, waitForConfirmation=True)

        QLabsDeliveryTube().spawn(self.QLabs, 5, [tableCenterX+0.182*math.sin(math.radians(drop_tube_angle+180)), tableCenterY+0.182*math.cos(math.radians(drop_tube_angle+180)), 3.4], [math.pi, 0, 0], QLabsDeliveryTube().CONFIG_HOVER)
        QLabsDeliveryTube().setHeight(self.QLabs, 5, 2.98) #changed from 3.0 to avoid clipping
        
        line_thickness = 1.3
        line_width = 1.524 # m (5')
        line_height = 2.1336 # m (7')
        corner_radius = line_width/4.0
        QLabsUtils.spawnSplineRoundedRectangleFromCenter(self.QLabs, 1, centerLocation=[.4332,0,0], rotation=[0,0,0], cornerRadius=corner_radius, xWidth=line_height, yLength=line_width, lineWidth=line_thickness, color=[0.3, 0.3, 0], waitForConfirmation=True)
        
        lineWidth = 1.3
        bin_size = 0.3
        bin_height = 0.2
        bin_center_y = 0.8

        
        bin1_line_coordinates = [[1.0556, .76, 0,lineWidth],[1.0556, bin_center_y+bin1_offset,0,lineWidth]]
        bin1_line_color = bin1_color
        bin1_center = [1.0556, bin_center_y+bin1_offset+(bin_size/2),0]

        bin2_line_coordinates = [[0.0092, .76, 0,lineWidth],[.0092, bin_center_y+bin2_offset,0,lineWidth]]
        bin2_line_color = bin2_color
        bin2_center = [0.0092, bin_center_y+bin2_offset+(bin_size/2),0]

        bin3_line_coordinates = [[0.0092, -.76, 0,lineWidth],[0.0092, -bin_center_y-bin3_offset,0,lineWidth]]
        bin3_line_color = bin3_color
        bin3_center = [0.0092, -bin_center_y-bin3_offset-(bin_size/2),0]
        
        bin4_line_coordinates = [[1.0556, -.76, 0,lineWidth],[1.0556, -bin_center_y-bin4_offset,0,lineWidth]]
        bin4_line_color = bin4_color
        bin4_center = [1.0556, -bin_center_y-bin4_offset-(bin_size/2),0]

        QLabsSplineLine().spawn(self.QLabs, 5, [0,0,0], [0,0,0], [1, 1, 1], 0, True)
        QLabsSplineLine().setPoints(self.QLabs, 5, bin1_line_color, alignEndPointTangents=False, pointList=bin1_line_coordinates)
        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1400, centerLocation=bin1_center, yaw=0, xSize=bin_size, ySize=bin_size, zHeight=bin_height, wallThickness=0.025, floorThickness=0.025, wallColor=bin1_color, floorColor=bin1_color)

        QLabsSplineLine().spawn(self.QLabs, 4, [0,0,0], [0,0,0], [1, 1, 1], 0, True)
        QLabsSplineLine().setPoints(self.QLabs, 4, bin2_line_color, alignEndPointTangents=False, pointList=bin2_line_coordinates)
        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1300, centerLocation=bin2_center, yaw=0, xSize=bin_size, ySize=bin_size, zHeight=bin_height, wallThickness=0.025, floorThickness=0.025, wallColor=bin2_color, floorColor=bin2_color)

        QLabsSplineLine().spawn(self.QLabs, 3, [0,0,0], [0,0,0], [1, 1, 1], 0, True)
        QLabsSplineLine().setPoints(self.QLabs, 3, bin3_line_color, alignEndPointTangents=False, pointList=bin3_line_coordinates)
        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1200, centerLocation=bin3_center, yaw=0, xSize=bin_size, ySize=bin_size, zHeight=bin_height, wallThickness=0.025, floorThickness=0.025, wallColor=bin3_color, floorColor=bin3_color)

        QLabsSplineLine().spawn(self.QLabs, 10, [0,0,0], [0,0,0], [1, 1, 1], 0, True)
        QLabsSplineLine().setPoints(self.QLabs, 10, bin4_line_color, alignEndPointTangents=False, pointList=bin4_line_coordinates)
        QLabsUtils.spawnBoxWallsFromCenterDegrees(self.QLabs, deviceNumberStart=1100, centerLocation=bin4_center, yaw=0, xSize=bin_size, ySize=bin_size, zHeight=bin_height, wallThickness=0.025, floorThickness=0.025, wallColor=bin4_color, floorColor=bin4_color)
          
        QLabsQBot2e().spawnDegrees(self.QLabs, 0, [1.5, 0, 0], [0, 0, 90])
        QLabsQBotHopper().spawnAndParentWithRelativeTransform(self.QLabs, 0, [0, 0, 0], [0, 0, 0], QLabsQBot2e().ID_QBOT, 0, 0, True)

        print("Stopping any open RT models...")
        QLabsQBot2e().terminateRTModel()
        QLabsQArm().terminateRTModel()
        QLabsSRV02().terminateRTModel()
        
        print("Starting QBot2e realtime model...")
        QLabsQBot2e().startRTModel(0, self.QLabsHostname)

        print("Starting QArm realtime model...")
        QLabsQArm().startRTModel(0, self.QLabsHostname)  

        print("Starting Turntable realtime model...")
        QLabsSRV02().startRTModel(0, self.QLabsHostname) 
        #self.QLabs.close()

    ##### QBOT STARTING FREE CAMERA COORDINATES ([3.086, 1.07, 0.958], [-0.002, -0.324, -2.561])

# Container properties
empty_plastic_container = 9.25 # empty mass of plastic container in g
empty_metal_container = 15.0 # empty mass of metal container in g
empty_paper_container = 10.0 # empty mass of paper container in g
interval = 0.2
class servo_table:
    def __init__(self,ip_address,QLabs,table_configuration,hardware = False):
        self.QLabsHostname = str(ip_address)
        self.hardware = hardware
        self.my_table = RotaryTable(0,'localhost',self.QLabsHostname,self.hardware)
        self.QLabs = QLabs
        self.table_weight = 0
        self.proximity_short = False
        self.proximity_tall = False
        self.drop_tube_angle = table_configuration[2]
        
# -----------------------------------------------------------------------------------------------
# Table Control
    def rotate_table_speed(self, speed):
        if float(speed) >= 0.0 and float(speed) <= 1.0:
            voltage = 2.76704*speed+0.1021 #Changed from 6.6958
            self.my_table.rotate_clockwise(voltage)
        elif float(speed) > 1.0:
            print("Input speed is too fast. Enter a speed less than 1 m/s")
        elif float(speed) < 0.0:
            print("Input a positive speed.")
        else:
            print("Invalid input.")

    def rotate_table_angle(self, deg):
        if deg < 0:
            print("Input a positive angle.")
        else:
            Kenc = 360/4096
            current  = self.my_table.read_encoder()*Kenc
            target = current+deg
            self.my_table.command_abs_position_pid(target)

    def stop_table(self):
        self.my_table.stop_table()

# -----------------------------------------------------------------------------------------------
# Spawning Containers

    # Dispense bottle based on a random number between 0 and 5 inclusive. Where:
    # 1 = non contaminated plastic, 2 = non-contaminated metal, 3 = non-contaminated paper,
    # 4 = contaminated plastic, 5 = contaminated metal, 6 = contaminated paper
    def dispense_container(self,num,properties=False):
        tableCenterX = 2.1
        tableCenterY = -0.80
        red = [1,0,0]
        blue = [0,0,1]
        clear = [1,1,1]
        #4 for bottles 5 for cans
        obj1 = [4, clear, 'plastic', empty_plastic_container, "Bin03"]
        obj2 = [5, red, 'metal', empty_metal_container, "Bin01"]
        obj3 = [4, blue, 'paper', empty_paper_container, "Bin02"]

        # contaminated containers - a list containing the color of the object and the mass in grams
        obj4 = [4, clear, 'plastic', empty_plastic_container + random.uniform(5.0, 50.0), "Bin04"]
        obj5 = [5, red, 'metal', empty_metal_container + random.uniform(5.0, 50.0), "Bin01"]
        obj6 = [4, blue, 'paper', empty_paper_container + random.uniform(5.0, 50.0), "Bin04"]
        #Default -> [tableCenterX + 0.0,   tableCenterY + 0.182, 0.16]
        location = [tableCenterX+0.182*math.sin(math.radians(self.drop_tube_angle+180)), tableCenterY+0.182*math.cos(math.radians(self.drop_tube_angle+180)), 0.3]
        
        containers = [obj1, obj2, obj3, obj4, obj5, obj6]
        size, color,material,mass,dist = containers[num-1]    
        QLabsWidget().spawn(self.QLabs, size, location, [0, 0, 0], [0.6, 0.6, 0.6], color,   mass, IDTag=0, properties=material, waitForConfirmation=False)
        if properties:
            return material,mass,dist

# -----------------------------------------------------------------------------------------------
# Table Sensors

    # returns the position of the bottle from the short tower                                     
    def proximity_sensor_short(self):
        distance, material = QLabsBottleTableSensorTowerShort().GetProximity(self.QLabs, 0)
        if distance[0] != 0 or distance[1] != 0 or distance[2] != 0:
            self.proximity_short = True
        else:
            self.proximity_short = False
        return self.proximity_short

    # returns the position of the bottle from the tall tower                                     
    def proximity_sensor_tall(self):
        distance = QLabsBottleTableSensorTowerTall().GetTOF(self.QLabs, 0)
        if distance <= 14:
            self.proximity_tall = True
        else:
            self.proximity_tall = False
        return self.proximity_tall

    # A single mass reading of the entire table is appended to a list at given interval steps (0.2) for the entire duration
    def load_cell_sensor(self, duration):
        load_cell_mass = round(QLabsBottleTableAttachment().getMeasuredMass(self.QLabs,0),2)
        mass = []    
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < duration:
            if load_cell_mass > 0.0:
                mass.append(load_cell_mass + random.uniform(0.0, 0.4))
            else:
                mass.append(load_cell_mass)
            time.sleep(interval)
            elapsed_time = time.time() - start_time
        
        return mass
    
    # returns the height of the gap between the top of the tall tower and the top of the container    
    def tof_sensor(self):
        distance = QLabsBottleTableSensorTowerTall().GetTOF(self.QLabs, 0)
        return distance

    # Gives a high reading if an object is within it's proximity
    def capacitive_sensor(self):
        if self.proximity_sensor_short():
            return random.uniform(4.5, 5.0)
        else:
            return random.uniform(0, 0.4)
        
    # Gives a high reading metal is detected.
    # To randomize the values, a duration is input and readings are appended to a table for a given intervals (0.2)
    # during that specified time.
    def inductive_sensor(self, duration):
        distance, material = QLabsBottleTableSensorTowerTall().GetProximity(self.QLabs, 0)
        reading = []
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < duration:
            if material=="metal":
                reading.append(random.uniform(4.5, 5.0))
            else:
                reading.append(random.uniform(0, 0.4))
            time.sleep(interval)
            elapsed_time = time.time() - start_time        
        return reading

    # Dark On Retro-reflective photoelectric sensor. It read a high value when it senses a target and a low value
    # when the light comes back to the receiver (e.g no target / clear target)
    # To randomize the values, a duration is input and readings are appended to a table for a given intervals (0.2)
    # during that specified time.
    def photoelectric_sensor(self, duration):
        distance, material = QLabsBottleTableSensorTowerTall().GetProximity(self.QLabs, 0)
        reading = []
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < duration:
            if material=="metal" or material=="paper":
                reading.append(random.uniform(4.5, 5.0))
            else:
                reading.append(random.uniform(0, 0.4))
            time.sleep(interval)
            elapsed_time = time.time() - start_time        
        return reading
    
class qarm:

    def __init__(self, project_identifier, ip_address, QLabs, hardware = False):
        self.device_ip_address = str(ip_address) # Not used for anything (keeping it consistent with everything else)
        self.hardware = hardware
        self.my_qarm = QArm(0, 'localhost', hardware)
        self.QLabs = QLabs # May help with spawing items - UPDATED, Must edit original object -Basem
        self.my_qarm.set_base_color([0, 0, 1])
        self.project_identifier = project_identifier
        # self.tolerance = 0.01
        #
        # self.cage_red_small = [1, 0.5, "Small red cage"]
        # self.cage_green_small = [2, 0.5, "Small green cage"]
        # self.cage_blue_small = [3, 0.5, "Small blue cage"]
        # self.cage_red_large = [4, 1, "Large red cage"]
        # self.cage_green_large = [5, 1, "Large green cage"]
        # self.cage_blue_large = [6, 1, "Large blue cage"]
        #
        # self.my_cage = genericSpawn_sim(QIL)
        #
        # self.red_autoclave = autoclave_sim(QIL, 0)
        # self.green_autoclave = autoclave_sim(QIL, 1)
        # self.blue_autoclave = autoclave_sim(QIL, 2)
        #
        # self.my_emg = EMG_sim(QIL)
        #
        self.b, self.s, self.e, self.w, self.g = 0, 0, 0, 0, 0
        self.home() # Sets the arm to the home position

    def effector_position(self):
        x_pos, y_pos, z_pos = self.my_qarm.qarm_forward_kinematics(self.b, self.s, self.e, self.w)
        return round(x_pos,3), round(y_pos,3), round(z_pos,3)
    
    def home(self):
        if self.hardware==False:
            self.my_qarm.return_home()
        else:
            self.move_arm_intermediate()
            self.rotate_wrist(math.degrees(-self.w))
            self.control_gripper(math.degrees(-self.g))
            # self.my_qarm.return_home()                                                                                
        self.b, self.s, self.e, self.w, self.g = 0, 0, 0, 0, 0
        time.sleep(0.1)

    # # enter a value between 1 and 6 (inclusive). 1,2,3 for small red, green and blue containers respectively,
    # # and 4,5,6 for large red, green and blue containers respectively
    # def spawn_cage(self, value):
    #     if value == 1:
    #         self.my_cage.spawn_with_properties(self.cage_red_small[0], self.cage_red_small[1],
    #                                            self.cage_red_small[2])
    #     elif value == 2:
    #         self.my_cage.spawn_with_properties(self.cage_green_small[0], self.cage_green_small[1],
    #                                            self.cage_green_small[2])
    #     elif value == 3:
    #         self.my_cage.spawn_with_properties(self.cage_blue_small[0], self.cage_blue_small[1],
    #                                            self.cage_blue_small[2])
    #     elif value == 4:
    #         self.my_cage.spawn_with_properties(self.cage_red_large[0], self.cage_red_large[1], self.cage_red_large[2])
    #     elif value == 5:
    #         self.my_cage.spawn_with_properties(self.cage_green_large[0], self.cage_green_large[1],
    #                                            self.cage_green_large[2])
    #     elif value == 6:
    #         self.my_cage.spawn_with_properties(self.cage_blue_large[0], self.cage_blue_large[1],
    #                                            self.cage_blue_large[2])
    #     else:
    #         print("Please enter a value between 1 and 6 (inclusive)")
    #     time.sleep(0.1)
    #     return value
    #
    
    # # Rotate Joints
    def rotate_base(self, deg):
        b = self.b + math.radians(deg)
        if abs(b) > math.radians(175):
            print("Invalid Angle. Base does not rotate beyond +/- 175 degrees")
        else:
            self.b = b
            self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g)
    
    def rotate_shoulder(self, deg):
        s = self.s + math.radians(deg)
        if abs(s) > math.radians(90):
            print("Invalid Angle. Shoulder does not rotate beyond +/- 90 degrees")
        else:
            self.s = s
            self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g)
    
    def rotate_elbow(self, deg):
        e = self.e + math.radians(deg)
        if e > math.radians(90) or e < math.radians(-80):
            print("Invalid Angle. Elbow does not rotate beyond +90 or -80 degrees")
        else:
            self.e = e
            self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g)
    
    def rotate_wrist(self, deg):
        w = self.w - math.radians(deg) # subtracting to make sure it rotates counter-clockwise as per documentation.
        if abs(w) > math.radians(170):
            print("Invalid Angle. Wrist does not rotate beyond +/- 170 degrees")
        else:
            self.w = w
            self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g)
    
    # Control Gripper. Gripper moves between 0 - 45 degrees
    def control_gripper(self, value):
        if abs(value) <= 45 and math.degrees(self.g + math.radians(value)) >= 0 and math.degrees(self.g + math.radians(value)) < 46:
            self.g = self.g + math.radians(value)
            self.my_qarm.qarm_move_gripper(self.g, wait = False)
        else:
            print("Please enter a value in between +/- 45 degrees.")
###############################################################################################
# Currently doesn't work with the new updates. will need to look at this section in the future

    # Open / Close the Autoclave. Takes values of True = open, False = close
    def open_red_autoclave(self, value):
        if self.project_identifier == "P2B":
            QLabsAutoclave().setDrawer(self.QLabs, 0, value, waitForConfirmation=True)
        else:
            print("Autoclave feature not available in the current environment.")
        #self.red_autoclave.open_drawer(value)
    
    def open_green_autoclave(self, value):
        if self.project_identifier == "P2B":
            QLabsAutoclave().setDrawer(self.QLabs, 1, value, waitForConfirmation=True)
        else:
            print("Autoclave feature not available in the current environment.")
        #self.green_autoclave.open_drawer(value)
    
    def open_blue_autoclave(self, value):
        if self.project_identifier == "P2B":
            QLabsAutoclave().setDrawer(self.QLabs, 2, value, waitForConfirmation=True)
        else:
            print("Autoclave feature not available in the current environment.")
        #self.blue_autoclave.open_drawer(value)
    
    # EMG Sensor Readings
    # def emg_left(self):
    #     emg_left, emg_right = self.my_emg.read_all_sensors()
    #     return round(emg_left,3)
    #
    # def emg_right(self):
    #     emg_left, emg_right = self.my_emg.read_all_sensors()
    #     return round(emg_right,3)

###############################################################################################

    # Move arm to target location based on cartesian coordinate inputs
    def move_arm(self, x, y, z):
        if self.hardware==False:
            self.b, self.s, self.e = [math.radians(ang) for ang in self.my_qarm.qarm_inverse_kinematics(x, y, z)] # FIXED inverse kinematics outputted degrees instead of radians
            self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g, wait = False)
        else:
            # WILL NEED TO VERIFY. MOVEMENT IS WEIRD BUT HAD WORKED FOR PROJECT 2 BEFORE
            self.move_arm_intermediate() # first move to home position (the intermediate step)

            base, shoulder, elbow = [math.radians(ang) for ang in self.my_qarm.qarm_inverse_kinematics(x, y, z)]
            # print(math.degrees(base))
            # print(math.degrees(shoulder))
            # print(math.degrees(elbow))

            self.rotate_base(math.degrees(base - self.b))
            #time.sleep(2)
            self.rotate_elbow(math.degrees(elbow - self.e))
            #time.sleep(2)
            self.rotate_shoulder(math.degrees(shoulder - self.s))
            #print("base, elbow, shoulder")
            self.b = base
            self.s = shoulder
            self.e = elbow
            # self.b, self.s, self.e = self.my_qarm.qarm_inverse_kinematics(x, y, z)
            # self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g, wait = False)

#########################################################
# Do not include in code documentation for students.
#########################################################
    def move_arm_intermediate(self):
        base, shoulder, elbow = [math.radians(ang) for ang in self.my_qarm.qarm_inverse_kinematics(0.406, 0, 0.483)] # First move home without affecting the wrist or gripper
        # print(math.degrees(base))
        # print(math.degrees(shoulder))
        # print(math.degrees(elbow))
        if math.degrees(shoulder - self.s) >= 10:
            self.rotate_elbow(math.degrees(elbow) - math.degrees(self.e))
            #time.sleep(2)
            self.rotate_shoulder(math.degrees(shoulder) - math.degrees(self.s))
            #time.sleep(2)
            self.rotate_base(math.degrees(base) - math.degrees(self.b))
            #print("elbow, shoulder, base")
        else:
            self.rotate_shoulder(math.degrees(shoulder - self.s))
            #time.sleep(2)
            self.rotate_base(math.degrees(base - self.b))
            #time.sleep(2)
            self.rotate_elbow(math.degrees(elbow - self.e))
            #print("shoulder, base, elbow")
        time.sleep(0.5)
        self.b = base
        self.s = shoulder
        self.e = elbow

wheel_to_wheel_distance = 0.235
camera_bumper_depth = 0.156

class qbot:
    def __init__(self, speed, ip_address,QLabs,bins = None,hardware = False):
        QLabsHostname = str(ip_address)
        self.bot = QBot2e(0,'localhost',QLabsHostname,hardware)
        self.QLabs = QLabs
        self.max_speed = 100
        self.speed = speed
        self.turn = 0
        self.hardware = hardware

        # Initialize kinect and camera for line following
        # Ensure that the default image file is in the same location of both the library and the file that will be run.
        self.kinect = Kinect()
        self.camera_image = CameraUI()
        self.img_RGB = np.zeros((480,640,3), dtype = np.uint8) #initial image
        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])

        # activated actuator
        self.stepper_motor_activated = False
        self.linear_actuator_activated = False

        # activated Sensors
        self.ultrasonic_sensor_activated = False
        #self.hall_sensor_activated = False #Deprecated for 2022 Winter
        self.ir_sensor_activated = False
        self.ldr_sensor_activated = False
        self.color_sensor_activated = False

        # Sensor ranges.
        self.ultrasonic_sensor_range = 2.5 # Adafruit HC-SR04, source = datasheet. 250 cm
        #self.hall_sensor_range = 0.25 # 25 cm
        self.ir_sensor_range = 0.25 # 25 cm
        self.ldr_sensor_range = 0.25 # 25 cm
        self.color_sensor_range = 0.25 # 25 cm

        # initialize variable for line following capabilities
        self.lost_line = 0

        # bins object
        self.bin = bins

        if self.hardware:
            self.GPIO = __import__('RPi.GPIO')
            self.GPIO_TRIGGER = 23
            self.GPIO_ECHO = 24 
            self.MCP3008 = __import__('Adafruit_MCP3008').MCP3008
            #Pins
            #set GPIO Pins
            self.GPIO_TRIGGER = 23
            self.GPIO_ECHO = 24
            self.GPIO_LDR = 12
            self.GPIO_S2 = 13
            self.GPIO_S3 = 16
            self.GPIO_OUT = 19
            self.GPIO_left = 18
            self.GPIO_right = 27
            #ADC Pins
            self.CLK  = 21
            self.MISO = 9
            self.MOSI = 10
            self.CS   = 22
            #Actuators
            self.ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
            self.ser.flush()
            self.linear_actuator_time_counter = 0 # used to keep track of time since the linear actuator doesn't have an encoder
            self.linear_actuator_max_time = 5 # (50mm/10mm/s) Assume that the actuator moves at 10mm/s (0.4 times full speed)

    # Moves the bot for the specified time in seconds.
    def forward_time(self, time):
        #Convert speed to distance and command QBot
        distance = self.speed*time
        self.bot.move_time(distance=distance,move_time=time)
        self.depth()

    # Moves the bot for the specified distance in meters.
    def forward_distance(self,distance):
        # Convert speed to time and command QBot
        time = distance/self.speed
        self.bot.move_time(distance=distance,move_time=time)
        self.depth()

    # Moves the bot until the specified distance in meters from an object is reached i.e. the threshold.
    # Distance is measured from the bot's bumper to the object within vicinity.
    # Recommend to increase wall height to 22 cm.
    def travel_forward(self,threshold):
        d = round(self.depth(),3) # Initial depth measurement in meters. From bumper to measured object
        if d >= 0.125 and threshold >= 0.125:        
            # Continue to drive until threshold is reached
            while threshold < d:
                # Drive 
                self.bot.set_velocity([self.speed,self.speed])

                print ("Depth (m): ", d)
                d = round(self.depth(),3)
                #time.sleep(0.05)

            # Stop QBot
            self.bot.halt()
            print("Depth (m): ",d)
        else:
            self.bot.halt() # unnecessary, but just in case.
            print("Threshold will cause the bot to crash into the object.")        

    # Sets the speed of the left and right wheel. inputer is a list containing the left wheel speed at index 0 and right wheel speed at index 1.
    def set_wheel_speed(self,speeds):
        speeds = speeds[::-1] #Wheel speeds are reversed on the backend
        if type(speeds) == list:
            self.bot.set_velocity(speeds)
        else:
            print("Invalid input. Enter the left and right wheel speed as a list. e.g. [left_speed, right_speed].")

    # Rotates the bot by the specified degrees clockwise.
    def rotate(self, degree):
        time = 1.0
        self.bot.move_time(angle=math.radians(degree),move_time=time)
        self.depth()

    # Read and return how far the Q-Bot's bumper is from an object in meters e.g. the walls.
    # Suggest to start with camera at 0 degrees and decrease if a non-sensical reading is output
    def depth (self):
        row = 385
        col = 319
        # Get last depth frame
        depth_frame = self.kinect.get_depth_frame()
        
        # Extract central point; frame size 640 by 480; pixel values 0-255 (0~9.44 m)
        d = depth_frame[row][col][1]
        
        # Convert to m and return value
        d_meters = 9.44*d/255 - camera_bumper_depth
        
        return d_meters
    
    def initialize_camera(self):
        self.camera_image = CameraUI()

        self.img_RGB = np.zeros((480,640,3), dtype = np.uint8) #initial image
        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])
        #self.show_camera_image()

    def stop_camera(self):
        self.camera_image.destroy()

    def show_camera_image(self):
         cv2.startWindowThread()
         cv2.namedWindow('Camera Image 33 x 32', cv2.WINDOW_AUTOSIZE)
         cv2.imshow('Camera Image 33 x 32', self.img_binary)

    # Returns the values of the left and right IR sensors.
    # Must angle the camera to -21.5 degrees
    def line_following_sensors(self):
        if self.hardware:
            
            mcp = self.MCP3008(clk=self.CLK, cs=self.CS, miso=self.MISO, mosi=self.MOSI)
            left_ir_analog = mcp.read_adc(4)
            right_ir_analog = mcp.read_adc(3)
            ir_reading = [left_ir_analog, right_ir_analog]
            return ir_reading
        else:
            image_buffer = self.kinect.get_RGB_frame()
            line_ctr = self.camera_image.process(image_buffer)
            max_speed = 1
            qbot_speed = 0
            turn = 0

            if line_ctr != -1:
                self.lost_line = 0

                # Normalize the position of the line in the range (-1, 1)
                err = (320 - line_ctr) / 320 # Takes the curve path around the track

                # Calculate the offset for turning
                turn = err * 0.5
                # Slow down as the line approaches the edge of the FOV
                qbot_speed = max_speed * (1 - abs(err))

            else:
                # Stop the robot if the line is not found for 5 frames
                self.lost_line += 1
                if self.lost_line > 5 and max_speed != 0 :
                    self.stop()
                    print("Cannot find line, QBot stopped...")

            delta = turn * 0.235  # QBOT_DIAMETER
            left = qbot_speed - delta
            right = qbot_speed + delta
            
            if abs(left-right)<=0.02 and left!=0 and right!=0:
                ir_reading = [1,1]
            elif left>right:
                ir_reading = [0,1]
            elif right>left:
                ir_reading = [1,0]
            else:
                ir_reading = [0,0]
            
            return ir_reading
    
    def stop(self):
        self.bot.halt()
        self.depth()
        
    def position(self):
        return QLabsQBot2e().requestGlobalPosition(self.QLabs,0)

# -----------------------------------------------------------------------------------------------
# Used for internal calculations to reset the box's position and rotation.
# Do not include in the library documentation
    def reset_box(self):
        QLabsQBotHopper().commandDegrees(self.QLabs, 0, 0)

# -----------------------------------------------------------------------------------------------
# Actuators
    
    # Takes an input file from the modelling sub-team that contains time and angle data.
    # Recommended file type is a .txt file without headers i.e. string characters identifying
    # the time and angle columns. It is assumed that the first column is time (s) and the
    # second column is the angle (deg)
    def process_file(self,filename):
        rotation_time=[]
        rotation=[]
        with open(filename,"r") as f:
            for line in f:
                line = line.strip()
                line_pair = line.split("\t") #assuming translation and rotation coordinates are spearated by \t

                rotation_time.append(float(line_pair[0]))
                rotation.append(float(line_pair[1]))
        return rotation_time,rotation


    # Activates the stepper motor
    def activate_stepper_motor(self):
        self.stepper_motor_activated = True
        self.reset_box()
        print("Actuator activated.")

    # Deactivates the stepper motor
    def deactivate_stepper_motor(self):
        self.stepper_motor_activated = False
        self.reset_box()
        print("Actuator deactivated.")

    # Activates the linear actuator
    def activate_linear_actuator(self):
        self.linear_actuator_activated = True
        self.reset_box()
        print("Actuator activated.")

    # Deactivates the linear actuator
    def deactivate_linear_actuator(self):
        self.linear_actuator_activated = False
        self.reset_box()
        if self.hardware:
            self.linear_actuator_in(self.linear_actuator_time_counter) #Resets linear actuator
        print("Actuator deactivated.")

    # Rotates the container box angle to a specified angle position (Enter positive angles only)
    def rotate_hopper(self, angle):
        if self.hardware:
            print("Please use hardware acctuator methods.")
            return
        if self.linear_actuator_activated is not self.stepper_motor_activated:
            if angle >= 0.0 and angle <= 90:
                QLabsQBotHopper().commandDegrees(self.QLabs, 0, angle)
            elif angle < 0:
                print("Enter a positive angle.")
            elif angle > 90:
                print("Angle is greater than 90 degrees")
            else:
                print("Invalid angle.")
        elif self.linear_actuator_activated is self.stepper_motor_activated:
            print("Both actuators are activated. Deactivate one.")
        else:
            print("Actuator is not activated.")

    # Dumps the containers along a generic pre-defined motion.
    def dump(self):
        if self.hardware:
            print("Please use hardware acctuator methods.")
            return
        if self.linear_actuator_activated is not self.stepper_motor_activated:
            for i in range(100):
                j = (i/100)*math.tau
                theta = 1-math.cos(j)
                QLabsQBotHopper().commandDegrees(self.QLabs, 0, math.degrees(theta))
        else:
            print("Actuator is not activated.")

    def linear_actuator_out(self, time_extend):
        if not self.hardware:
            print("Please use simulation acctuator methods.")
            return
        if self.linear_actuator_activated:
            if (self.linear_actuator_time_counter + time_extend) <= self.linear_actuator_max_time:
                self.ser.write(b"extend\n")
                time.sleep(time_extend)
                self.ser.write(b"stop\n")
                self.linear_actuator_time_counter += time_extend
            else:
                print("Time exceeds linear actuator limit")
        else:
            print("Linear actuator not activated")

    def linear_actuator_in(self, time_retract):
        if not self.hardware:
            print("Please use simulation acctuator methods.")
            return
        if self.linear_actuator_activated:
            if (self.linear_actuator_time_counter - time_retract) >= 0:
                self.ser.write(b"retract\n")
                time.sleep(time_retract)
                self.ser.write(b"stop\n")
                self.linear_actuator_time_counter -= time_retract            
            else:
                print("Time exceeds linear actuator limit")
        else:
            print("Linear actuator not activated")

    def rotate_stepper_cw(self, time_rotate):
        if not self.hardware:
            print("Please use simulation acctuator methods.")
            return
        if self.stepper_motor_activated:
            steps = time_rotate*100 #time * 180deg/s / 1.8deg/step 
            self.ser.write(b"forwards\n")
            time.sleep(1)
            self.ser.write( str( steps ).encode( 'ascii' ) + b'\n' )
        else:
            print("Stepper motor not activated")

    def rotate_stepper_ccw(self, time_rotate):
        if not self.hardware:
            print("Please use simulation acctuator methods.")
            return
        if self.stepper_motor_activated:    
            steps = time_rotate*100 #time * 180deg/s / 1.8deg/step 
            self.ser.write(b"backwards\n")
            time.sleep(1)
            self.ser.write( str( steps ).encode( 'ascii' ) + b'\n' )
        else:
            print("Stepper motor not activated")

# ------------------------------------------------------------------------------------------------------------------------
# DON'T INCLUDE IN THE LIBRARY DESCRIPTION.

    # Used to calculate the length, which is used to evaluate the distance
    def dotproduct(self, v1, v2):
        return sum((a * b) for a, b in zip(v1, v2))

    # Same as above
    def length(self,v):
        return math.sqrt(self.dotproduct(v, v))

    # Used to randomize the readings by adding simulated noise.i.e. Readings are taken at time interaval (0.2) for a specified duration.
    # Students will need to calculate the average reading.
    def sensor_readings(self,duration, lower_limit, upper_limit):
        reading = []
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < duration:
            reading.append(random.uniform(lower_limit, upper_limit))
            time.sleep(interval)
            elapsed_time = time.time() - start_time
        return reading

    # Used in the ultrasonic sensor method to calculate the distance; and in the hall, and IR sensor for range verification.
    def box_to_bin_distance(self,bin_id):
        [self.bot_position_x, self.bot_position_y,self.bot_position_z] = self.position()  # Get intial bot reading. Always needs to be run or you would get a 0,0,0 reading.

        [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.bin.bin_position(bin_id)
        [self.bot_position_x, self.bot_position_y, self.bot_position_z] = self.position()

        qbot_radius = 0.5 * wheel_to_wheel_distance
        bin_length = 0.3 
        offset = 0.07 # The position of the bins from the centre of the yellow line is not exactly 0. It is 2 cm (1/2 thickness of the line) + 5 cm offset.

        distance = self.length([(self.bot_position_x - self.bin_position_x),(self.bot_position_y - self.bin_position_y )]) - 0.5*bin_length - offset - qbot_radius

        return distance

        #Used to map values from one range to another
    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    #Finds the bin closest to the qbot for the sensor to read
    def closest_bin(self):
        distances = []
        for i in range(1,5):
            bin_id = "Bin0"+str(i)
            distances.append(self.box_to_bin_distance(bin_id))
        ind = distances.index(min(distances))
        closest = "Bin0"+str(ind+1)
        return closest

# ------------------------------------------------------------------------------------------------------------------------
# Sensors

    def activate_ultrasonic_sensor(self):
        if self.hardware:
            self.GPIO.setup(self.GPIO_TRIGGER, self.GPIO.OUT)
            self.GPIO.setup(self.GPIO_ECHO, self.GPIO.IN)
        self.ultrasonic_sensor_activated = True
        print("Ultrasonic sensor activated")

    def deactivate_ultrasonic_sensor(self):
        self.ultrasonic_sensor_activated = False
        print("Ultrasonic sensor deactivated")

    # Outputs a distance from the closest bin from the qbot's bumper to the front face of the bin.
    def read_ultrasonic_sensor(self):
        if self.ultrasonic_sensor_activated == True:
            if self.hardware:
                self.GPIO.output(self.GPIO_TRIGGER, True)
             
                # set Trigger after 0.01ms to LOW
                time.sleep(0.00001)
                self.GPIO.output(self.GPIO_TRIGGER, False)
             
                StartTime = time.time()
                StopTime = time.time()
             
                # save StartTime
                while self.GPIO.input(self.GPIO_ECHO) == 0:
                    StartTime = time.time()
             
                # save time of arrival
                while self.GPIO.input(self.GPIO_ECHO) == 1:
                    StopTime = time.time()
             
                # time difference between start and arrival
                TimeElapsed = StopTime - StartTime
                # multiply with the sonic speed (34300 cm/s)
                # and divide by 2, because there and back
                distance = (TimeElapsed * 34300) / 2
             
                return round(distance,2)
            else:
                bin_id = self.closest_bin()
                distance = self.box_to_bin_distance(bin_id)
                if distance <= self.ultrasonic_sensor_range:
                    return round(distance,2)
                else:
                    return 0
        else:
            print("Ultrasonic sensor not activated")

    #Deprecated
    def activate_hall_sensor(self):
        self.hall_sensor_activated = True
        print("Hall sensor activated")

    #Deprecated
    def deactivate_hall_sensor(self):
        self.hall_sensor_activated = False
        print("Hall sensor deactivated")

    #Deprecated
    # Outputs high voltage readings for a specific duration if the closest bin is metallic and the box is within the sensor's range.
    def read_hall_sensor(self, duration):
        if self.hall_sensor_activated == True:
            if self.hardware:
                reading = []
                elapsed_time = 0
                start_time = time.time()
                mcp = self.MCP3008(clk=self.CLK, cs=self.CS, miso=self.MISO, mosi=self.MOSI)
                while elapsed_time < duration:
                    time.sleep(interval) 
                    digital = mcp.read_adc(5)
                    analog = 5-digital*5.0/1023.0
                    reading.append(round(analog,1))
                    elapsed_time = time.time() - start_time
                return reading
            else:
                bin_id = self.closest_bin()
                distance = self.box_to_bin_distance(bin_id)
                if distance <= self.hall_sensor_range:
                    
                    r, g, b, self.metallic, roughness = self.bin.bin_properties(bin_id)

                    if self.metallic != 0:#Hall sensor maxes out around 2.5V 
                        reading = self.sensor_readings(duration, 2.1, 2.5)
                    else:
                        reading = self.sensor_readings(duration, 1.8, 2.0)
                        
                else:
                    reading = self.sensor_readings(duration, 1.8, 2.0)
                return reading
        else:
            print("Hall sensor not activated")


    def activate_ir_sensor(self):
        self.ir_sensor_activated = True
        print("Active IR sensor activated")

    def deactivate_ir_sensor(self):
        self.ir_sensor_activated = False
        print("Active IR sensor deactivated")

    # Outputs high voltage reading if the closest bin is within proximity to the QBot.
    def read_ir_sensor(self):
        if self.ir_sensor_activated == True:
            if self.hardware:
                mcp = self.MCP3008(clk=self.CLK, cs=self.CS, miso=self.MISO, mosi=self.MOSI)
                digital = mcp.read_adc(3)
                analog = digital*5.0/1023.0
                return analog
            else:
                bin_id = self.closest_bin()
                distance = self.box_to_bin_distance(bin_id)
                if distance <= self.ir_sensor_range:
                    reading = random.uniform(4.5,5.0)
                else:
                    reading = random.uniform(0.0,0.4)
                return reading
        else:
            print("Active IR sensor not activated")

    def activate_ldr_sensor(self):
        self.ldr_sensor_activated = True
        print("LDR sensor activated")

    def deactivate_ldr_sensor(self):
        self.ldr_sensor_activated = False
        print("LDR sensor deactivated")

    # Outputs high voltage readings if a bin is sensed around the QBot.
    def read_ldr_sensor(self):
        if self.ldr_sensor_activated == True:
            if self.hardware:
                self.GPIO.setup(self.GPIO_LDR, self.GPIO.OUT)
                self.GPIO.output(self.GPIO_LDR, self.GPIO.LOW)
                time.sleep(0.1)

                #Change the pin back to input
                self.GPIO.setup(self.GPIO_LDR, self.GPIO.IN)
                
                count = 0
                while (self.GPIO.input(self.GPIO_LDR) == self.GPIO.LOW):
                    count += 1
                reading = self.map_value(count,800,1300,1,0)
                return round(reading,2)
            else:
                bin_id = self.closest_bin()
                distance = self.box_to_bin_distance(bin_id)
                if distance <= self.ldr_sensor_range:
                    reading = random.uniform(0.6,1)
                else:
                    reading = random.uniform(0.0,0.4)
                return round(reading,2)
        else:
            print("LDR sensor not activated")

    
    def activate_color_sensor(self):
        if self.hardware:
            self.GPIO.setup(self.GPIO_OUT,self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
            self.GPIO.setup(self.GPIO_S2,self.GPIO.OUT)
            self.GPIO.setup(self.GPIO_S3,self.GPIO.OUT)
        self.color_sensor_activated = True
        print("Color sensor activated")
        
    def deactivate_color_sensor(self):
        self.color_sensor_activated = False
        print("Color sensor deactivated")

    # Outputs voltage readings as well as mapped RGB values based on the closest bin's color.
    def read_color_sensor(self):
        if self.color_sensor_activated == True:
            if self.hardware:
                NUM_CYCLES = 10
                self.GPIO.output(self.GPIO_S2,self.GPIO.LOW)
                self.GPIO.output(self.GPIO_S3,self.GPIO.LOW)
                time.sleep(0.3)
                start = time.time()
                for impulse_count in range(NUM_CYCLES):
                  self.GPIO.wait_for_edge(self.GPIO_OUT, self.GPIO.FALLING)
                duration = time.time() - start      #seconds to run for loop
                red  = round((NUM_CYCLES / duration),2)   #in Hz
                red_RGB = round(self.map_value(red,200,600,0,1),1)

                self.GPIO.output(self.GPIO_S2,self.GPIO.HIGH)
                self.GPIO.output(self.GPIO_S3,self.GPIO.HIGH)
                time.sleep(0.3)
                start = time.time()
                for impulse_count in range(NUM_CYCLES):
                  self.GPIO.wait_for_edge(self.GPIO_OUT, self.GPIO.FALLING)
                duration = time.time() - start
                green = round((NUM_CYCLES / duration),2)
                green_RGB = round(self.map_value(green,120,600,0,1),1)

                self.GPIO.output(self.GPIO_S2,self.GPIO.LOW)
                self.GPIO.output(self.GPIO_S3,self.GPIO.HIGH)
                time.sleep(0.3)
                start = time.time()
                for impulse_count in range(NUM_CYCLES):
                  self.GPIO.wait_for_edge(self.GPIO_OUT, self.GPIO.FALLING)
                duration = time.time() - start
                blue = round((NUM_CYCLES / duration),2)
                blue_RGB = round(self.map_value(blue,160,800,0,1),1)

                return [red, green, blue],[red_RGB, green_RGB, blue_RGB]
            
            else:
                bin_id = self.closest_bin()
                distance = self.box_to_bin_distance(bin_id)
                red_min = random.uniform(200, 250)
                red_max = random.uniform(600, 650)
                green_min = random.uniform(110,130)
                green_max = random.uniform(600, 650)
                blue_min = random.uniform(150,160)
                blue_max = random.uniform(800,850)
                if distance <= self.color_sensor_range:
                    self.r, self.g, self.b, metallic_property, roughness = self.bin.bin_properties(bin_id)
                    red = round(self.map_value(self.r,0,1,red_min,red_max),1)
                    green = round(self.map_value(self.g,0,1,green_min,green_max),1)
                    blue = round(self.map_value(self.b,0,1,blue_min,blue_max),1)
                    reading = [round(self.r,2), round(self.g,2), round(self.b,2)],[red,green,blue]
                else:
                    reading = [0,0,0],[round(red_min,2),round(green_min,2),round(blue_min,2)]        
                return reading
        else:
            print("Color sensor not activated")


# -----------------------------------------------------------------------------------------------
# Class used for internal use only. It was used to be used in conjuction with the sensors on the Q-Bot.
# Leave it out of the documentation

class bins:
    def __init__(self,bin_configuration):
        
        bin1_offset,bin2_offset,bin3_offset,bin4_offset = bin_configuration[0]

        #Roughness and metallic are 1 and false by default
        self.bin1_properties,self.bin2_properties,self.bin3_properties,self.bin4_properties = list(zip(bin_configuration[1],[False]*4,[1]*4))

        bin_size = 0.3
        bin_center_y = 0.8
        
        self.bin1_position = [1.0556, bin_center_y+bin1_offset+(bin_size/2),0]
        self.bin2_position = [0.0092, bin_center_y+bin2_offset+(bin_size/2),0]
        self.bin3_position = [0.0092, -bin_center_y-bin3_offset-(bin_size/2),0]
        self.bin4_position = [1.0556, -bin_center_y-bin4_offset-(bin_size/2),0]

    # Returns the position of the specified bin.
    def bin_position(self,bin_id):
        if bin_id == "Bin01":
            return self.bin1_position
        elif bin_id == "Bin02":
            return self.bin2_position
        elif bin_id == "Bin03":
            return self.bin3_position
        elif bin_id == "Bin04":
            return self.bin4_position

    # Returns the properties of the specified bin.
    def bin_properties(self,bin_id):
        if bin_id == "Bin01":
            return self.bin1_properties[0]+list(self.bin1_properties[1:])
        elif bin_id == "Bin02":
            return self.bin2_properties[0]+list(self.bin2_properties[1:])
        elif bin_id == "Bin03":
            return self.bin3_properties[0]+list(self.bin3_properties[1:])
        elif bin_id == "Bin04":
            return self.bin4_properties[0]+list(self.bin4_properties[1:])



# -----------------------------------------------------------------------------------------------


