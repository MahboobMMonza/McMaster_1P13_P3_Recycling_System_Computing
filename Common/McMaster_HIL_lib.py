from quanser.communications import Stream, StreamError, PollFlag, Timeout
from quanser.common import GenericError
from quanser.hardware import HIL, HILError, PWMMode, MAX_STRING_LENGTH
from quanser.multimedia import Video3D, Video3DStreamType, ImageFormat, ImageDataType
import math
import numpy as np
import sys
import time
import cv2
import os
import busio
import adafruit_vl6180x
sys.path.append('../Common/')

from library_qlabs_image_utilities import *
from library_qlabs import QuanserInteractiveLabs
from library_qlabs_qbot_hopper import QLabsQBotHopper
from library_qlabs_bottle_table import QLabsBottleTableSensorTowerShort, QLabsBottleTableSensorTowerTall

########################################################################
######################### VIRTUAL QBOT CLASSES ######################### 
########################################################################

class QBot2e:

    # Define class-level variables 
    _card = None
    _dev_num = None
    _RGB_stream = None
    _ai_channels = None
    _ai_buffer = None
    _di_channels = None
    _di_buffer = None
    _do_channels = None
    _do_buffer = None
    _oi_channels = None
    _oi_buffer = None
    _gyro_z_bias = None
    _oo_channels = None
    _oo_buffer = None
    _enc_channels = None
    _enc_buffer = None
    _image_data = None
    _hardware = None
    _QLabs = None


    _qbot_diameter = 0.235
    
    # Initilize QBot2e
    def __init__(self, device_num, spawn_hostname, QLabsHostname, hardware = False):
        print ("Initializing QBot2e...")
        
        # Define DAQ type
        self._card = HIL()
        self._dev_num = device_num
        self._QLabs = QuanserInteractiveLabs()
        self._hardware = hardware
        
        if (self._hardware == False):
            self._QLabs.open("tcpip://{}:18000".format(QLabsHostname))
        
        # Define device number (for simulated targets)
        board_identifier = "{}@tcpip://{}:18910?nagle='off'".format(int(self._dev_num), spawn_hostname)
        if (self._hardware==True):
            board_identifier = str(self._dev_num)
        
        try:
            self._card.open("qbot2e", board_identifier)
        
        except HILError as h:
            print(h.get_error_message())  

        # Configure analog channels
        self._ai_channels = np.array([0, 4, 5], dtype=np.int32)
        self._ai_buffer = np.zeros(len(self._ai_channels), dtype=np.float64)
        
        # Configure digital channels
        self._do_channels = np.array([i for i in range(28,35)], dtype=np.int32)
        self._do_buffer = np.array([0, 0, 0, 0, 1, 1, 1, 1], dtype=np.bool_)
        self._di_channels = np.array([i for i in range(28,58)], dtype=np.int32)
        self._di_buffer = np.zeros(len(self._di_channels), dtype=np.bool_)
        #self._card.set_digital_directions(self._di_channels, self._di_num_channels, self._do_channels, self._do_num_channels)
    
        # Configure other channels 
        self._oo_channels = np.array([2000, 2001], dtype=np.int32)
        self._oo_buffer = np.zeros(len(self._oo_channels), dtype=np.float64)
        self._oi_channels = np.array([1002, 3000, 3001, 3002, 11000, 11001, 12000, 16000], dtype=np.int32)
        self._oi_buffer = np.zeros(len(self._oi_channels), dtype=np.float64)

        # Configure encoder channels
        self._enc_channels = np.array([0, 1], dtype=np.int32)
        self._enc_buffer = np.zeros(len(self._enc_channels), dtype=np.int32)
        
        # Reset QBot2e
        self.reset()

        print("QBot2e Initialized")
    
    ################## ANALOG IN METHODS ##################
    
    # Get analog input buffers
    def update_ai_buffer(self): 
        try:
            self._card.read_analog(self._ai_channels, len(self._ai_channels), self._ai_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    # Read battery voltage
    def get_batt_volts(self):
        self.update_ai_buffer()
        return self._ai_buffer[0]
    
    ################## ENCODER METHODS ####################
    
    # Get/set encoder values
    def update_enc_buffer(self): 
        try:
            self._card.read_encoder(self._enc_channels, len(self._enc_channels), self._enc_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    def push_enc_buffer(self): 
        try:
            self._card.set_encoder_counts(self._enc_channels, len(self._enc_channels), self._enc_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    # Read raw encoder counts
    def read_encoder_count(self, _channel):
        self.update_enc_buffer()
        return self._enc_buffer[_channel]

    def read_all_encoders(self):
        self.update_enc_buffer()
        return self._enc_buffer
    
    ################## DIGITAL I/O METHODS ################
    
    # Get/set digital I/O buffers
    def update_di_buffer(self):
        try:
            self._card.read_digital(self._di_channels, len(self._di_channels), self._di_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    def push_do_buffer(self): 
        try:
            self._card.write_digital(self._do_channels, len(self._do_channels), self._do_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    # Set LED outputs
    def set_leds(self, bLED = [0, 0, 0, 0]):
        self._do_buffer[0:4] = bLED
        self.push_do_buffer()
        
    # Read all digital in
    def read_din(self):
        self.update_di_buffer()
        return self._di_buffer
    
    # Return bump sensor inputs
    def read_bump_sensors(self):
        self.update_di_buffer()
        return self._di_buffer[0:3]
        
    # Return button inputs
    def read_buttons(self):
        self.update_di_buffer()
        return self._di_buffer[8:11]
        
    # Read right dock IR; [NR, NC, NL, FR, FC, FL]
    def read_right_dock_ir(self):
        self.update_di_buffer()
        return self._di_buffer[13:19]
        
    # Read center dock IR; [NR, NC, NL, FR, FC, FL]
    def read_center_dock_ir(self):
        self.update_di_buffer()
        return self._di_buffer[19:25]
        
    # Read left dock IR; [NR, NC, NL, FR, FC, FL]
    def read_left_dock_ir(self):
        self.update_di_buffer()
        return self._di_buffer[25:31]
    
    ################## OTHER I/O METHODS ##################
    
    # Get/set other I/O buffers
    def update_oi_buffer(self): 
        try:
            self._card.read_other(self._oi_channels, len(self._oi_channels), self._oi_buffer)
        except HILError as h:
            print(h.get_error_message())  
            
    def push_oo_buffer(self):
        try:
            self._card.write_other(self._oo_channels, len(self._oo_channels), self._oo_buffer)
        except HILError as h:
            print(h.get_error_message())  
            
    # Read z gyro
    def read_z_gyro(self):
        self.update_oi_buffer()
        z_gyro = self._oi_buffer[3] - self._z_bias
        return z_gyro
        
    # Measure gyro z-axis bias
    def update_gyro_z_bias(self):
        gyro = 0.0
        for i in range(10000):
            self.update_oi_buffer()
            gyro += self._oi_buffer[3]
        self._gyro_z_bias = gyro/10000.0
        print ("Z-axis gyro bias: {}".format(self._gyro_z_bias))
        
    # Set motor speed command
    def set_velocity(self, velocity = [0, 0]):
        self._oo_buffer = np.array(velocity, dtype=np.float64)
        self.push_oo_buffer()

    ################## CONTROL METHODS ####################

    def dist_to_enc(self, distance):
        mm = distance * 1000
        wheel_angle = mm / 35.0
        motor_angle = wheel_angle * 49.5833
        counts = motor_angle * (52.0 / (2*math.pi))
        return counts
    
    # Move a set distance and angle open loop
    def move_time(self, distance = 0, angle = 0, move_time = 1):
        # Ignore command if time <= 0 seconds
        if(move_time > 0):
            avg_linear_velocity = distance / move_time
            avg_angular_velocity = angle / move_time
            linear_velocity_delta = avg_angular_velocity * (self._qbot_diameter / 2)
            velocity_command = [avg_linear_velocity + linear_velocity_delta, avg_linear_velocity - linear_velocity_delta]
            self.set_velocity(velocity_command)
            time.sleep(move_time)
            self.halt()
    
    # Move a set distance and angle using odometry
    def move_odo(self, distance = 0, angle = 0, move_time = 1):
        avg_linear_velocity = distance / move_time
        avg_angular_velocity = angle / move_time
        linear_velocity_delta = avg_angular_velocity * (self._qbot_diameter / 2)
        velocity_left = avg_linear_velocity + linear_velocity_delta
        velocity_right = avg_linear_velocity - linear_velocity_delta

        [enc_left, enc_right] = self.read_all_encoders()

        target_enc_left = enc_left + self.dist_to_enc(velocity_left * move_time)
        target_enc_right = enc_right + self.dist_to_enc(velocity_right * move_time)

        arr_l = False
        arr_r = False

        while not (arr_l and arr_r):
            [vl, vr] = [0,0]
            [enc_left, enc_right] = self.read_all_encoders()
            err_l = abs(enc_left - target_enc_left)
            err_r = abs(enc_right - target_enc_right)
            #print("l = {:.0f}, r = {:.0f}, l_t = {:.1f}, r_t = {:.1f}, err_l = {:.1f}, err_r = {:.1f}".format(enc_left, enc_right, target_enc_left, target_enc_right, err_l, err_r))
            if (err_l < 100) or arr_l:
                arr_l = True
            else:
                vl = velocity_left
            if (err_r < 100) or arr_r:
                arr_r = True
            else:
                vr = velocity_right

            self.set_velocity([vl, vr])
            time.sleep(0.01)
            
    # Stop both motors
    def halt(self):
        self.set_velocity([0, 0])
    
    ################## EXTERNAL DEVICES ##################
    
    def command_dump(self, angle):
        if self._hardware:
            #Hardware interface not defined
            pass
        else:
            QLabsQBotHopper().commandDegrees(self._QLabs, self._dev_num, angle)
    
    
    ################## DEVICE MANAGEMENT ##################
    
    # Reset QBot2e
    def reset(self):
        print ("Resetting QBot2e...")
        
        #Clear LEDs
        self.set_leds([0, 0, 0, 0])

        # Stop motors
        self.set_velocity([0, 0])
        
        # Reset encoders to 0
        self._enc_buffer = np.zeros(len(self._enc_channels), dtype=np.int32)
        self.push_enc_buffer()
        
        # Sample gyroscope z-axis bias
        #if self._gyro_z_bias == None:
        #    self.update_gyro_z_bias()

    # Close DAQ
    def close(self):
        self.__exit__()
        
    # Exit routine
    def __exit__(self):
        if not self._QLabs == None:
            self._QLabs.close()
        print ("Closing DAQ...")
        self.reset()
        self._card.close()
        print ("QBot2e closed")

#####################################################################

#class Kinect
#Defines methods for initializing the kinect and getting images 
class Kinect:

    # Define class-level variables
    _image_width = 640
    _image_height = 480
    _image_rate = 10
    _image_buffer = np.zeros((_image_height, _image_width, 3), dtype=np.uint8)
    _stream_index = 0
    _3D_camera = None
    _RGB_stream = None
    _depth_stream = None
    _status = 0

    # Initialize Kinect
    def __init__(self, QBot2eHostname = 'localhost', hardware = False, rate = 10):
        print ("Initializing Kinect")
        self._image_rate = rate
        self._image_buffer = self.placeholder_image()
        
        ID = "0@tcpip://localhost:18911"
        if (hardware==True):
            ID = "0"
        
        try:
            self._3D_camera = Video3D(ID)
            print("Created Video3D instance")
            self._RGB_stream = self._3D_camera.stream_open(Video3DStreamType.COLOR,
                                                           self._stream_index,
                                                           self._image_rate,
                                                           self._image_width,
                                                           self._image_height,
                                                           ImageFormat.ROW_MAJOR_INTERLEAVED_BGR,
                                                           ImageDataType.UINT8)
            print("Opened RGB stream")
            self._depth_stream = self._3D_camera.stream_open(Video3DStreamType.DEPTH,
                                                           self._stream_index,
                                                           self._image_rate,
                                                           self._image_width,
                                                           self._image_height,
                                                           ImageFormat.ROW_MAJOR_INTERLEAVED_BGR,
                                                           ImageDataType.UINT8)
            print("Opened depth stream")
            self._3D_camera.start_streaming()
            print("Waiting for Kinect...")
            for i in range(0,300):
                cv2.waitKey(33)
                frame = self._RGB_stream.get_frame()
                if frame != None:
                    frame.get_data(self._image_buffer)
                    frame.release()
                    self._status = 1
                    break
        
        except GenericError as ex:
            print("ERROR: " + ex.get_error_message())
            
        if self._status == 0:
            print("No frames received, Kinect initialization failed")
            self.halt()
        else:
            print("Kinect Initialized")


    #Get an RGB frame from the Kinect
    def get_RGB_frame(self):
        frame = self._RGB_stream.get_frame()
        if frame != None:
            frame.get_data(self._image_buffer)
            frame.release()
        return self._image_buffer
        
    #Get a depth frame from the Kinect
    def get_depth_frame(self):
        frame = self._depth_stream.get_frame()
        if frame != None:
            frame.get_data(self._image_buffer)
            frame.release()
        return self._image_buffer

    #Return the status of the Kinect (are frames being received)
    def get_status(self):
        return self._status

    #Return a placeholder image
    def placeholder_image(self):
        return cv2.imread('DefaultImage.jpg')

    #Cleanup for shutting down the Kinect
    def halt(self):
        if self._3D_camera != None:
            print("Stopping stream")
            self._3D_camera.stop_streaming()
            self._3D_camera = None
        if self._RGB_stream != None:
            print("Closing stream")
            self._RGB_stream.close()
            self._RGB_stream = None
        print("Kinect reset")
        self._status = 0

#####################################################################

#class CameraUI
#Defines methods for image UI and processing images using OpenCV
class CameraUI:

    #Define a camera UI using openCV for doing line following
    #_hue_ctr and _hue_width updated from 49 and 17 to 60 and 40 - BASEM 
    _max_speed = 0
    _hue_ctr = 60
    _hue_width = 40
    _ROI_x = [0,640]
    _ROI_y = [0,480]

    def __init__(self, look_ahead = 0.2, ROI_height = 32):

        y_min = int(480 * (1 - look_ahead))
        self._ROI_x = [0, 640]
        self._ROI_y = [y_min - ROI_height, y_min]

        #cv2.startWindowThread()
##        cv2.namedWindow('rgb_stream', cv2.WINDOW_AUTOSIZE)
##        cv2.namedWindow('binary_ROI', cv2.WINDOW_AUTOSIZE)
        img_RGB = cv2.imread('DefaultImage.jpg')
##        cv2.imshow('rgb_stream', img_RGB)
        img_binary = crop_rect(img_RGB, self._ROI_x, self._ROI_y)
##        cv2.imshow('binary_ROI', img_binary)
##
##        cv2.createTrackbar("Hue Center", 'binary_ROI', self._hue_ctr, 360, self._on_center)
##        cv2.createTrackbar("Hue Width", 'binary_ROI', self._hue_width, 180, self._on_width)
##        cv2.createTrackbar("Speed (mm/s)", 'binary_ROI', self._max_speed, 500, self._on_speed)
##        cv2.waitKey(1000)
        
    def _on_center(self, val):
        self._hue_ctr = val

    def set_center(self, val):
        cv2.setTrackbarPos("Hue Center", 'binary_ROI', val)
        self._on_center(val)

    def _on_width(self, val):
        self._hue_width = val

    def set_width(self, val):
        cv2.setTrackbarPos("Hue Width", 'binary_ROI', val)
        self._on_width(val)

    def _on_speed(self, val):
        self._max_speed = val/1000

    def set_speed(self, val):
        cv2.setTrackbarPos("Speed (mm/s)", 'binary_ROI', val)
        self._on_speed(val)

    def process(self, img_RGB):
        #Threshold image for a given hue center and width
        img_buffer = img_RGB
        img_bin = hue_threshold(img_RGB, self._hue_ctr, self._hue_width, 360)

        #Crop thresholded image to ROI and show
        img_cropped = crop_rect(img_bin, self._ROI_x, self._ROI_y)
        #cv2.imshow('binary_ROI', img_cropped) 

        #Find center of line segment in ROI
        line_ctr = extract_line_ctr(img_cropped)

        #Overlay ROI and line location on image and show
        img_overlay = show_ROI_target(img_buffer, self._ROI_x, self._ROI_y, line_ctr)
        #cv2.imshow('rgb_stream', img_overlay)

        #cv2.waitKey(33)

        return line_ctr

    def get_ROI(self):
        return self._ROI_x, self._ROI_y

    def get_hue(self):
        return self._hue_ctr, self._hue_width

    def get_speed_lim(self):
        return self._max_speed

    def destroy(self):
        print("Closing UI")
        cv2.destroyAllWindows()

########################################################################
####################### (VIRTUAL) QARM CLASSES ######################### 
########################################################################

class QArm:

    # Define class-level variables 
    base = 0
    shoulder = 0
    elbow = 0
    wrist = 0
    gripper = 0
    contact = 0
    contact_id = 0
    static_environment_collision = 0
    finger_pad_detection_right_proximal = 0
    finger_pad_detection_right_distal = 0
    finger_pad_detection_left_proximal = 0
    finger_pad_detection_left_distal = 0
    
    _dev_num = 0

    # Manipulator parameters in meters:
    _L1 = 0.127
    _L2 = 0.3556
    _L3 = 0.4064

    # Define joint angle (in deg) and gripper limits
    _qarm_base_upper_lim = 175
    _qarm_base_lower_lim = -175
    _qarm_shoulder_upper_limit = 90
    _qarm_shoulder_lower_limit = -90
    _qarm_elbow_upper_limit = 90
    _qarm_elbow_lower_limit = -80
    _qarm_wrist_upper_limit = 170
    _qarm_wrist_lower_limit = -170
    _qarm_gripper_upper_limit = 1
    _qarm_gripper_lower_limit = 0
    
    # Define base LED color
    _base_color_r = 1
    _base_color_g = 0
    _base_color_b = 0
        
    _arm_brightness = 1 
    
    _err_lim = 0.05
        
    image_rgb = None
    image_depth = None
    
    #region: Channel and Buffer definitions
    # Other Writes 
    write_other_channels = np.array([1000, 1001, 1002, 1003, 1004, 11000, 11001, 11002, 11003], dtype=np.int32)
    write_other_buffer = np.zeros(9, dtype=np.float64)
    write_LED_channels = np.array([11005, 11006, 11007], dtype=np.int32)
    write_LED_buffer = np.array([1,0,0], dtype=np.float64)

    # Other Reads
    read_other_channels = np.array([1000, 1001, 1002, 1003, 1004, 3000, 3001, 3002, 3003, 3004, 10000, 10001, 10002, 10003, 10004, 11000, 11001, 11002, 11003, 11004], dtype=np.int32)
    read_other_buffer = np.zeros(20, dtype=np.float64)
    
    # User LEDs Write Only - Other channel 11004:11007 are User LEDs
    read_analog_channels = np.array([5, 6, 7, 8, 9], dtype=np.int32)
    read_analog_buffer = np.zeros(5, dtype=np.float64)

    measJointCurrent = np.zeros(5, dtype=np.float64)
    measJointPosition = np.zeros(5, dtype=np.float64)
    measJointSpeed = np.zeros(5, dtype=np.float64)
    measJointPWM= np.zeros(5, dtype=np.float64)
    measJointTemperature = np.zeros(5, dtype=np.float64)    
    status = False
    #endregion
    
    # Initilize QuanserSim
    def __init__(self, device_num, QArm_hostname, hardware = False):
    
        self._dev_num = device_num
        self.mode = 0 # Only Position Mode is tested and available in Python
        self.card = HIL()
        
        print("HIL initialized")

        board_identifier = "{}@tcpip://{}:18900?nagle='off'".format(int(self._dev_num), QArm_hostname)

        if (hardware==True):
            board_identifier = str(self._dev_num)

        board_specific_options = "j0_mode=0;j1_mode=0;j2_mode=0;j3_mode=0;gripper_mode=0;"

        try:
            # Open the Card
            self.card.open("qarm_usb", board_identifier)
            if self.card.is_valid():
                self.card.set_card_specific_options(board_specific_options, MAX_STRING_LENGTH)
                self.status = True

                print('QArm configured in Position Mode.')
        
        except HILError as h:
            print(h.get_error_message())  
        print ("QArm initialized")

    # Set base LED color; color is an array of 3 elements of [r, g, b]; element values from 0-1
    def set_base_color (self, color=[1, 0, 0]):
        
        self.write_LED_buffer = np.array(color, dtype=np.float64)
       
        # IO
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, None, 0, None, 0, self.write_LED_channels, 3, 
                                None, None, None, self.write_LED_buffer)  

        except HILError as h:
            print(h.get_error_message())

        finally:
            pass     
        
    def return_home(self):
        self.qarm_move(0, 0, 0, 0, 0)

    def qarm_move_degrees(self, base, shoulder, elbow, wrist, gripper, wait = True):
        b = 2 * math.pi * (base/360.0)
        s = 2 * math.pi * (shoulder/360.0)
        e = 2 * math.pi * (elbow/360.0)
        w = 2 * math.pi * (wrist/360.0)
        self.qarm_move(b, s, e, w, gripper, wait)
    
    # All angles in rads
    def qarm_move(self, base, shoulder, elbow, wrist, gripper, wait = True):
        
        self.write_other_buffer[0:4] = np.array([base, shoulder, elbow, wrist], dtype=np.float64)
        grpCMD = np.maximum(0.1, np.minimum(gripper, 0.9))
        self.write_other_buffer[4] = grpCMD
        
        self.write_all_arm_joints() 
        
        if (wait == True):
            reached = False
            count = 0
            while not reached and not count > 100:
                b, s, e, w, g = self.read_all_arm_joints()
                #errors = (abs(b - base), abs(s - shoulder), abs(e - elbow), abs(w - wrist), abs(g - grpCMD))
                if ((abs(b - base) < self._err_lim) and (abs(s - shoulder) < self._err_lim) and (abs(e - elbow) < self._err_lim) and (abs(w - wrist) < self._err_lim) and (abs(g - grpCMD) < self._err_lim)):
                    reached = True
                else:
                    time.sleep(0.1)
                    count = count + 1
                    #print(errors)
            return b, s, e, w, g 
        return 0
        
    def qarm_move_gripper(self, gripper, wait = True):
        grpCMD = np.maximum(0.1, np.minimum(gripper, 0.9))
        self.write_other_buffer[4] = grpCMD
        
        self.write_all_arm_joints() 
        
        if (wait == True):
            reached = False
            while not reached:
                b, s, e, w, g = self.read_all_arm_joints()
                if ((abs(g - grpCMD) < self._err_lim)):
                    reached = True
                else:
                    time.sleep(0.1)
            return g        
        return 0

    def read_all_arm_joints(self):
        try:
            self.card.read(None, 0, None, 0, None, 0, self.read_other_channels, 20, None, None, None, self.read_other_buffer)
        except HILError as h:
            print(h.get_error_message())   
        finally:
            b, s, e, w, g = self.read_other_buffer[0:5]
        return b, s, e, w, g
        
    def write_all_arm_joints(self):
        try:
            self.card.write(None, 0, None, 0, None, 0, self.write_other_channels, 9, 
                    None, None, None, self.write_other_buffer)
                
        except HILError as h:
            print(h.get_error_message())   
        
        finally: 
            pass

    def terminate(self):
        ''' This function terminates the QArm card after setting final values for home position and 0 pwm.'''
        
        self.set_base_color([1, 0, 0]) 
        self.write_other_buffer = np.zeros(9, dtype=np.float64)
        
        try:    
            self.card.write(None, 0, None, 0, None, 0, self.write_other_channels, 9, 
                                 None, None, None, self.write_other_buffer)
            self.card.close()
            print('QArm terminated successfully.')
            
        except HILError as h:
            print(h.get_error_message())
            
    def close(self):
        self.terminate()
     
    # Check if given joint angles and gripper value are within acceptable limit
    # Return 1 if withing bound, 0 otherwise
    def angles_within_bound (self, qarm_base, qarm_shoulder, qarm_elbow, qarm_wrist, qarm_gripper):
        if qarm_base > self._qarm_base_upper_lim or qarm_base < self._qarm_base_lower_lim or \
                qarm_shoulder > self._qarm_shoulder_upper_limit or qarm_shoulder < self._qarm_shoulder_lower_limit or \
                qarm_elbow > self._qarm_elbow_upper_limit or qarm_elbow < self._qarm_elbow_lower_limit or \
                qarm_wrist > self._qarm_wrist_upper_limit or qarm_wrist < self._qarm_wrist_lower_limit or \
                qarm_gripper > self._qarm_gripper_upper_limit or qarm_gripper < self._qarm_gripper_lower_limit:
            return 0
        else:
            return 1

    # Check if given end-effector coordinates are within bounds
    # Return 1 if withing bound, 0 otherwise
    def coordinates_within_bound(self, p_x, p_y, p_z):
        R = math.sqrt(p_x ** 2 + p_y ** 2)

        # Vertical offset within the verical plane from Frame 1 to End-Effector
        # Note: Frame 1 y-axis points downward (negative global Z-axis direction)
        Z = self._L1 - p_z

        # Distance from Frame 1 to End-Effector Frame
        Lambda = math.sqrt(R ** 2 + Z ** 2)

        if Lambda > (self._L2 + self._L3) or p_z < 0:
            return 0
        else:
            return 1

    # Calculate standard DH parameters
    # Inputs:
    # a       :   translation  : along : x_{i}   : from : z_{i-1} : to : z_{i}
    # alpha   :      rotation  : about : x_{i}   : from : z_{i-1} : to : z_{i}
    # d       :   translation  : along : z_{i-1} : from : x_{i-1} : to : x_{i}
    # theta   :      rotation  : about : z_{i-1} : from : x_{i-1} : to : x_{i}
    # Outputs:
    # transformed       : transformation                   : from :     {i} : to : {i-1}
    def qarm_dh(self, theta, d, a, alpha):
        # Rotation Transformation about z axis by theta
        a_r_z = np.array(
            [[math.cos(theta), -math.sin(theta), 0, 0],
             [math.sin(theta), math.cos(theta), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        # Translation Transformation along z axis by d
        a_t_z = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, d],
             [0, 0, 0, 1]])

        # Translation Transformation along x axis by a
        a_t_x = np.array(
            [[1, 0, 0, a],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        # Rotation Transformation about x axis by alpha
        a_r_x = np.array(
            [[1, 0, 0, 0],
             [0, math.cos(alpha), -math.sin(alpha), 0],
             [0, math.sin(alpha), math.cos(alpha), 0],
             [0, 0, 0, 1]])

        # For a transformation from frame {i} to frame {i-1}: transformed
        transformed = a_r_z @ a_t_z @ a_r_x @ a_t_x

        return transformed

    # Calculate end-effector position (x, y, z) using forward kinematics
    # Input:    joint angles in rads
    # Output:   end-effector position (x, y, z) expressed in base frame {0}
    def qarm_forward_kinematics(self, joint1, joint2, joint3, joint4):
        # Transformation matrices for all frames:
        # A{i-1}{i} = quanser_arm_dh(theta, d, a, alpha)

        A01 = self.qarm_dh(joint1, self._L1, 0, -math.pi/2)
        A12 = self.qarm_dh(joint2 - math.pi/2, 0, self._L2, 0)
        A23 = self.qarm_dh(joint3, 0, 0, -math.pi/2)
        A34 = self.qarm_dh(joint4, self._L3, 0, 0)

        A04 = A01 @ A12 @ A23 @ A34

        # Extract and return the x, y, z Position rounded to four decimal digits
        return round(A04[0, 3], 4), round(A04[1, 3], 4), round(A04[2, 3], 4)

    # Compute the position of the end-effector using inverse kinematics
    #
    # The solution is based on the geometric configuration of the QArm
    # where the upper links are contained within the vertical plane rotating
    # with the based joint angle q1.
    # The frame definition is consistent with the S&V DH convention.
    # Inputs: end-effector position, p_x, p_y, p_z
    # Outputs: joint angles in degrees (base, shoulder, elbow) based on inverse kinematics
    def qarm_inverse_kinematics(self, p_x, p_y, p_z):

        # Initialization
        q_base = 0
        q_shoulder = 0
        q_elbow = 0

        # Base angle:
        q_base = math.atan2(p_y, p_x)

        # Geometric definitions
        # Radial distance (R) projection onto the horizontal plane
        R = math.sqrt(p_x**2 + p_y**2)

        # Vertical offset within the verical plane from Frame 1 to End-Effector
        # Note: Frame 1 y-axis points downward (negative global Z-axis direction)
        Z = self._L1 - p_z

        # Distance from Frame 1 to End-Effector Frame
        Lambda = math.sqrt(R**2 + Z**2)

        # Angle of Lambda vector from horizontal plane (Frame 1)
        # Note: theta is measured about z-axis of Frame 1 so positive theta
        # rotates Lambda "down".
        theta = math.atan2(Z, R)

        # Based angle of the triangle formed by L2, L3 and Lambda
        # Computed using cosine law
        # Note: The sign of alpha determines whether it is elbow up (alpha < 0) or
        # elbow down (alpha > 0) configuration (i.e., consistent with Frame 1)
        alpha = math.acos(-(self._L3**2 - self._L2**2 - Lambda**2) / (2*self._L2*Lambda))

        #Solve for q_shoulder; elbow up solution
        q_shoulder = math.pi/2 + (theta - alpha)

        #Solve for q_elbow, elbow up solution
        q_elbow = math.atan2(self._L2 - R*math.sin(q_shoulder) + Z*math.cos(q_shoulder), R*math.cos(q_shoulder) + Z*math.sin(q_shoulder))

        # Convert angles to degrees
        q_base_deg = math.degrees(q_base)
        q_shoulder_deg = math.degrees(q_shoulder)
        q_elbow_deg = math.degrees(q_elbow)

        # Return the joint angles in degrees
        return round(q_base_deg, 4), round(q_shoulder_deg, 4), round(q_elbow_deg, 4)
 
########################################################################
#################### VIRTUAL ROTARY TABLE CLASSES ###################### 
########################################################################

class RotaryTable:

    # Define class-level variables 
    
    _card = None
    _dev_num = None
    _ai_channels = None
    _ai_buffer = None
    _ao_channels = None
    _ao_buffer = None
    _enc_channels = None
    _enc_buffer = None
    
    _tof_sensor = None
    
    _encoder_value = None
    _relative_x = None
    _relative_y = None
    _relative_z = None
    _properties = None
    
    _hardware = False
    _QLabs = None

    
    # Initilize Virtual rotary table
    def __init__(self, device_num = 0, spawn_hostname = 'localhost', QLabsHostname = 'localhost', hardware = False):
        print ("Initializing Rotary Table...")
        
        # Define DAQ type
        self._card = HIL()
        self._dev_num = device_num
        self._QLabs = QuanserInteractiveLabs()
        self._hardware = hardware

        if (self._hardware == False):
            
            self._QLabs.open("tcpip://{}:18000".format(QLabsHostname))
        # Define device number (for simulated targets)
        self._spawn_hostname = spawn_hostname
        board_identifier = "{}@tcpip://{}:18940?nagle='off'".format(int(self._dev_num), self._spawn_hostname)
        #print(board_identifier)
        
        # Hardware-specific setup
        if (self._hardware==True):
            board_identifier = str(self._dev_num)
            self._tof_sensor = adafruit_vl6180x.VL6180X(busio.I2C(board.SCL, board.SDA))
        
        try:
            if (self._hardware==False):
                self._card.open("q8_usb", board_identifier)
            else:
                self._card.open("q2_usb", board_identifier)
        
        except HILError as h:
            print(h.get_error_message())  

        # Configure analog channels
        self._ai_channels = np.array([0, 1], dtype=np.int32)
        self._ai_buffer = np.zeros(len(self._ai_channels), dtype=np.float64)
        self._ao_channels = np.array([0, 1], dtype=np.int32)
        self._ao_buffer = np.zeros(len(self._ai_channels), dtype=np.float64)
    
        # Configure encoder channels
        self._enc_channels = np.array([0, 1], dtype=np.int32)
        self._enc_buffer = np.zeros(len(self._enc_channels), dtype=np.int32)
        print ("Virtual rotary table initialzied")
    
    ##--------------- VIRTUAL ROTARY TABLE METHODS ----------------------- 

    ################## ANALOG METHODS ##################
    
    # Get analog input buffers
    def update_ai_buffer(self): 
        try:
            self._card.read_analog(self._ai_channels, len(self._ai_channels), self._ai_buffer)
        except HILError as h:
            print(h.get_error_message())  
            
    # Set analog output buffers
    def push_ao_buffer(self): 
        try:
            self._card.write_analog(self._ao_channels, len(self._ao_channels), self._ao_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    def command_voltage(self, voltage):
        self._ao_buffer[0] = -voltage
        self.push_ao_buffer()
        
    # Read external analog sensor
    def read_analog_input(self):
        self.update_ai_buffer()
        return self._ai_buffer[0]
    
    ################## ENCODER METHODS ####################
    
    # Get/set encoder values
    def update_enc_buffer(self): 
        try:
            self._card.read_encoder(self._enc_channels, len(self._enc_channels), self._enc_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    def push_enc_buffer(self): 
        try:
            self._card.set_encoder_counts(self._enc_channels, len(self._enc_channels), self._enc_buffer)
        except HILError as h:
            print(h.get_error_message())  
        
    # Read raw encoder counts
    def read_encoder(self):
        self.update_enc_buffer()
        return -self._enc_buffer[0]
        
    def write_encoder(self, counts):
        self._enc_buffer[0] = -counts
        self.push_enc_buffer()

    ################## CONTROL METHODS ####################

    # Rotate table clockwise for a given positive speed
    def rotate_clockwise (self, voltage):
        #open loop placeholder
        self.command_voltage(voltage)
    
    # Rotate table counter clockwise for a positive speed
    def rotate_counterclockwise (self, voltage):
        #open loop placeholder
        self.command_voltage(-voltage)
    
    # Rotate table for given angle in degrees in CW direction (open-loop)
    def command_rel_position_cw(self, angle):

        if(angle < 0):
            target = 360 - angle
        else:
            target = angle
        # Encoder counts to degrees
        K_enc = 360/4096
        voltage = -0.5
        
        initial_encoder_count = self.read_encoder()
        #print("Init encoder: ", initial_encoder_count)
        
        current_encoder_count = initial_encoder_count
        target_encoder_count = initial_encoder_count + (angle/K_enc)
        
        
        while current_encoder_count < target_encoder_count:
            current_encoder_count = self.read_encoder()
            #print("Curr encoder: {}".format(current_encoder_count))
            self.rotate_clockwise(voltage)
        
        print ("Commanded (deg): ", angle)
        print ("Actual (deg): ", (current_encoder_count - initial_encoder_count)*K_enc)
        self.stop_table()

    def command_abs_position_pid(self, angle):
        # Encoder counts to degrees
        K_enc = 360/4096
        #Proportional gain
        Kp = .05
        # Saturation voltage
        saturation_voltage = 0.5
        
        initial_encoder_count = self.read_encoder()
        
        current_angle = initial_encoder_count
        
        error = angle - current_angle

        while abs(error) > 0.1:
            try:
                PTerm = Kp*error
                speed = -PTerm
                    
                # Saturate speed at saturation value
                if speed > saturation_voltage:
                    speed = saturation_voltage
                if -speed > saturation_voltage:
                    speed = -saturation_voltage
                
                self.command_voltage(speed)
                current_encoder_count = self.read_encoder()
                current_angle = current_encoder_count*K_enc
            
                error = angle - current_angle

            except KeyboardInterrupt:    
                print("User Interrupt")
                break
        
        #print ("Commanded (deg): {}".format(angle))
        #print ("Actual (deg): {}".format(current_angle))
        self.stop_table()

    # Stop rotarytable
    def stop_table(self):
        self.command_voltage(0)
        
    def terminate(self):
        self.stop_table()
        
        try:    
            self._card.close()
            print('Rotary Table terminated successfully.')
            
        except HILError as h:
            print(h.get_error_message())
            
    def close(self):
        self.terminate()
        
    ################## EXTERNAL SENSORS ####################   
    #Dedented these 2 functions that were for some reason inside close()
    def read_TOF(self):
        if self._hardware:
            tof = self._tof_sensor.range
        else:
            tof = QLabsBottleTableSensorTowerTall().GetTOF(self._QLabs, self._dev_num)
        return tof
    
    
    def read_inductive_proximity(self):
        if self._hardware:
            prox = self.read_analog_input()
        else:
            location, properties = QLabsBottleTableSensorTowerShort().GetProximity(self._QLabs, self._dev_num)
            if properties == "METAL":
                prox = 3
            else:
                prox = 10
        return prox
        
    
 
########################################################################
######################## VIRTUAL EMG CLASSES ########################### 
########################################################################

class EMGSim:

    # Define class-level variables 
    
    image_width = 700
    image_height = 224
    
    _emgLeft = 0.0
    _emgRight = 0.0
    _imgHead = cv2.imread('../Common/HeadOutline.png')
    _imgFlexSheet = cv2.imread('../Common/FlexAnimSpriteSheet.png')
    _imgFlexArray = np.zeros((200, 300, 3, 13), dtype=np.uint8)
    _imgBGnd = np.full((image_height, image_width, 3), (255, 255, 255), dtype=np.uint8)
    
    
    ##--------------- VIRTUAL EMG METHODS ----------------------- 
    
    # Callback fcn for left slider
    def _on_change_Left(self, left):
        self._emgLeft = left / 100.0
        self._updateEMGImage()
        
    # Callback fcn for right slider
    def _on_change_Right(self, right):
        self._emgRight = right / 100.0
        self._updateEMGImage()
    
    # Read EMG values and check for inputs
    def readEMG(self):
        cv2.waitKey(1)
        return (self._emgLeft, self._emgRight)
        
    def EMG_right(self):
        cv2.waitKey(1)
        return self._emgRight
        
    def EMG_left(self):
        cv2.waitKey(1)
        return self._emgLeft
    
    # Draw the flexing guy
    def _updateEMGImage(self):
        imgComp = self._imgBGnd
        imgComp[24:224, :300, :] = self._imgFlexArray[:, :, :, int(self._emgLeft * 12)]
        imgComp[24:224, (self.image_width - 300):, :] = cv2.flip(self._imgFlexArray[:, :, :, int(self._emgRight * 12)], 1)
        imgComp[:170, 268:438, :] = cv2.bitwise_and(imgComp[:170, 268:438, :], self._imgHead)
        cv2.imshow("Myo Sim", imgComp)
        return None
    
    # Close window
    def close(self):
        cv2.destroyWindow("Myo Sim")
        return None
                
    # Initilize EMG sensor
    def __init__(self):
        for i in range(13):
            self._imgFlexArray[:, :, :, i] = self._imgFlexSheet[(200*i):(200*(i+1)), :300, :]
        cv2.imshow("Myo Sim", self._imgBGnd)
        cv2.createTrackbar("Left (%)", "Myo Sim", 0, 100, self._on_change_Left)
        cv2.createTrackbar("Right (%)", "Myo Sim", 0, 100, self._on_change_Right)
        self._updateEMGImage()
        cv2.waitKey(1)
        print ("Virtual EMG initialized")
        return None

