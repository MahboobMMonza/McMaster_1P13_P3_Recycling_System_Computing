import sys
import time
sys.path.append('../')
from Common.project_library import *

# Modify the information below according to you setup and uncomment the entire section

# 1. Interface Configuration
project_identifier = 'P3B' # enter a string corresponding to P0, P2A, P2A, P3A, or P3B
ip_address = '169.254.134.253' # enter your computer's IP address
hardware = False # True when working with hardware. False when working in the simulation

# 2. Servo Table configuration
short_tower_angle = 270 # enter the value in degrees for the identification tower 
tall_tower_angle = 0 # enter the value in degrees for the classification tower
drop_tube_angle = 180 # enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# 3. Qbot Configuration
bot_camera_angle = -21.5 # angle in degrees between -21.5 and 0

# 4. Bin Configuration
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin 

bin1_offset = 0.17 # offset in meters
bin1_color = [1,0,0] # e.g. [1,0,0] for red
bin2_offset = 0.17
bin2_color = [0,1,0]
bin3_offset = 0.17
bin3_color = [0,0,1]
bin4_offset = 0.17
bin4_color = [0,0,0]

#--------------- DO NOT modify the information below -----------------------------

if project_identifier == 'P0':
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    bot = qbot(0.1,ip_address,QLabs,None,hardware)
    
elif project_identifier in ["P2A","P2B"]:
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    arm = qarm(project_identifier,ip_address,QLabs,hardware)

elif project_identifier == 'P3A':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    configuration_information = [table_configuration,None, None] # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    
elif project_identifier == 'P3B':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    qbot_configuration = [bot_camera_angle]
    bin_configuration = [[bin1_offset,bin2_offset,bin3_offset,bin4_offset],[bin1_color,bin2_color,bin3_color,bin4_color]]
    configuration_information = [table_configuration,qbot_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    bins = bins(bin_configuration)
    bot = qbot(0.1,ip_address,QLabs,bins,hardware)
    

#---------------------------------------------------------------------------------
# STUDENT CODE BEGINS
#---------------------------------------------------------------------------------

##bot.travel_forward(0.2)
##table.rotate_table_angle(45)
##arm.rotate_base(45)

# Defining (t) global constants to be used throughout the program
ARM_HOME = (0.406, 0, 0.483)
BOT_HOME = (1.5, 0, 0)
PICK_UP = (0.63, 0, 0.26)
DROP_OFF = (0, -0.50, 0.55)
'''
def relative_position(pos):
    Determines the position of the bot compared to its home position
    Returns True if the positions are about 0.3 m from original home position
    Returns False otherwise
    cur_pos = bot.position()
    for i in range(3):
        if abs(pos[i] - cur_pos[i]) > 0.3:
            return False
    return True
'''

def load_bottle():
    '''
    Position the Qbot near the Qarm and loads a bottle onto the Qbot using Qarm
    '''
    # Need to make this less repetitive and readable
    bot.rotate(-93)
    time.sleep(0.5)
    bot.forward_distance(0.05)
    arm.move_arm(*PICK_UP)
    time.sleep(1)
    arm.control_gripper(40)
    time.sleep(1)
    arm.move_arm(*ARM_HOME)
    time.sleep(1)
    arm.move_arm(*DROP_OFF)
    time.sleep(3)
    arm.control_gripper(-40)
    time.sleep(1)
    arm.rotate_shoulder(-25)
    time.sleep(1)
    arm.rotate_base(45)
    time.sleep(0.5)
    bot.rotate(93)
    arm.home()

def dispension(bottle_id):
    '''
    Gets and returns the bin destination of the given bin
    '''
    return int(table.dispense_container(bottle_id, True)[2][3:])


def bot_home():
    '''
    Determines if the bot is near its home position. Returns True for a
    difference <= 0.05 m before each coordinate and False otherwise.
    '''
    pos = bot.position()
    for i, j in zip(BOT_HOME, pos):
        if i - j > 0.1:
            return False
    return True


def line_follow(bin_num=-1):
    '''
    Makes the Q-bot follow the line until it reaches the intended bin number, or
    if the bin number is -1, then follows the line home
    '''
    bot.activate_ultrasonic_sensor()
    mult = 4
    PROX_DIST = 0.10
    PASS_DIST = 0.2
    cur_dist = 10000
    # if bot_home():
      #  bot.forward_distance(0.25) # move away from home position to run
    cur_bin = 0 # tracks the number of bins passed
    reading, speeds = [], [] # lists to hold line sensor readings and wheel speeds
    encounter_bin = False # Flag to check if currently near a bin
    print('Target:', bin_num)
    # While the bot is not at the desired bin or not home
    while cur_bin < bin_num or (bin_num == -1 and not bot_home()):
        # Check if bot reads a nearby object. If it does, it might have
        # encountered a bin if it hasn't already. Add 1 to the bin count and
        # set the flag to True until the bin has been passed
        if bot.read_ultrasonic_sensor() < PROX_DIST and not encounter_bin:
            cur_bin += 1
            encounter_bin = True
        # Else if readings are far away then a bin has been passed, reset
        # flag to False to detect the next bin
        elif encounter_bin and bot.read_ultrasonic_sensor() > PASS_DIST:
            encounter_bin = False
        print('Bins encountered:', cur_bin)
        # Read the line sensor. In order to move towards the line, the opposite
        # wheel must spin faster (To turn left, right wheel spins faster, and vice
        # versa). This can mathematically be done by multiplying the desired speed
        # of the corresponding wheel with the opposite value in the reading. Using
        # list comprehensions, this can easily be set for speeds by getting the entries
        # from the back of the list using range and simple math and the fact that the
        # lists are a known fixed size of 2
        reading = bot.line_following_sensors()
        print("Readings:", reading)
        speeds = [mult * 0.025 * (1 + reading[1 - i]) for i in range(2)]
        # If line is lost, spin left since all turns are left and line is eventually found
        if speeds[0] == mult * 0.025 and speeds[1] == mult * 0.025:
            bot.stop()
            speeds[1] *= 2
        if speeds[0] != speeds[1]:
            bot.stop()
            for i in range(2):
                speeds[i] /= 4
            bot.set_wheel_speed(speeds)
            time.sleep(0.25)
            bot.stop()
        else:
            bot.set_wheel_speed(speeds)
    else:
        bot.stop()
        print("Bin found")
        bot.deactivate_ultrasonic_sensor()


def drop_container(angle):
    '''
    Tilts the hopper to dispense containers into the bin
    '''
    bot.activate_stepper_motor()
    bot.rotate_hopper(angle)
    time.sleep(3)
    bot.rotate_hopper(0)
    bot.deactivate_stepper_motor()


def main():
    '''
    Main function calls all necessary operations in order and also gets the
    destination bins
    '''
    # time.sleep(3)
    loc = dispension(2)
    print(loc)
    time.sleep(1)
    load_bottle()
    time.sleep(1)
    line_follow(loc)
    time.sleep(1)
    drop_container(60)
    line_follow()
    print(bot.position())
    


main()

#---------------------------------------------------------------------------------
# STUDENT CODE ENDS
#---------------------------------------------------------------------------------
