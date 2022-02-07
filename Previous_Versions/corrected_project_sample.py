import sys
import time
import random
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
def bot_align(home=False):
    readings = bot.line_following_sensors()
    while readings[0] + readings[1] != 2:
        speeds = [0.025 + 0.025 * readings[1], 0.025 + 0.025 * readings[0]]
        bot.set_wheel_speed(speeds)
        time.sleep(0.1)
        bot.stop()
        readings = bot.line_following_sensors()
    while home and not check_home(0.025):
            bot.forward_distance(0.01)



def load_bottle(bottle_count, bottle_id):
    '''
    Position the Qbot near the Qarm and loads a bottle onto the Qbot using Qarm
    '''
    # Constants to keep track of positions for pickup, home, and drop
    # DROP_OFF changes depending on the value of bottle_count to move
    # for multiple bottles
    DROP_OFF = (0, -0.58 + (0.05 + 0.02 * (not(bottle_id == 2 or bottle_id == 5))) * bottle_count, 0.55)
    PICK_UP = (0.63, 0, 0.22)
    ARM_HOME = (0.406, 0, 0.483)
    # Open/close grip tracker
    grip = 0
    # Create a list that orders the sequence of arm movement locations
    # assuming a start from home
    locations = [PICK_UP, ARM_HOME, DROP_OFF]
    # Go to every movment location and wait 2 seconds
    # If the location index is even, it is a drop-off or pick-up spot
    # In that case, control the gripper to either close or open depending
    # on the grip tracker and wait 2 seconds again
    for i in range(len(locations)):
        arm.move_arm(*locations[i])
        time.sleep(2)
        if (i % 2 == 0):
            # (-1) ^ n is 1 for even n and -1 for odd n
            # n in this case is grip, and when it is 0, 1 * 40 closes it
            # when it is 1, -1 * 40 opens it
            arm.control_gripper(pow(-1, grip) * 40)
            grip += 1
            time.sleep(2)
    # Move the arm away from the bottles and push them to line up better
    safe_return()
    adjust_bottles(DROP_OFF[1])


def safe_return():
    '''
    Moves the Q-arm out of the way safely
    '''
    arm.rotate_elbow(-15)
    time.sleep(2)
    arm.rotate_base(25)
    time.sleep(2)
    arm.home()


def adjust_bottles(end_pos):
    '''
    Adjusts the positions of the bottles to allow for dropping of more items
    '''
    START = (0, -0.3, 0.45)
    DROP = (0, end_pos, 0.45)
    time.sleep(2)
    arm.control_gripper(45)
    time.sleep(2)
    arm.move_arm(*START)
    time.sleep(2)
    arm.move_arm(*DROP)
    time.sleep(2)
    arm.control_gripper(-15)
    safe_return()


def extract_container_info(bottle_id):
    '''
    Dispenses a container and gets its mass and bin destination as a pair of integers
    '''
    info = table.dispense_container(bottle_id, True)
    # Note that the 3rd item given from the function above is a string formatted as 'bin##'
    # where ## is a number which we can extract using substring and return as int
    return info[1], int(info[2][3:])


def check_home(threshold=0.05):
    '''
    Determines if the bot is near its home position. Returns True for a
    difference <= threshold (m) before each coordinate and False otherwise.
    '''
    BOT_HOME = (1.5, 0, 0)
    pos = bot.position()
    for i, j in zip(BOT_HOME, pos):
        if i - j > threshold:
            return False
    return True


def line_follow(bin_num=-1):
    '''
    Makes the Q-bot follow the line until it reaches the intended bin number, or
    if the bin number is -1, then follows the line home
    '''
    bot.activate_ultrasonic_sensor()
    mult = 4
    PROX_DIST = 0.09
    PASS_DIST = 0.2
    # if check_home():
      #  bot.forward_distance(0.25) # move away from home position to run
    cur_bin = 0 # tracks the number of bins passed
    reading, speeds = [], [] # lists to hold line sensor readings and wheel speeds
    encounter_bin = False # Flag to check if currently near a bin
    print('Target:', bin_num)
    # While the bot is not at the desired bin or not home
    while cur_bin < bin_num or (bin_num == -1 and not check_home()):
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
            speeds = [0, 0.1]
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
        bot_align(bin_num == -1)
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


def dispense_containers(bottles, has_bottle=False, bottle_info=None):
    # Default values remain for testing purposes
    # Quickly sets tracker variables to 0
    bottle_count, total_mass, dest_bin = [0] * 3
    if not has_bottle:
        # bottles.pop() gets and removes the last item in the randomized bottles list
        bottle_info = extract_container_info(bottles.pop())
    total_mass += bottle_info[0]
    main_dest = bottle_info[1]
    load_bottle(bottle_count, main_dest)
    bottle_count += 1
    while bottle_count < 3 and bottles:
        bottle_info = extract_container_info(bottles.pop())
        if bottle_info[0] + total_mass < 90 and bottle_info[1] == main_dest:
            load_bottle(bottle_count, main_dest)
            bottle_count += 1
        else:
            return True, main_dest, bottle_info
    return False, main_dest, bottle_info


def random_container_list():
    bottles_order = []
    for i in range(3):
        for j in range(6):
            bottles_order.append(j + 1)
    random.shuffle(bottles_order)
    return bottles_order
        

def main():
    '''
    Main function calls all necessary operations in order and also gets the
    destination bins
    '''
    # time.sleep(3)
    bottles = random_container_list()
    # Holds information about the bottle type for reloading
    # Information pertains to mass at [0] and destination at [1]
    # Useful for loading any existing bottles on the table that
    # were not loaded previously
    bottle_info = ()
    has_bottle = False
    # While there are still bottles left in the list or the table, loop
    # sorting process
    while bottles or has_bottle:
        # Dispense and load bottles as needed. Check for any remaining bottles
        # on the table as well as get the destination bin number for dumping
        # and information on remaining bottles
        has_bottle, bin_num, bottle_info = dispense_containers(bottles, has_bottle, bottle_info)
        line_follow(bin_num)
        time.sleep(1)
        drop_container(60)
        time.sleep(1)
        line_follow()
        bot_align(True)


main()

#---------------------------------------------------------------------------------
# STUDENT CODE ENDS
#---------------------------------------------------------------------------------
