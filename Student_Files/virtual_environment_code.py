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

#========================= Bottle loading with Q-Arm =========================#

def load_bottle(bottle_count, offset=0):
    '''
    Function: load_bottle()

    Purpose: This function loads the current dispensed bottle onto the Q-Bot.

    Inputs: bottle_count - integer; offset - real number (default to 0)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/13
    '''
    # Constants for pick-up location for bottles and Q-Arm home location, and
    # gripper opening angle
    PICK_UP = (0.63, 0, 0.22)
    ARM_HOME = (0.406, 0, 0.483)
    GRIPPER_ANGLE = 40
    # Calculated drop-off position based on the offset and the bottle count
    # Offset is negative if the bot stopped past its home position, and
    # is positive if it stopped before. Positive is on the side of the
    # turntable. Correction of 0.025 to account for the centre x-component
    # being 0.025.
    drop_off = (0.025 + offset, -0.58 + (0.07 * bottle_count), 0.55)
    # Open/close grip tracker
    grip = 0
    # Create a list that orders the sequence of arm movement locations
    # assuming a start from home
    locations = [PICK_UP, ARM_HOME, drop_off]
    # Go to every movment location and wait 2 seconds
    # If the location index is even, it is a drop-off or pick-up spot
    # In that case, control the gripper to either close or open depending
    # on the grip tracker and wait 2 seconds again
    for i in range(len(locations)):
        arm.move_arm(*locations[i])
        time.sleep(2)
        # If the current iteration of the loop is an even number, the
        # gripper must either open or close to handle the container
        if (i % 2 == 0):
            # (-1) ^ n is 1 for even n and -1 for odd n
            # n in this case is grip, and when it is 0, 1 * 40 closes it
            # when it is 1, -1 * 40 opens it
            arm.control_gripper(pow(-1, grip) * GRIPPER_ANGLE)
            grip += 1
            time.sleep(2)
    # Move the arm away from the bottles and push them to line up better
    safe_return()
    adjust_bottles(drop_off[1], drop_off[0])


def safe_return():
    '''
    Function: safe_return()

    Purpose: This function moves the Q-Arm out of the way of the hopper
    safely.

    Inputs: None
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/13
    '''
    arm.rotate_elbow(-15)
    arm.rotate_shoulder(-15)
    time.sleep(2)
    arm.rotate_base(25)
    time.sleep(2)
    arm.home()


def adjust_bottles(end_pos, offset=0.025):
    '''
    Function: adjust_bottles()

    Purpose: This function pushes bottles currently on the hopper back to
    allow for more space for another bottle to be placed.

    Inputs: end_pos - real number; offset - real number (default to 0)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/13
    '''
    # Calculate push start and end location based on the given offsets and
    # ending positions
    start = (offset, -0.3, 0.45)
    drop = (offset, end_pos, 0.45)
    time.sleep(2)
    arm.control_gripper(45)
    time.sleep(2)
    locations = [start, drop]
    for i in range(2):
        arm.move_arm(*locations[i])
        time.sleep(2)
    # Slightly open the gripper to drop any friction-sticking bottles in the
    # simulation.
    arm.control_gripper(-15)
    # Move Q-Arm safely out of the way
    safe_return()

#========================= Q-Bot Movement and Positioning =========================#

def check_home(threshold=0.05):
    '''
    Function: check_home()

    Purpose: This function checks whether the Q-Bot is within an acceptable
    distance away from the home position.

    Inputs: threshold - real number (default 0.05)
    Outputs: real number

    Author: Mohammad Mahdi Mahboob, Liam Walker
    Last Update: 2022/02/07
    '''
    # Constant to track the Q-Bot's home (stopping) position
    BOT_HOME = (1.5, 0, 0)
    # Get the bot's current position
    pos = bot.position()
    # Check each coordinate of the home position against the correspinding
    # coordinate of the bot. If they are further than the threshold distance,
    # return -3000 to indicate it is not there. If all 3 positions are within
    # the threshold, then return the difference between the home position
    # y-coordinate and the bot's y-coordinate, as that is the most affected
    # dimension by movement along the line in this area.
    for i, j in zip(BOT_HOME, pos):
        if i - j > threshold:
            return -3000
    return BOT_HOME[1] - pos[1]


def bot_align(home=False):
    '''
    Function: bot_align()

    Purpose: This function aligns the bot line sensors to the line so that the
    hopper faces perpendicular to boxes or the Q-Arm.
    
    Inputs: home - Boolean (default to False)
    Outputs: None

    Author: Liam Walker, Mohammad Mahdi Mahboob
    Last Update: 2022/02/07
    '''
    # Read the value of the sensors until [1, 1] has been read twice to
    # Ensure it is moreso in the middle. If it has not been read twice,
    # slightly rotate the bot towards the line.
    readings = bot.line_following_sensors()
    # Keep count of the location of the wires. Initial calculation is done
    # using integer division with 2. This works because the only possible
    # options are [0, 0], [0, 1], [1, 0], [1, 1], of which only sums of
    # 0, 1, and 2 can be made. Integer division truncates the remainder, so
    # 0 // 2 and 1 // 2 both give 0, while 2 // 2 gives 1, which would
    # increment the counter whenever the sum is 2, which is when both sensors
    # read 1.
    count = (readings[0] + readings[1]) // 2
    # List to track the wheel speeds and update them
    speeds = [0, 0]
    while count < 2:
        # Calculate the speed of the wheels based on sensor readings.
        # If the left sensor reads high, the right wheel should spin faster.
        # If the right sensor reads high, the left wheel should spin faster.
        # Calculating using mathematics is slightly faster and better than
        # using if statements for this since there is less branching involved.
        # The [0, 0] is not accounted for because alignment is only successful
        # after the bot has followed the line for a while, and all stop
        # locations are after the bot moves straight for some time, so it is
        # highly probable that at least one sensor is detecting the line.
        speeds[0] = 0.025 + 0.025 * readings[1]
        speeds[1] = 0.025 + 0.025 * readings[0]
        # Rotate slightly, stop, then re-evaluate alignment and count
        bot.set_wheel_speed(speeds)
        time.sleep(0.1)
        bot.stop()
        readings = bot.line_following_sensors()
        # Increment the count if a [1, 1] reading is found.
        count += (readings[0] + readings[1]) // 2
    # If the bot is not home, move forward slightly to ensure it is dumping
    # inside a bin
    if not home:
        bot.forward_distance(0.05)


def line_follow(bin_num=-1):
    '''
    Function: line_follow()

    Purpose: This function moves the Q-Bot along the line until it reaches
    the given target location.

    Inputs: bin_num - integer (default -1)
    Outputs: None

    Author: Mohammad Mahdi Mahboob, Liam Walker
    Last Update: 2022/02/13
    '''
    # Turn on the ultrasonic sensor
    bot.activate_ultrasonic_sensor()
    # Constants for dictating minimum and maximum thresholds to determine
    # if the bot has passed a bin, and for the mimimum speed of the bot's
    # wheels
    PROX_DIST = 0.1
    PASS_DIST = 0.2
    SPEED = 0.05
    # tracks the number of bins passed
    cur_bin = 0
    # lists to hold line sensor readings and wheel speeds
    reading, speeds = [], [0, 0]
    # Flag to check if currently near a bin
    encounter_bin = False
    print('Target:', bin_num)
    # While the bot is not at the desired bin or not home
    while cur_bin < bin_num or (bin_num == -1 and check_home(0.025) == -3000):
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
        # Output to check if bin detection is correct
        print('Bins encountered:', cur_bin)
        # Calculate the speed of the bot's wheels. Similar to the method
        # dictated in the bot_align() function. Only difference is that a
        # reading of [0, 0] can occur when attempting to turn left. As such,
        # the bot should reflect this bias towards the left on a [0, 0]
        # reading. Speed is calculated by multiplying the SPEED value with
        # a multiplier determined by the possible conditions.
        reading = bot.line_following_sensors()
        print("Readings:", reading)
        # Want something that maps readings to speeds along this scheme
        # Reading | Speed Multipliers
        # [0, 0] -> [1, 2]
        # [0, 1] -> [2, 1]
        # [1, 0] -> [1, 2]
        # [1, 1] -> [2, 2]
        # but through some sort of mathematical calculation. The bot_align()
        # function's calculation already maps the left wheel perfectly, but
        # the same logic would not account for the [0, 0] bias. Instead, set
        # the default for the right wheel to be double the minimum speed,
        # and subtract 1 for the [1, 0] case. This can be done by defaulting
        # the right wheel to twice the speed and only slowing it down in the
        # [1, 0] case.
        speeds[0] = SPEED * (1 + reading[1])
        # The nested boolean is only True in the case of [1, 0], and that
        # True gets converted into the integer value of 1 and subtracting
        # from 2 gives 1, which is the desired speed of the wheel. For the
        # rest of the cases, the boolean evaluates to False (0), and the
        # multiplier is 2. This calculation reduces branching caused by if
        # statements, allowing for faster calculations and thereby faster
        # repetitions on average. This is also shorter to write.
        speeds[1] = SPEED * (2 - (reading[1] > reading[0]))
        # Move a little bit with the current speeds. This is done to ensure
        # that the bot can turn if necessary.
        bot.set_wheel_speed(speeds)
        time.sleep(0.25)
        # Recalculate speeds to stay the same if they are both maximum speed
        # (i.e. going forwards) or 0 if they were turning. This product gives
        # 0 for the sum of reading being 0 or 1 and 1 for 2 by exploiting
        # integer division.
        speeds[0] = speeds[1] = SPEED * 2 * ((reading[1] + reading[0]) // 2)
        bot.set_wheel_speed(speeds)
    # Once a stopping condition has been correctly met, stop the bot and
    # align it according to the location it has stopped.
    else:
        bot.stop()
        bot_align(bin_num == -1)
        print("Bin/stop location found")
        bot.deactivate_ultrasonic_sensor()


def drop_container(angle):
    '''
    Function: drop_container()

    Purpose: This function rotates the actuator to tilt the hopper and
    drop the containers inside of the bin.

    Inputs: angle - real number
    Outputs: None

    Author: Liam Walker
    Last Update: 2022/02/13
    '''
    # Activate and deactivate the motor
    bot.activate_stepper_motor()
    # Keep track of the tilt angle of the hopper to control its speed
    tilt = 0
    # To slow down the hopper's speed, tilt it part way and pause for a bit before
    # tilting it again
    while tilt < angle:
        tilt += angle / 2
        bot.rotate_hopper(tilt)
        time.sleep(0.15)
    time.sleep(1.5)
    # Revert hopper back to original position and deactivate motor
    bot.rotate_hopper(0)
    bot.deactivate_stepper_motor()

#========================= Container Categorizing and Management =========================#

def extract_container_info(bottle_id):
    '''
    Function: extract_container_info()

    Purpose: This function dispenses a container onto the turntable and
    gets all necessary information from it.

    Inputs: bottle_id - integer
    Outputs: 2-tuple of integers

    Author: Alvin Qian
    Last Update: 2022/01/31
    '''
    # Spawn the next bottle and get its information. Note that the 3rd item given
    # from the function above is a string formatted as 'bin##' where ## is a
    # number which can be extracted using substring and parsed as int. Do this and
    # return a pair of integers with mass first and destination bin second.
    info = table.dispense_container(bottle_id, True)
    return info[1], int(info[2][3:])


def dispense_containers(bottles, has_bottle=False, bottle_info=None, offset=0):
    '''
    Function: dispense_containers()

    Purpose: This function takes the list of remaining bottles and spawns
    them onto the turntable, and decides whether or not it can be picked up
    at the current turn.

    Inputs: bottles - list of integers; has_bottle - Boolean (default to
    False); bottle_info - 2-integer tuple (default to empty); offset -
    real number (default to 0)
    Outputs: 2-item tuple - Boolean first, integer tuple second.

    Author: Alvin Qian
    Last Update: 2022/02/13
    '''
    # Quickly sets tracker variables to 0
    # Track the number of bottles, the total mass and the destination bins
    # with counters set to 0
    bottle_count, total_mass, dest_bin = [0] * 3
    # If a botlle does not exist on the table right now, spawn one and use its
    # information and baing them
    if not has_bottle:
        # bottles.pop() gets and removes the last item in the randomized bottles list
        bottle_info = extract_container_info(bottles.pop())
    # Add the total mass of the bottle to total_mass and set the destination
    # to that of the bottle.
    total_mass += bottle_info[0]
    main_dest = bottle_info[1]
    # Also keep track of the location to place the bottle(s)
    load_bottle(bottle_count, offset)
    bottle_count += 1
    # While either there are less than 3 bottles or other conditions such
    # as the list being non-empty
    while total_mass < 90 and bottle_count < 3 and bottles:
        # For debugging
        print("Next bottle ID:", bottles[-1])
        # Remove the last item from the list and then multiply by 2
        bottle_info = extract_container_info(bottles.pop())
        print(bottle_info)
        # If the current total mass and the current container mass are below
        # 90 for the hopper wieght limit cutoff, and the destination of the
        # current spawned container matches.
        if bottle_info[0] + total_mass < 90 and bottle_info[1] == main_dest:
            load_bottle(bottle_count, offset)
            bottle_count += 1
        # Otherwise if the mass is too much or there is a mismatch of bins,
        # return True to indicate there is a container still on the table,
        # followed by the destination of the loaded containers and the
        # information of the most recently dropped bottle.
        else:
            # For debugging
            print("Bin #:", main_dest, "\nTotal mass:", total_mass, "\n# of bottles:", bottle_count)
            return True, main_dest, bottle_info
    # Return False to indicate there is no containers on the table, followed
    # by the destination of the loaded containers and the information of the
    # most recently dropped bottle.
    return False, main_dest, bottle_info


def random_container_list(size=18):
    '''
    Function: random_container_list()

    Purpose: This function creates a randomly shuffled order of bottles as
    a list. The list is shuffled with 3 copies of each bottle.

    Inputs: size - integer (default to 18)
    Outputs: Randomized list of integers

    Author: Alvin Qian, Mohammad Mahdi Mahboob
    Last Update: 2022/01/31
    '''
    # Create the list and add 3 copies of each bottle type
    bottles_order = []
    for i in range(3):
        for j in range(6):
            bottles_order.append(j + 1)
    # Shuffle the list, then reverse it and return the reversed order so that
    # users can then pop from the back of the list in the original order. For
    # sublists, this returns the last n elements in the reversed list, where
    # n = size.
    random.shuffle(bottles_order)
    # For debugging, print the sublist of elements that will be returned
    print('Order of bottle dispensions: ', bottles_order[:size])
    bottles_order.reverse()
    return bottles_order[-size:]

#========================= Main Function =========================#

def main():
    '''
    Function: main()

    Purpose: Main function, calls all other functions and maintains program's
    general workflow.

    Inputs: None
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/13
    '''
    bottles = random_container_list(5)
    # Holds information about the bottle type for reloading. Information
    # pertains to mass at [0] and destination at [1]. Useful for loading
    # any existing bottles on the table that were not loaded previously.
    bottle_info = ()
    has_bottle = False
    # Keeps track of the offset of the Q-Bot from original home position
    offset = 0
    # While there are still bottles left in the list or the table, run cycles
    while bottles or has_bottle:
        # Dispense and load bottles as needed. Check for any remaining bottles
        # on the table as well as get the destination bin number for dumping
        # and information on remaining bottles. Load the containers after they
        # spawn and move them to the corresponding bin. Dump the containers,
        # then return home.
        has_bottle, bin_num, bottle_info = dispense_containers(
            bottles, has_bottle, bottle_info, offset)
        line_follow(bin_num)
        time.sleep(1)
        drop_container(50)
        time.sleep(1)
        line_follow()
        # Update the offset
        offset = check_home(0.025)
        print(offset)

main()


#---------------------------------------------------------------------------------
# STUDENT CODE ENDS
#---------------------------------------------------------------------------------
