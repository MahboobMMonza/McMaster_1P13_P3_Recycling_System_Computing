import sys
from time import sleep
from random import shuffle

sys.path.append('../')

from Common.project_library import *

# Modify the information below according to your setup and uncomment the entire section

# 1. Interface Configuration
project_identifier = 'P3B'  # enter a string corresponding to P0, P2A, P2A, P3A, or P3B
ip_address = '169.254.134.253'  # enter your computer's IP address
hardware = False  # True when working with hardware. False when working in the simulation

# 2. Servo Table configuration
short_tower_angle = 270  # enter the value in degrees for the identification tower
tall_tower_angle = 0  # enter the value in degrees for the classification tower
drop_tube_angle = 180  # enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# 3. Qbot Configuration
bot_camera_angle = -21.5  # angle in degrees between -21.5 and 0

# 4. Bin Configuration
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin

bin1_offset = 0.17  # offset in meters
bin1_color = [1, 0, 0]  # e.g. [1,0,0] for red
bin2_offset = 0.17
bin2_color = [0, 1, 0]
bin3_offset = 0.17
bin3_color = [0, 0, 1]
bin4_offset = 0.17
bin4_color = [0, 0, 0]

# --------------- DO NOT modify the information below -----------------------------

if project_identifier == 'P0':
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    bot = qbot(0.1, ip_address, QLabs, None, hardware)

elif project_identifier in ["P2A", "P2B"]:
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    arm = qarm(project_identifier, ip_address, QLabs, hardware)

elif project_identifier == 'P3A':
    table_configuration = [short_tower_angle, tall_tower_angle, drop_tube_angle]
    configuration_information = [table_configuration, None, None]  # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware, configuration_information).QLabs
    table = servo_table(ip_address, QLabs, table_configuration, hardware)
    arm = qarm(project_identifier, ip_address, QLabs, hardware)

elif project_identifier == 'P3B':
    table_configuration = [short_tower_angle, tall_tower_angle, drop_tube_angle]
    qbot_configuration = [bot_camera_angle]
    bin_configuration = [[bin1_offset, bin2_offset, bin3_offset, bin4_offset],
                         [bin1_color, bin2_color, bin3_color, bin4_color]]
    configuration_information = [table_configuration, qbot_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware, configuration_information).QLabs
    table = servo_table(ip_address, QLabs, table_configuration, hardware)
    arm = qarm(project_identifier, ip_address, QLabs, hardware)
    bins = bins(bin_configuration)
    bot = qbot(0.1, ip_address, QLabs, bins, hardware)


# ---------------------------------------------------------------------------------
# STUDENT CODE BEGINS
# ---------------------------------------------------------------------------------

# ========================= Bottle Loading with Q-Arm ========================= #


def load_bottle(bottle_count, offset=0):
    """
    Function: load_bottle()

    Purpose: This function loads the current dispensed bottle onto the Q-Bot.

    Inputs: bottle_count - integer; offset - real number (default to 0)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/17
    """
    # Constants for pick-up location for bottles and Q-Arm home location, and
    # gripper opening angle
    PICK_UP = (0.63, 0, 0.22)
    ARM_HOME = (0.406, 0, 0.483)
    GRIPPER_ANGLE = 40
    # Calculated drop-off position based on the offset and the bottle count,
    # with default bot home position correction applied
    drop_off = (0.02 + offset, -0.58 + (0.07 * bottle_count), 0.55)
    # Open/close grip tracker
    grip = 0
    # Create a list that orders the sequence of arm movement locations
    # assuming a start from home
    locations = [PICK_UP, ARM_HOME, drop_off]
    # Go to every movement location and wait 2 seconds. If the location
    # index is even, it is a drop-off or pick-up spot, so move gripper
    # accordingly. Afterwards, push bottles inwards to make space for
    # more bottles.
    for i in range(len(locations)):
        arm.move_arm(*locations[i])
        sleep(2)
        if i % 2 == 0:
            arm.control_gripper(pow(-1, grip) * GRIPPER_ANGLE)
            grip += 1
            sleep(2)
    safe_return()
    adjust_bottles(drop_off[1], drop_off[0])


def safe_return():
    """
    Function: safe_return()

    Purpose: This function moves the Q-Arm out of the way of the hopper
    safely.

    Inputs: None
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/15
    """
    # Rotate the arm upwards fully to avoid bottles, then rotate it
    # away from the direction of the hopper before going back to
    # home position.
    arm.rotate_elbow(-15)
    arm.rotate_shoulder(-15)
    sleep(2)
    arm.rotate_base(25)
    sleep(2)
    arm.home()


def adjust_bottles(end_pos, offset=0.025):
    """
    Function: adjust_bottles()

    Purpose: This function pushes bottles currently on the hopper back to
    allow for more space for another bottle to be placed.

    Inputs: end_pos - real number; offset - real number (default to 0)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/17
    """
    # Calculate push start and end location based on the given offsets and
    # ending positions, and put them in a list.
    start = (offset, -0.3, 0.45)
    drop = (offset, end_pos, 0.45)
    locations = [start, drop]
    # Close the gripper fully to fit inside the hopper. Go to the start
    # position, and then go to the drop location to push the bottles
    # inwards of the hopper. To prevent any accidental grips in the virtual
    # environment, open the arm slightly. Move the arm away from the hopper
    # and reset the arm to its home position.
    sleep(2)
    arm.control_gripper(45)
    sleep(2)
    for i in range(2):
        arm.move_arm(*locations[i])
        sleep(2)
    arm.control_gripper(-15)
    sleep(1)
    safe_return()


# ========================= Q-Bot Movement and Positioning ========================= #


def check_home(threshold=0.05):
    """
    Function: check_home()

    Purpose: This function checks whether the Q-Bot is within an acceptable
    distance away from the home position.

    Inputs: threshold - real number (default 0.05)
    Outputs: real number

    Author: Mohammad Mahdi Mahboob, Liam Walker
    Last Update: 2022/02/07
    """
    # Constant to track the Q-Bot's home (stopping) position
    BOT_HOME = (1.5, 0, 0)
    # Get the bot's current position
    pos = bot.position()
    # Check each coordinate of home against the coordinates of the
    # bot, and return a value indicating its offset from home.
    for i, j in zip(BOT_HOME, pos):
        if i - j > threshold:
            return -3000
    return BOT_HOME[1] - pos[1]

def fine_rotate(speed, duration=0.1):
    """
    Function: fine_rotate()

    Purpose: This function rotates the bot in a slight increment at specified
    wheel speeds to allow for more precise control.

    Inputs: speed - 2-integer list; duration - real number (default 0.1)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/17
    """
    # Slightly rotate the bot at the specified speed and duration
    bot.set_wheel_speed(speed)
    time.sleep(duration)
    bot.stop()

def bot_align(home=False):
    """
    Function: bot_align()

    Purpose: This function aligns the bot line sensors to the line so that the
    hopper faces perpendicular to boxes or the Q-Arm.

    Inputs: home - Boolean (default to False)
    Outputs: None

    Author: Liam Walker, Mohammad Mahdi Mahboob
    Last Update: 2022/02/17
    """
    # Align the bot by turning the bot until [1, 1] has been read.
    readings = bot.line_following_sensors()
    print(home)
    # Wheel speed settings
    speeds = [[0.025, -0.025], [-0.025, 0.025]]
    # Step tracker
    steps = 0
    # Rotate bot right to the edge of the line, in small steps,
    # then count the steps taken turning left to the other
    # edge, then turn back right half-way.
    while readings[0] != 1:
        fine_rotate(speeds[0])
        bot.stop()
        readings = bot.line_following_sensors()
    for i in range(2):
        while readings[(i + 1) % 2] != 0:
            fine_rotate(speeds[i])
            steps += i
            readings = bot.line_following_sensors()
    for i in range(steps // 2 + 1):
        fine_rotate(speeds[0])
    # If the bot is not home, move forward slightly to ensure it is dumping
    # inside a bin
    if not home:
        bot.forward_distance(0.1)


def line_follow(bin_num=-1):
    """
    Function: line_follow()

    Purpose: This function moves the Q-Bot along the line until it reaches
    the given target location.

    Inputs: bin_num - integer (default -1)
    Outputs: None

    Author: Mohammad Mahdi Mahboob, Liam Walker
    Last Update: 2022/02/17
    """
    # Turn on the ultrasonic sensor
    bot.activate_ultrasonic_sensor()
    # Constants for dictating minimum and maximum thresholds to determine
    # if the bot has passed a bin, and to control the speed of the bot
    PROX_DIST = 0.1
    PASS_DIST = 0.2
    SPEED = 0.025
    # Tracks the number of bins passed
    cur_bin = 0
    # Tracks wheel speeds
    speeds = [0, 0]
    # Flag to check if currently near a bin
    encounter_bin = False
    print('Target:', bin_num)
    # Follow the line until enough bins have been encountered or until the
    # bot has returned home.
    while cur_bin < bin_num or (bin_num == -1 and check_home() == -3000):
        # Check if bot has recently encountered a bin and update accordingly
        if bot.read_ultrasonic_sensor() < PROX_DIST and not encounter_bin:
            cur_bin += 1
            encounter_bin = True
        # If the bot hasn't encountered a bin recently, check if it has
        # recently passed a bin and update accordingly
        elif encounter_bin and bot.read_ultrasonic_sensor() > PASS_DIST:
            encounter_bin = False
        print('Bins encountered:', cur_bin)
        # Calculate the speed of the bot's wheels based on line sensor
        # readings
        reading = bot.line_following_sensors()
        print("Readings:", reading)
        # Want something that maps readings to speeds along this scheme
        # Reading | Speed Multipliers
        # [0, 0] -> [1, 2]
        # [0, 1] -> [2, 1]
        # [1, 0] -> [1, 2]
        # [1, 1] -> [2, 2]
        # but through some sort of mathematical calculation to reduce
        # branching, which sets speeds quicker and the bot follows
        # the line faster.
        speeds[0] = SPEED * (1 + reading[1])
        speeds[1] = SPEED * (2 - (reading[1] > reading[0]))
        # Set speeds to calculated values for a short while before
        # stopping the bot if the wheel speeds don't match (i.e. it
        # is turning).
        bot.set_wheel_speed(speeds)
        sleep(0.25)
        speeds[0] = speeds[1] = SPEED * 2 * ((reading[1] + reading[0]) // 2)
        bot.set_wheel_speed(speeds)
    # Once a stopping condition has been correctly met, stop the bot and
    # align it, then deactivate the ultrasonic sensor
    else:
        bot.stop()
        bot_align(bin_num == -1)
        print("Bin/stop location found")
        bot.deactivate_ultrasonic_sensor()


def drop_bottle(angle, slow_bin=True):
    """
    Function: drop_bottle()

    Purpose: This function rotates the actuator to tilt the hopper and
    drop the bottles inside the bin.

    Inputs: angle - real number; slow_bin - Boolean (default to False)
    Outputs: None

    Author: Liam Walker
    Last Update: 2022/02/17
    """
    # Activate motor before use
    sleep(1)
    bot.activate_linear_actuator()
    # Keep track of the tilt angle of the hopper to control its speed,
    # and calculate the number of steps required for the rate based
    # on slow_bin. Full speed for metal cans, 1/2 speed otherwise.
    tilt = 0
    print(slow_bin)
    step_size = angle / (slow_bin + 1)
    # Tilt the hopper until the given angle and then wait 3 seconds
    # to deposit all bottles safely, before resetting the hopper
    # and deactivating the motor.
    while tilt < angle:
        tilt += step_size
        bot.rotate_hopper(tilt)
        sleep(0.15)
    sleep(3)
    bot.rotate_hopper(0)
    bot.deactivate_linear_actuator()


# ========================= Bottle Categorizing and Management ========================= #


def extract_bottle_info(bottle_id):
    """
    Function: extract_bottle_info()

    Purpose: This function dispenses a bottle onto the turntable and gets all
    necessary information from it.

    Inputs: bottle_id - integer
    Outputs: 2-tuple of integers

    Author: Alvin Qian
    Last Update: 2022/01/31
    """
    # Spawn the next bottle and get its information. Note that the 3rd item
    # given from the function above is a string formatted as 'bin##' where
    # ## is a number which can be extracted using substring and parsed as
    # int. Do this and return a pair of integers with mass first and
    # destination bin second.
    info = table.dispense_container(bottle_id, True)
    return info[1], int(info[2][3:])


def dispense_bottles(bottles, offset=0, has_bottle=False, bottle_info=None):
    """
    Function: dispense_bottles()

    Purpose: This function takes the list of remaining bottles and spawns
    them onto the turntable, and decides whether it can be picked up
    at the current turn.

    Inputs: bottles - list of integers; has_bottle - Boolean (default to
    False); bottle_info - integer 2-tuple (default to empty);
    offset - real number (default to 0)
    Outputs: 2-item tuple - Boolean first, integer 2-tuple second.

    Author: Alvin Qian
    Last Update: 2022/02/13
    """
    # Track the number of bottles, the total mass, and destination bins,
    # setting all counters to 0
    bottle_count, total_mass, dest_bin = [0] * 3
    # If a bottle does not exist on the table right now, spawn one and use its
    # information
    if not has_bottle:
        print("Next bottle ID:", bottles[-1])
        bottle_info = extract_bottle_info(bottles.pop())
        print(bottle_info)
    # Add the mass of the bottle to total_mass and set the destination to the
    # first bottle's bin. Track the count, load the bottle and prepare to
    # track any bottles left on the turntable.
    total_mass += bottle_info[0]
    main_dest = bottle_info[1]
    load_bottle(bottle_count, offset)
    bottle_count += 1
    has_bottle = False
    # Spawn and load up to 3 bottles if the total mass of all bottles is
    # below 90 gram and the destinations are the same. If not, indicate
    # that a bottle remains on the table and break from the loop.
    while bottle_count < 3 and bottles:
        print("Next bottle ID:", bottles[-1])
        bottle_info = extract_bottle_info(bottles.pop())
        print(bottle_info)
        if bottle_info[0] + total_mass < 90 and bottle_info[1] == main_dest:
            load_bottle(bottle_count, offset)
            bottle_count += 1
        else:
            has_bottle = True
            break
    # Return all necessary information regarding destination and bottles
    # left on the table
    print("Bin #:", main_dest, "\nTotal mass:",
          total_mass, "\n# of bottles:", bottle_count)
    return has_bottle, main_dest, bottle_info


def random_bottle_list(size=18):
    """
    Function: random_bottle_list()

    Purpose: This function creates a randomly shuffled order of bottles
    as a list. The list is shuffled with 3 copies of each bottle.

    Inputs: size - integer (default to 18)
    Outputs: Randomized list of integers

    Author: Alvin Qian, Mohammad Mahdi Mahboob
    Last Update: 2022/02/13
    """
    # Create the list and add 3 copies of each bottle type
    bottles_order = []
    for i in range(3):
        for j in range(6):
            bottles_order.append(j + 1)
    # Shuffle the list, then reverse it and return the reversed order so that
    # users can then pop from the back of the list in the original order. For
    # sub-lists, this returns the last n elements in the reversed list, where
    # n = size.
    shuffle(bottles_order)
    print('Order of bottle dispensing: ', bottles_order[:size])
    bottles_order.reverse()
    return bottles_order[-size:]


# ============================ Main Function ============================ #


def main():
    """
    Function: main()

    Purpose: Main function, calls all other functions and maintains
    program's general workflow.

    Inputs: None
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/17
    """
    # Create variables to track any bottles left on the table during
    # loading
    bottle_info = ()
    has_bottle = False
    # Keeps track of the offset of the Q-Bot from original home position
    offset = 0
    # Generate the random order of bottles. Remove size input to run full
    # length program with 3 occurrences of all 6 bottle types.
    bottles = random_bottle_list(5)
    # While there are still bottles left in the list or the table, run
    # cycle: dispense bottles and load them to hopper, tracking any that
    # were left behind; transport them to the required bin, deposit them
    # and return home; account for any error caused in the return.
    while bottles or has_bottle:
        has_bottle, bin_num, bottle_info = dispense_bottles(
            bottles, offset, has_bottle, bottle_info)
        line_follow(bin_num)
        drop_bottle(45, bin_num != 1)
        line_follow()
        offset = check_home()
        print(offset)


main()

# ---------------------------------------------------------------------------------
# STUDENT CODE ENDS
# ---------------------------------------------------------------------------------
