import sys
import time
import random
from Common.project_library import *

sys.path.append('../')

# Modify the information below according to you setup and uncomment the entire section

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

# ========================= Bottle loading with Q-Arm ========================= #


def load_bottle(bottle_count, offset=0):
    """
    Function: load_bottle()

    Purpose: This function loads the current dispensed bottle onto the Q-Bot.

    Inputs: bottle_count - integer; offset - real number (default to 0)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/15
    """
    # Constants for pick-up location for bottles and Q-Arm home location, and
    # gripper opening angle
    PICK_UP = (0.63, 0, 0.22)
    ARM_HOME = (0.406, 0, 0.483)
    GRIPPER_ANGLE = 40
    # Calculated drop-off position based on the offset and the bottle count
    # Offset is negative if the bot stopped past its home position, and
    # is positive if it stopped before. A correction of 0.025 is applied
    # for the hopper's initial position during a run.
    drop_off = (0.025 + offset, -0.58 + (0.07 * bottle_count), 0.55)
    # Open/close grip tracker
    grip = 0
    # Create a list that orders the sequence of arm movement locations
    # assuming a start from home
    locations = [PICK_UP, ARM_HOME, drop_off]
    # Go to every movement location and wait 2 seconds. If the location
    # index is even, it is a drop-off or pick-up spot In that case,
    # control the gripper to either close or open depending on the grip
    # tracker and wait 2 seconds again. Pick-up and drop-off spots have
    # even indexes in locations. Once loading is done, move arm safely
    # away and push bottles to make room for any other bottles that may
    # be added.
    for i in range(len(locations)):
        arm.move_arm(*locations[i])
        time.sleep(2)
        if i % 2 == 0:
            # (-1) ^ grip is 1 for even grip values and -1 for odd grip
            # values. When grip is 0, 1 * 40 closes it, and when it is 1,
            # -1 * 40 opens it.
            arm.control_gripper(pow(-1, grip) * GRIPPER_ANGLE)
            grip += 1
            time.sleep(2)
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
    time.sleep(2)
    arm.rotate_base(25)
    time.sleep(2)
    arm.home()


def adjust_bottles(end_pos, offset=0.025):
    """
    Function: adjust_bottles()

    Purpose: This function pushes bottles currently on the hopper back to
    allow for more space for another bottle to be placed.

    Inputs: end_pos - real number; offset - real number (default to 0)
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/15
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
    time.sleep(2)
    arm.control_gripper(45)
    time.sleep(2)
    for i in range(2):
        arm.move_arm(*locations[i])
        time.sleep(2)
    arm.control_gripper(-15)
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
    # Check each coordinate of the home position against the corresponding
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
    """
    Function: bot_align()

    Purpose: This function aligns the bot line sensors to the line so that the
    hopper faces perpendicular to boxes or the Q-Arm.

    Inputs: home - Boolean (default to False)
    Outputs: None

    Author: Liam Walker, Mohammad Mahdi Mahboob
    Last Update: 2022/02/07
    """
    # Read the value of the sensors until [1, 1] has been read twice to
    # Ensure it is more so in the middle. If it has not been read twice,
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
    # Align the bot until 2 readings of [1, 1] are encountered.
    while count < 2:
        # Calculate the speed of the wheels based on sensor readings.
        # When the left sensor reads high, the right wheel should spin faster.
        # When the right sensor reads high, the left wheel should spin faster.
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
        count += (readings[0] + readings[1]) // 2
    # If the bot is not home, move forward slightly to ensure it is dumping
    # inside a bin
    if not home:
        bot.forward_distance(0.05)


def line_follow(bin_num=-1):
    """
    Function: line_follow()

    Purpose: This function moves the Q-Bot along the line until it reaches
    the given target location.

    Inputs: bin_num - integer (default -1)
    Outputs: None

    Author: Mohammad Mahdi Mahboob, Liam Walker
    Last Update: 2022/02/13
    """
    # Turn on the ultrasonic sensor
    bot.activate_ultrasonic_sensor()
    # Constants for dictating minimum and maximum thresholds to determine
    # if the bot has passed a bin, and to control the speed of the bot
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
    # While the bot is not at the desired bin or not home, follow the line
    # and check to see if bins have been passed. A bin has been encountered
    # if the distance just recently dropped below the minimum proximity
    # distance, and it has been passed if the distance just recently
    # exceeded the minimum passing distance. During any case, an update
    # occurs only if one or the other of the two scenarios occur, so check
    # for either scenario using elif and update values accordingly.
    while cur_bin < bin_num or (bin_num == -1 and check_home(0.025) == -3000):
        # Check if bot has recently encountered a bin and update accordingly
        if bot.read_ultrasonic_sensor() < PROX_DIST and not encounter_bin:
            cur_bin += 1
            encounter_bin = True
        # If the bot hasn't encountered a bin recently, check if it has
        # recently passed a bin and update accordingly
        elif encounter_bin and bot.read_ultrasonic_sensor() > PASS_DIST:
            encounter_bin = False
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
        # Once calculations are complete, set the wheel speeds to the
        # calculated values and move for a short while before determining
        # if bot wheels have the same speed (i.e. [1, 1] was read) in a
        # calculation. The calculation yields [0, 0] for all other values.
        # Set the wheel speeds to the new calculated values, which stops
        # the bot if it is turning and keeps it going if it is following
        # a line completely.
        bot.set_wheel_speed(speeds)
        time.sleep(0.25)
        # This product gives 0 for the sum of reading being 0 or 1 and 1
        # for 2 by exploiting integer division.
        speeds[0] = speeds[1] = SPEED * 2 * ((reading[1] + reading[0]) // 2)
        bot.set_wheel_speed(speeds)
    # Once a stopping condition has been correctly met, stop the bot and
    # align it according to the location it has stopped, then deactivate
    # the sensors.
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
    Last Update: 2022/02/13
    """
    # Activate motor before use
    bot.activate_stepper_motor()
    # Keep track of the tilt angle of the hopper to control its speed,
    # and calculate the number of steps required for the rate based
    # on slow_bin. This allows the bot to tilt faster when depositing
    # metal cans and tilt slower when depositing other bottles.
    # The step size calculation is set to 1/2 the angle if the
    # slow_bin is True, since True evaluates to 1, and 1 + 1 = 2.
    # When slow_bin is False, it evaluates to 0, and the step size
    # is just the angle itself.
    tilt = 0
    step_size = angle / (slow_bin + 1)
    # Tilt the hopper until the given angle is reached by updating
    # tilt and setting the rotation value to tilt's new value.
    # Pause slightly after each tilt and then wait 3 seconds to
    # deposit all bottles safely, before resetting the hopper
    # and deactivating the motor.
    while tilt < angle:
        tilt += angle / 2
        bot.rotate_hopper(tilt)
        time.sleep(0.15)
    time.sleep(3)
    bot.rotate_hopper(0)
    bot.deactivate_stepper_motor()


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
    False); bottle_info - 2-integer tuple (default to empty); offset -
    real number (default to 0)
    Outputs: 2-item tuple - Boolean first, integer tuple second.

    Author: Alvin Qian
    Last Update: 2022/02/13
    """
    # Track the number of bottles, the total mass, and destination bins,
    # setting all counters to 0
    bottle_count, total_mass, dest_bin = [0] * 3
    # If a bottle does not exist on the table right now, spawn one and use its
    # information. Note that bottles.pop() removes the last item of the list.
    if not has_bottle:
        bottle_info = extract_bottle_info(bottles.pop())
    # Add the total mass of the bottle to total_mass and set the destination
    # to that of the bottle. Load the bottle, keeping track of the count to
    # indicate how deep inside the hopper the Q-Arm must reach to safely
    # load the bottle. Ensure the offset of the hopper position is passed
    # onto the Q-Arm. Reset has_bottle to False and track if another bottle
    # has been left on the table.
    total_mass += bottle_info[0]
    main_dest = bottle_info[1]
    load_bottle(bottle_count, offset)
    bottle_count += 1
    has_bottle = False
    # Spawn and potentially load bottles as long as there are less than
    # 3 loaded bottles and the total mass of the spawned and loaded
    # bottles is below the 90 gram threshold, and if there are still
    # bottles to spawn.
    while total_mass < 90 and bottle_count < 3 and bottles:
        # For debugging
        print("Next bottle ID:", bottles[-1])
        # Remove the last item from the list and spawn it. If the mass
        # of the bottle along with that of any previously loaded ones
        # is below 90, and the destination of the spawned bottle matches
        # the destination of the loaded bottles, then update the count
        # and load the bottle.
        bottle_info = extract_bottle_info(bottles.pop())
        print(bottle_info)
        if bottle_info[0] + total_mass < 90 and bottle_info[1] == main_dest:
            load_bottle(bottle_count, offset)
            bottle_count += 1
        # Otherwise, if the mass is too much or there is a mismatch of bins,
        # set has_bottle to True to indicate that a bottle remains on the
        # table, and break from the loop.
        else:
            # For debugging
            has_bottle = True
            break
    # Return has_bottle to indicate there is no bottles on the table,
    # followed by the destination of the loaded bottles and the
    # information of the most recently dropped bottle.
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
    Last Update: 2022/01/31
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
    random.shuffle(bottles_order)
    # For debugging, print the sub-list of elements that will be returned
    print('Order of bottle dispensing: ', bottles_order[:size])
    bottles_order.reverse()
    return bottles_order[-size:]


# ========================= Main Function ========================= #


def main():
    """
    Function: main()

    Purpose: Main function, calls all other functions and maintains program's
    general workflow.

    Inputs: None
    Outputs: None

    Author: Mohammad Mahdi Mahboob
    Last Update: 2022/02/13
    """
    # Holds information about the bottle type for reloading. Information
    # pertains to mass at [0] and destination at [1]. Useful for loading
    # any existing bottles on the table that were not loaded previously.
    bottle_info = ()
    has_bottle = False
    # Keeps track of the offset of the Q-Bot from original home position
    offset = 0
    # Generate the random order of bottles. Remove size input to run full
    # length program with 3 occurrences of all 6 bottle types.
    bottles = random_bottle_list(5)
    # While there are still bottles left in the list or the table, run cycles.
    while bottles or has_bottle:
        # Dispense and load bottles as needed. Check for any remaining bottles
        # on the table as well as get the destination bin number for dumping
        # and information on remaining bottles. Load the bottles after they
        # spawn and move them to the corresponding bin. Dump the bottles,
        # then return home, and update the offset caused by returning.
        has_bottle, bin_num, bottle_info = dispense_bottles(
            bottles, offset, has_bottle, bottle_info)
        line_follow(bin_num)
        time.sleep(1)
        drop_bottle(50, bin_num != 1)
        time.sleep(1)
        line_follow()
        offset = check_home(0.025)
        print(offset)


main()

# ---------------------------------------------------------------------------------
# STUDENT CODE ENDS
# ---------------------------------------------------------------------------------
