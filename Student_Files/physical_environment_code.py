import sys
from time import sleep
from Common.project_library import *

sys.path.append('../')

# Modify the information below according to your setup and uncomment the entire section

# 1. Interface Configuration
project_identifier = 'P3B'  # enter a string corresponding to P0, P2A, P2A, P3A, or P3B
ip_address = '169.254.134.253'  # enter your computer's IP address
hardware = True  # True when working with hardware. False when working in the simulation

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

# ========================= Q-Bot Movement and Positioning ========================= #


def bot_align():
    """
    Function: bot_align()

    Purpose: This function aligns the bot line sensors to the line so that the
    hopper faces perpendicular to boxes or the Q-Arm.

    Inputs: None
    Outputs: None

    Author: Liam Walker, Mohammad Mahdi Mahboob
    Last Update: 2022/02/16
    """
    # Read the value of the sensors until [1, 1] has been read twice to
    # Ensure it is more so in the middle. If it has not been read twice,
    # slightly rotate the bot towards the line.
    readings = bot.line_following_sensors()
    # Track the number of [1, 1] readings, starting from the most
    # recent reading
    count = (readings[0] + readings[1]) // 2
    # Track wheel speeds
    speeds = [0, 0]
    # Align the bot until 2 readings of [1, 1] are encountered.
    while count < 2:
        # Calculate the speed of the wheels based on sensor readings.
        # Turn left or right accordingly.
        speeds[0] = 0.025 + 0.025 * readings[1]
        speeds[1] = 0.025 + 0.025 * readings[0]
        # Move slightly, stop, then re-evaluate alignment and count
        bot.set_wheel_speed(speeds)
        sleep(0.1)
        bot.stop()
        readings = bot.line_following_sensors()
        count += (readings[0] + readings[1]) // 2


def line_follow(bin_num=-1, start_bin=-1):
    """
    Function: line_follow()

    Purpose: This function moves the Q-Bot along the line until it reaches
    the given target location.

    Inputs: bin_num - integer (default -1); start_bin - integer
    Outputs: integer

    Author: Mohammad Mahdi Mahboob, Liam Walker
    Last Update: 2022/02/16
    """
    # Turn on the ultrasonic sensor
    bot.activate_ultrasonic_sensor()
    # Constants for dictating minimum and maximum thresholds to determine
    # if the bot has passed a bin, and for the minimum speed of the bot's
    # wheels
    PROX_DIST = 0.1
    PASS_DIST = 0.2
    SPEED = 0.05
    # tracks the number of bins passed
    cur_bin = start_bin
    # lists to hold line sensor readings and wheel speeds
    reading, speeds = [], [0, 0]
    # Flag to check if currently near a bin
    encounter_bin = False
    print('Target:', bin_num)
    # While the bot is not at the desired bin or not home
    while cur_bin < bin_num:
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
        bot_align()
        print("Bin/stop location found")
        bot.deactivate_ultrasonic_sensor()
    return cur_bin - 1


def drop_bottle(duration):
    """
    Function: drop_bottle()

    Purpose: This function rotates the actuator to tilt the hopper and
    drop the containers inside the bin.

    Inputs: angle - real number
    Outputs: None

    Author: Liam Walker
    Last Update: 2022/02/13
    """
    # Activate the actuator, move the actuator out to tilt the hopper
    # and hold the tilt for some time before moving the actuator back
    # in to reset the hopper, then deactivate the actuator.
    bot.activate_linear_actuator()
    bot.linear_actuator_out(duration)
    sleep(3)
    bot.linear_actuator_in(duration)
    bot.deactivate_linear_actuator()


# ========================= Testing Plans ========================= #


def line_sensor_test():
    tests = ["Testing [1, 1]", "Testing [1, 0]", "Testing [0, 0]", "Testing [0, 1]"]
    print("Testing line sensor")
    for test_case in tests:
        print(test_case)
        for i in range(5):
            print(bot.line_following_sensors())
            sleep(2)
        print("Prepare for next test")
        sleep(5)


def ultrasonic_test_1():
    bot.activate_ultrasonic_sensor()
    print("Testing read values for different ranges")
    print("Direct readings")
    for i in range(20):
        print("Move object along sensor")
        print(bot.read_ultrasonic_sensor())
        sleep(3)
    print("Angled readings")
    for i in range(20):
        print("Move object perpendicular to sensor direction")
        print(bot.read_ultrasonic_sensor())
        sleep(3)
    bot.deactivate_ultrasonic_sensor()


def ultrasonic_test_2():
    bot.activate_ultrasonic_sensor()
    print("Testing detection values for different ranges")
    min_dist = 0.1
    max_dist = 0.2
    print("Within meet range")
    for i in range(20):
        print("Move object along sensor")
        print(0 < bot.read_ultrasonic_sensor() < min_dist)
        sleep(3)

    print("Out of meet range")
    for i in range(20):
        print("Move object along sensor")
        reading = bot.read_ultrasonic_sensor()
        print(max_dist < reading or reading == 0)
        sleep(3)


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
    duration = 5
    destination = line_follow(2)
    sleep(1)
    drop_bottle(duration)
    line_follow(5, destination)


# ========================= Test Function Calls ========================= #


def test():
    print("Running Tests")
    tests = [line_sensor_test, ultrasonic_test_1,
             ultrasonic_test_2]
    for test_function in tests:
        sleep(5)
        print("Prepare for next test")
        test_function()
        sleep(10)
    print("Tests completed")


# ======================================================================= #

# ---------------------------------------------------------------------------------
# STUDENT CODE ENDS
# ---------------------------------------------------------------------------------
