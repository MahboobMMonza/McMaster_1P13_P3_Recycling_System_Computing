# Virtual Environment Code Logic

## Description

This file provides additional documentation for [virtual_environment_code.py][1], and serves to bridge any gaps in logic
or functionality features that are not explained in the comments of the code. Any documentation not found in this
article can be understood from the comments of the code, and it is recommended that the comments in the code be read
before consulting this document.

## Documentation

### Bottle Loading with Q-Arm

#### *load_bottle()*

The program works by assuming the Q-Arm always starts from the `ARM_HOME` position. The variable `grip` tracks the
required status of the Q-Arm's gripper, specifically whether it should be open or closed. Based on the order of the
locations listed in `locations`, the gripper should be closed at `PICK_UP` (index 0) and open at `drop_off` (index 2).
It happens that the index of these two locations in the list are even, and that `grip` should indicate a closed gripper
at `PICK_UP` and an open gripper at `drop_off`. As such, checking whether an even index is reached is vital for
determining if the gripper should be closed or open.

When the index of the iteration is even, the alternating nature of raising -1 to a power is leveraged to instruct the
gripper to open or close. At index 0, `grip` is also 0, and (-1) raised to 0 is just 1. This positive value of 1 is then
multiplied by `GRIPPER_ANGLE` to instruct the gripper to close. The value of `grip` is then incremented by 1. When the
index is 2, the same logic is applied to instruct the gripper to open, noting this time that (-1) raised to 1 is -1,
which, when multiplied by the positive constant `GRIPPER_ANGLE`, gives a negative number with the same magnitude as the
constant. This instructs the gripper to revert to its original unopened shape, dropping the bottle in the process.

#### *adjust_bottles()*

The virtual environment simulation artificially creates static friction between the Q-Arm and the bottle when they are
in contact during the pushing motion. This causes the bottles to sometimes stick like a magnet to the end of the gripper
when it is moving away, causing it to be displaced from the hopper. To prevent this phenomenon, the gripper opens
slightly to release contact with the bottle before it moves away. In the real world, this adjustment is not necessary
and can be ignored.

### Q-Bot Movement and Positioning

#### *check_home()*

Due to the floating-point nature of the location data provided by the bot, floating-point precision error occurs when
checking exact matches for the position. This would cause the program to not recognize that the Q-Bot has reached
the `HOME` destination, causing it to continue moving along the line. To prevent this, an acceptable margin of error was
introduced that checks whether the Q-Bot is within some distance *just before* the `HOME` position, allowing it to stop
much closer to the intended stopping position.

#### *bot_align()*

The idea of this function was to have the Q-Bot complete "sweeps" across the line to determine where the middle of the
line was located from its current position. This technique at least guarantees that the hopper tilts towards the inside
of a bin when it deposits a container, and also allows the hopper to be more perpendicular to the Q-Arm when it returns
to the `HOME` position. The final angle of the hopper with respect to the target depends on the orientation of the Q-Bot
when it first detects it, but this method guarantees that the hopper is within an acceptable margin of error. The Q-Bot
moves 0.1 m forwards after it detects a bin to compensate for the nature of how bins are detected in the simulation,
which is the shortest distance from any point on the circumference of the Q-Bot to the closest bin. The short distance
it travels ensures that the hopper can tilt into a bin and not its edge or outside it.

#### *line_follow()*

The detection of a bin works by checking whether the distance read by the ultrasonic sensor is within `PROX_DIST` or
outside `PASS_DIST`. The first distance is used in conjunction with `encounter_bin` to determine if a new bin has been
detected, at which point `encounter_bin` switches on. This is to prevent double-counting a bin, as distance readings
when near a bin will all be smaller. `PASS_DIST` is used to determine if there is no bin nearby, which is determined if
the distance readings are larger than the distance between the bins and the line path. This again flips `encounter_bin`
off, priming it to detect the next bin.

The wheel speed calculation is made to match the following reading-to-speed mapping:

##### Table 1. Line Follow Sensor Readings Mapped to Q-Bot Wheel Speeds
| Readings [L, R] | Wheel Speeds [L, R] |
|:---------------:|:-------------------:|
|     [0, 0]      |   [0.025, 0.050]    |
|     [1, 0]      |   [0.025, 0.050]    |
|     [1, 1]      |   [0.050, 0.050]    |
|     [0, 1]      |   [0.050, 0.025]    |

There is a pattern to these mappings, and can be expressed as a function that takes in the readings as input and outputs
the speed for a wheel. Combining a function for both the right and left wheels that exploits this pattern, the new wheel
speeds can be calculated for the Q-Bot without the need for multiple branching if statements. This is important because
branching is a costly operation and avoiding branching in this portion allows for more readings to be analyzed by the
Q-Bot when following the line, making it more accurate. Noting the minimum speed of the wheels is `SPEED`, which is
0.025, it can be seen that the right wheel is twice this value as long as either the right sensor reading is 0 or the 
left sensor reading is 1, or equivalently, it is only slow whenever the right sensor reading is smaller than the left.
This is what is reflected for the right wheel's speed calculation, as the innermost bracket is a boolean expression that
is only true in the specified case, and the integer evaluation of `True` in Python is 1, which is subtracted from the
otherwise doubling 2, giving 2 * `SPEED` for the first 3 values in the table and just `SPEED` for the last. A similar
calculation is done for the left wheel, this time exploiting the fact that the left wheel is only ever double the speed
if the right sensor reading is 1; the addition evaluates 1 * `SPEED` for any 0 readings and 2 * `SPEED` for any 1
readings.

To make the bot move more accurately, these movements are only executed for a quarter of a second before another 
evaluation determines whether the bot should be stopped, which occurs when the bot is turning, or when the speeds are
not the same. This allows for a more accurate turn, preventing too much overshooting and overcorrecting. Another
mathematical trick is used again to reduce branching. The trick capitalizes on the readings being [1, 1] whenever the
bot does not stop, and uses integer division to set both wheels to 0 whenever this is not the case. The largest possible
sum of the left and right sensor readings is 2, which occurs in the [1, 1] case, otherwise it can be 0 or 1. The last
two cases indicate that wheel speeds are not the same, as seen in Table 1. Since integer division drops the remainder,
this makes it a useful tool for setting the speeds to 0, as both 0/2 and 1/2 have whole quotients of 0, while 2/2 is 1. 
Multiplying these quotients sets the wheel speed to be non-zero only in the [1, 1] case, allowing the Q-Bot to move
forwards. The multiplication by 4 allows the Q-Bot to move faster for short bursts while maintaining controlled
movement.

#### *drop_bottle()*

The need to control the tilting speed of the hopper arose from the occurrence of bottles being flung past bins during
initial drop sequences. To control this effect, the rate of tilting was controlled using steps and delays for certain
bottle types. This is akin to controlling the duration of the linear actuator being extended in steps, and therefore
makes it a viable solution.

[1]: virtual_environment_code.py
