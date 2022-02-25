# Virtual Environment Code Logic


## Description
This file provides additional documentation for [virtual_environment_code.py][1],
and serves to bridge any gaps in logic or functionality features that are not
explained in the comments of the code.

## Documentation

### Bottle Loading with Q-Arm

#### *load_bottle()*

The program works by assuming the Q-Arm always starts from the `ARM_HOME`
position. The variable `grip` tracks the required status of the Q-Arm's
gripper, specifically whether it should be open or closed. Based on the
order of the locations listed in `locations`, the gripper should be closed
at `PICK_UP` (index 0) and open at `drop_off` (index 2). It happens that 
the index of these two locations in the list are even, and that `grip` should
indicate a closed gripper at `PICK_UP` and an open gripper at `drop_off`. As
such, checking whether an even index is reached is vital for determining if
the gripper should be closed or open. 

When the index of the iteration is even, the alternating nature of raising -1 
to a power is leveraged to instruct the gripper to open or close. At index 0, 
`grip` is also 0, and (-1) raised to 0 is just 1. This positive value of 1 is 
then multiplied by `GRIPPER_ANGLE` to instruct the gripper to close. The value 
of `grip` is then incremented by 1. When the index is 2, the same logic is 
applied to instruct the gripper to open, noting this time that (-1) raised to 
1 is -1, which, when multiplied by the positive constant `GRIPPER_ANGLE`, 
gives a negative number with the same magnitude as the constant. This 
instructs the gripper to revert to its original unopened shape, dropping the 
bottle in the process.

#### *adjust_bottles()*

The virtual environment simulation artificially creates static friction between
the Q-Arm and the bottle when they are in contact during the pushing motion. This
causes the bottles to sometimes stick like a magnet to the end of the gripper
when it is moving away, causing it to be displaced from the hopper. To prevent
this phenomenon, the gripper opens slightly to release contact with the bottle
before it moves away. In the real world, this adjustment is not necessary and
can be ignored.

### Q-Bot Movement and Positioning

#### *check_home()*

Due to the floating-point nature of the location data provided by the bot,
floating-point precision error occurs when checking exact matches for the
position. This would cause the program to not recognize that the Q-Bot has
reached the `HOME` destination, causing it to continue moving along the line.
To prevent this, an acceptable margin of error was introduced that checks
whether the Q-Bot is within some distance *just before* the `HOME` position,
allowing it to stop much closer to the intended stopping position.

[1]: virtual_environment_code.py