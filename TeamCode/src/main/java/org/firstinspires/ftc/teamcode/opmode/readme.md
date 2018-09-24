## Opmode Section
This section has all of the classes to do with defining and running opModes - TeleOp and Autonomous.
Please only put the files that need to directly interface with these hardware classes and the inputs
  in this folder
Put other resources into other packages (vision, etc.)

## Organization
The main class that will be directly interfacing with all of the robot's actual hardware
is the RobotHardware abstract class. For each robot configuration, please extend this and
copy-paste the required sample functions from the 'Example' package. Those functions and the
functions in RobotHardware will be explained in more detail in those two classes.
From your abstract Hardware class, you extend it to make your teleop and various autonomi. All
required functions should be available to you.

## Best practices
1. Where things go
     -A lot of times, there are small quality-of-life functions that are very useful for controlling
       the robot in general. For example, you may have a function that raises/lowers a lift,
       but checks if it lower or higher than certain limits. If you can, put the function in the
       robot config-specific abstract class, because in that way all opmodes and autonomi will be
       able to access them.
2. Changing RobotHardware
     -A