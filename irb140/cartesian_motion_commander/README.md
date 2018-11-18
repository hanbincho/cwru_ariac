# cartesian_motion_commander
This library should be used by action clients of a cartMoveActionServer.
(see package arm_motion_interface).
Intent is that commands should be task oriented and robot agnostic.

This is a library that defines the class ArmMotionInterface, which contains functions
for sending codes and arguments to an arm-motion interface action server (and for
receiving result messages back from the action server).

This class populates goal messages corresponding to different generic (robot-agnostic)
functions, along with required arguments. A goal message includes at least a command (function) code.
This library communicates with an action server named "cartMoveActionServer".

## Example usage
Start up robot simulation in Gazebo with: 
`roslaunch irb140_description irb140.launch`

this launch file also starts up a corresponding Cartesian-move action server:
irb140_planner/irb140_cart_move_as 

Try running a sample action client:

`rosrun cartesian_motion_commander example_block_grabber`



    
