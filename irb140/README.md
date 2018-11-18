# irb140
model and example code for ABB IRB140

start up with:
simu of robot, block-finder node and cartesian-move action server
`roslaunch irb140_description irb140.launch`
`rosrun irb140_planner irb140_cart_move_as`
`rosrun cartesian_motion_commander example_block_grabber`

see output of block finder with:
`rostopic echo /block_pose`
