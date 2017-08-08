# mavros_offboard_control
You can do either a waypoint control or giving motion primitives using this package.

**ALERT** Whenever you change values, you must check it using gazebo to see if the quad works fine prior to flying the actual quad.

The initial position of the quad when you use this package is given by the moment when you change to the *offboard* mode. The unit of all variables is meter (m).

Run mavros px4.launch with the usb port setting first. (This package does not contain mavros itself.)

# 1. Waypoint control

You can set waypoints using `waypoint_points.yaml`  in the yaml folder. At the end of the algorithm, the quad remains in the position of the last waypoint you give.

Then, run `roslaunch uav_control waypoint_control.launch`.

# 2. Motion primitive

You can set motion primitives using `motion_primitive_points.yaml` in the yaml folder.

Run `roslaunch uav_control motion_primitive.launch`.

After you run the launch file, the quad remains in the positon that is given when you change the mode to the *offboard* mode. Then, you can give a command for motion primitive by typing `rostopic pub /mavros/motion_primitive std_msgs/String 'A name of your motion primitive'`.
