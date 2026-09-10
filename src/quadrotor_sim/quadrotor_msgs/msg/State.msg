# True state of the vehicle's base link, published by quadrotor_gazebo_plugin.
#
# Unlike nav_msgs/Odometry this carries acceleration, so a planner can seed a
# replan's initial boundary condition from what the vehicle is actually doing
# instead of assuming zero or re-sampling its own previous trajectory.
#
# pose             position (m) and orientation of base_link in the world frame
# velocity         linear velocity of base_link in the world frame (m/s)
# acceleration     linear acceleration of base_link in the world frame (m/s^2).
#                  Kinematic (d^2p/dt^2), NOT proper/IMU acceleration: it reads
#                  zero in hover and -g in free fall.
# angular_velocity angular velocity of base_link in the body frame (rad/s)
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 angular_velocity
