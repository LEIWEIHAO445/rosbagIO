########################################################################################################################
#
# IMU observation data definition
#
# This message is suitable for KAIST dataset (Xsens imu)
#
########################################################################################################################

Header header  # standard ROS message header

geometry_msgs/Quaternion quaternion_data
geometry_msgs/Vector3 eular_data
geometry_msgs/Vector3 gyro_data
geometry_msgs/Vector3 acceleration_data
geometry_msgs/Vector3 magneticfield_data
