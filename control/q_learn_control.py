# (ros2 node info /camera_controller)
#   Publishers:
#     /camera/camera_info: sensor_msgs/msg/CameraInfo
#     /camera/image_raw: sensor_msgs/msg/Image
#     /camera/image_raw/compressed: sensor_msgs/msg/CompressedImage
#     /camera/image_raw/compressedDepth: sensor_msgs/msg/CompressedImage
#     /camera/image_raw/theora: theora_image_transport/msg/Packet
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /rosout: rcl_interfaces/msg/Log
#   Service Servers:
#     /camera_controller/describe_parameters: rcl_interfaces/srv/DescribeParameters
#     /camera_controller/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
#     /camera_controller/get_parameters: rcl_interfaces/srv/GetParameters
#     /camera_controller/list_parameters: rcl_interfaces/srv/ListParameters
#     /camera_controller/set_parameters: rcl_interfaces/srv/SetParameters
#     /camera_controller/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
#     /set_camera_info: sensor_msgs/srv/SetCameraInfo

# (ros2 node info /diff_drive)
#   Publishers:
#     /odom: nav_msgs/msg/Odometry
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /rosout: rcl_interfaces/msg/Log
#     /tf: tf2_msgs/msg/TFMessage
#   Service Servers:
#     /diff_drive/describe_parameters: rcl_interfaces/srv/DescribeParameters
#     /diff_drive/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
#     /diff_drive/get_parameters: rcl_interfaces/srv/GetParameters
#     /diff_drive/list_parameters: rcl_interfaces/srv/ListParameters
#     /diff_drive/set_parameters: rcl_interfaces/srv/SetParameters
#     /diff_drive/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically

# /laser_controller
# Publishers:
#     /parameter_events: rcl_interfaces/msg/ParameterEvent
#     /rosout: rcl_interfaces/msg/Log
#     /scan: sensor_msgs/msg/LaserScan
#   Service Servers:
#     /laser_controller/describe_parameters: rcl_interfaces/srv/DescribeParameters
#     /laser_controller/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
#     /laser_controller/get_parameters: rcl_interfaces/srv/GetParameters
#     /laser_controller/list_parameters: rcl_interfaces/srv/ListParameters
#     /laser_controller/set_parameters: rcl_interfaces/srv/SetParameters
#     /laser_controller/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically