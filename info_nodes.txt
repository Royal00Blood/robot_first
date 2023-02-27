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



# topic list:
# /camera/camera_info
    # Type: sensor_msgs/msg/CameraInfo
            # # This message defines meta information for a camera. It should be in a
            # # camera namespace on topic "camera_info" and accompanied by up to five
            # # image topics named:
            # #
            # #   image_raw - raw data from the camera driver, possibly Bayer encoded
            # #   image            - monochrome, distorted
            # #   image_color      - color, distorted
            # #   image_rect       - monochrome, rectified
            # #   image_rect_color - color, rectified
            # #
            # # The image_pipeline contains packages (image_proc, stereo_image_proc)
            # # for producing the four processed image topics from image_raw and
            # # camera_info. The meaning of the camera parameters are described in
            # # detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
            # #
            # # The image_geometry package provides a user-friendly interface to
            # # common operations using this meta information. If you want to, e.g.,
            # # project a 3d point into image coordinates, we strongly recommend
            # # using image_geometry.
            # #
            # # If the camera is uncalibrated, the matrices D, K, R, P should be left
            # # zeroed out. In particular, clients may assume that K[0] == 0.0
            # # indicates an uncalibrated camera.

            # #######################################################################
            # #                     Image acquisition info                          #
            # #######################################################################

            # # Time of image acquisition, camera coordinate frame ID
            # std_msgs/Header header # Header timestamp should be acquisition time of image
            #     builtin_interfaces/Time stamp
            #             int32 sec
            #             uint32 nanosec
            #     string frame_id
            #                             # Header frame_id should be optical frame of camera
            #                             # origin of frame should be optical center of camera
            #                             # +x should point to the right in the image
            #                             # +y should point down in the image
            #                             # +z should point into the plane of the image


            # #######################################################################
            # #                      Calibration Parameters                         #
            # #######################################################################
            # # These are fixed during camera calibration. Their values will be the #
            # # same in all messages until the camera is recalibrated. Note that    #
            # # self-calibrating systems may "recalibrate" frequently.              #
            # #                                                                     #
            # # The internal parameters can be used to warp a raw (distorted) image #
            # # to:                                                                 #
            # #   1. An undistorted image (requires D and K)                        #
            # #   2. A rectified image (requires D, K, R)                           #
            # # The projection matrix P projects 3D points into the rectified image.#
            # #######################################################################

            # # The image dimensions with which the camera was calibrated.
            # # Normally this will be the full camera resolution in pixels.
            # uint32 height
            # uint32 width

            # # The distortion model used. Supported models are listed in
            # # sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
            # # simple model of radial and tangential distortion - is sufficent.
            # string distortion_model

            # # The distortion parameters, size depending on the distortion model.
            # # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
            # float64[] d

            # # Intrinsic camera matrix for the raw (distorted) images.
            # #     [fx  0 cx]
            # # K = [ 0 fy cy]
            # #     [ 0  0  1]
            # # Projects 3D points in the camera coordinate frame to 2D pixel
            # # coordinates using the focal lengths (fx, fy) and principal point
            # # (cx, cy).
            # float64[9]  k # 3x3 row-major matrix

            # # Rectification matrix (stereo cameras only)
            # # A rotation matrix aligning the camera coordinate system to the ideal
            # # stereo image plane so that epipolar lines in both stereo images are
            # # parallel.
            # float64[9]  r # 3x3 row-major matrix

            # # Projection/camera matrix
            # #     [fx'  0  cx' Tx]
            # # P = [ 0  fy' cy' Ty]
            # #     [ 0   0   1   0]
            # # By convention, this matrix specifies the intrinsic (camera) matrix
            # #  of the processed (rectified) image. That is, the left 3x3 portion
            # #  is the normal camera intrinsic matrix for the rectified image.
            # # It projects 3D points in the camera coordinate frame to 2D pixel
            # #  coordinates using the focal lengths (fx', fy') and principal point
            # #  (cx', cy') - these may differ from the values in K.
            # # For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
            # #  also have R = the identity and P[1:3,1:3] = K.
            # # For a stereo pair, the fourth column [Tx Ty 0]' is related to the
            # #  position of the optical center of the second camera in the first
            # #  camera's frame. We assume Tz = 0 so both cameras are in the same
            # #  stereo image plane. The first camera always has Tx = Ty = 0. For
            # #  the right (second) camera of a horizontal stereo pair, Ty = 0 and
            # #  Tx = -fx' * B, where B is the baseline between the cameras.
            # # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
            # #  the rectified image is given by:
            # #  [u v w]' = P * [X Y Z 1]'
            # #         x = u / w
            # #         y = v / w
            # #  This holds for both images of a stereo pair.
            # float64[12] p # 3x4 row-major matrix


            # #######################################################################
            # #                      Operational Parameters                         #
            # #######################################################################
            # # These define the image region actually captured by the camera       #
            # # driver. Although they affect the geometry of the output image, they #
            # # may be changed freely without recalibrating the camera.             #
            # #######################################################################

            # # Binning refers here to any camera setting which combines rectangular
            # #  neighborhoods of pixels into larger "super-pixels." It reduces the
            # #  resolution of the output image to
            # #  (width / binning_x) x (height / binning_y).
            # # The default values binning_x = binning_y = 0 is considered the same
            # #  as binning_x = binning_y = 1 (no subsampling).
            # uint32 binning_x
            # uint32 binning_y

            # # Region of interest (subwindow of full camera resolution), given in
            # #  full resolution (unbinned) image coordinates. A particular ROI
            # #  always denotes the same window of pixels on the camera sensor,
            # #  regardless of binning settings.
            # # The default setting of roi (all values 0) is considered the same as
            # #  full resolution (roi.width = width, roi.height = height).
            # RegionOfInterest roi
            #     #
            #     uint32 x_offset  #
            #                         # (0 if the ROI includes the left edge of the image)
            #     uint32 y_offset  #
            #                         # (0 if the ROI includes the top edge of the image)
            #     uint32 height    #
            #     uint32 width     #
            #     bool do_rectify
# /camera/image_raw
    # Type: sensor_msgs/msg/Image
    # Publisher count: 1
    # Subscription count: 0
            # # This message contains an uncompressed image
            # # (0, 0) is at top-left corner of image

            # std_msgs/Header header # Header timestamp should be acquisition time of image
            # builtin_interfaces/Time stamp
            #     int32 sec
            #     uint32 nanosec
            # string frame_id
            #                     # Header frame_id should be optical frame of camera
            #                     # origin of frame should be optical center of cameara
            #                     # +x should point to the right in the image
            #                     # +y should point down in the image
            #                     # +z should point into to plane of the image
            #                     # If the frame_id here and the frame_id of the CameraInfo
            #                     # message associated with the image conflict
            #                     # the behavior is undefined

            # uint32 height                # image height, that is, number of rows
            # uint32 width                 # image width, that is, number of columns

            # # The legal values for encoding are in file src/image_encodings.cpp
            # # If you want to standardize a new string format, join
            # # ros-users@lists.ros.org and send an email proposing a new encoding.

            # string encoding       # Encoding of pixels -- channel meaning, ordering, size
            #             # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

            # uint8 is_bigendian    # is this data bigendian?
            # uint32 step           # Full row length in bytes
            # uint8[] data          # actual matrix data, size is (step * rows)
# /clicked_point
        # geometry_msgs/msg/PointStamped
                # # This represents a Point with reference coordinate frame and timestamp

                # std_msgs/Header header
                # builtin_interfaces/Time stamp
                #         int32 sec
                #         uint32 nanosec
                # string frame_id
                # Point point
                # float64 x
                # float64 y
                # float64 z
# /clock
    # rosgraph_msgs/msg/Clock
            # # This message communicates the current time.
            # #
            # # For more information, see https://design.ros2.org/articles/clock_and_time.html.
            # builtin_interfaces/Time clock
            # int32 sec
            # uint32 nanosec
# /cmd_vel
        #    geometry_msgs/msg/Twist
            # # This expresses velocity in free space broken into its linear and angular parts.

            # Vector3  linear
                # float64 x
                # float64 y
                # float64 z
            # Vector3  angular
                # float64 x
                # float64 y
                # float64 z
# /goal_pose
        # geometry_msgs/msg/PoseStamped
                # # A Pose with reference coordinate frame and timestamp

                # std_msgs/Header header
                #         builtin_interfaces/Time stamp
                #                 int32 sec
                #                 uint32 nanosec
                #         string frame_id
                # Pose pose
                #         Point position
                #                 float64 x
                #                 float64 y
                #                 float64 z
                #         Quaternion orientation
                #                 float64 x 0
                #                 float64 y 0
                #                 float64 z 0
                #                 float64 w 1
# /initialpose
        # geometry_msgs/msg/PoseWithCovarianceStamped
                # # This expresses an estimated pose with a reference coordinate frame and timestamp

                # std_msgs/Header header
                #         builtin_interfaces/Time stamp
                #                 int32 sec
                #                 uint32 nanosec
                #         string frame_id
                # PoseWithCovariance pose
                #         Pose pose
                #                 Point position
                #                         float64 x
                #                         float64 y
                #                         float64 z
                #                 Quaternion orientation
                #                         float64 x 0
                #                         float64 y 0
                #                         float64 z 0
                #                         float64 w 1
                #         float64[36] covariance  
# /joint_states
    #sensor_msgs/msg/JointState
        # # This is a message that holds data to describe the state of a set of torque controlled joints.
        # #
        # # The state of each joint (revolute or prismatic) is defined by:
        # #  * the position of the joint (rad or m),
        # #  * the velocity of the joint (rad/s or m/s) and
        # #  * the effort that is applied in the joint (Nm or N).
        # #
        # # Each joint is uniquely identified by its name
        # # The header specifies the time at which the joint states were recorded. All the joint states
        # # in one message have to be recorded at the same time.
        # #
        # # This message consists of a multiple arrays, one for each part of the joint state.
        # # The goal is to make each of the fields optional. When e.g. your joints have no
        # # effort associated with them, you can leave the effort array empty.
        # #
        # # All arrays in this message should have the same size, or be empty.
        # # This is the only way to uniquely associate the joint name with the correct
        # # states.

        # std_msgs/Header header
        #         builtin_interfaces/Time stamp
        #                 int32 sec
        #                 uint32 nanosec
        #         string frame_id

        # string[] name
        # float64[] position
        # float64[] velocity
        # float64[] effort
# /odom
    # nav_msgs/msg/Odometry
            # # This represents an estimate of a position and velocity in free space.
            # # The pose in this message should be specified in the coordinate frame given by header.frame_id
            # # The twist in this message should be specified in the coordinate frame given by the child_frame_id

            # # Includes the frame id of the pose parent.
            # std_msgs/Header header
            #         builtin_interfaces/Time stamp
            #                 int32 sec
            #                 uint32 nanosec
            #         string frame_id

            # # Frame id the pose points to. The twist is in this coordinate frame.
            # string child_frame_id

            # # Estimated pose that is typically relative to a fixed world frame.
            # geometry_msgs/PoseWithCovariance pose
            #         Pose pose
            #                 Point position
            #                         float64 x
            #                         float64 y
            #                         float64 z
            #                 Quaternion orientation
            #                         float64 x 0
            #                         float64 y 0
            #                         float64 z 0
            #                         float64 w 1
            #         float64[36] covariance

            # # Estimated linear and angular velocity relative to child_frame_id.
            # geometry_msgs/TwistWithCovariance twist
            #         Twist twist
            #                 Vector3  linear
            #                         float64 x
            #                         float64 y
            #                         float64 z
            #                 Vector3  angular
            #                         float64 x
            #                         float64 y
            #                         float64 z
            #         float64[36] covariance
# /parameter_events
        # rcl_interfaces/msg/ParameterEvent
                # # This message contains a parameter event.
                # # Because the parameter event was an atomic update, a specific parameter name
                # # can only be in one of the three sets.

                # # The time stamp when this parameter event occurred.
                # builtin_interfaces/Time stamp
                #         int32 sec
                #         uint32 nanosec

                # # Fully qualified ROS path to node.
                # string node

                # # New parameters that have been set for this node.
                # Parameter[] new_parameters
                #         string name
                #         ParameterValue value
                #                 uint8 type
                #                 bool bool_value
                #                 int64 integer_value
                #                 float64 double_value
                #                 string string_value
                #                 byte[] byte_array_value
                #                 bool[] bool_array_value
                #                 int64[] integer_array_value
                #                 float64[] double_array_value
                #                 string[] string_array_value

                # # Parameters that have been changed during this event.
                # Parameter[] changed_parameters
                #         string name
                #         ParameterValue value
                #                 uint8 type
                #                 bool bool_value
                #                 int64 integer_value
                #                 float64 double_value
                #                 string string_value
                #                 byte[] byte_array_value
                #                 bool[] bool_array_value
                #                 int64[] integer_array_value
                #                 float64[] double_array_value
                #                 string[] string_array_value

                # # Parameters that have been deleted during this event.
                # Parameter[] deleted_parameters
                #         string name
                #         ParameterValue value
                #                 uint8 type
                #                 bool bool_value
                #                 int64 integer_value
                #                 float64 double_value
                #                 string string_value
                #                 byte[] byte_array_value
                #                 bool[] bool_array_value
                #                 int64[] integer_array_value
                #                 float64[] double_array_value
                #                 string[] string_array_value
# /performance_metrics
        # gazebo_msgs/msg/PerformanceMetrics
                # td_msgs/Header header
                #     builtin_interfaces/Time stamp
                #         int32 sec
                #         uint32 nanosec
                #     string frame_id

                # float64 real_time_factor
                # gazebo_msgs/SensorPerformanceMetric[] sensors
                #     string name
                #     float64 sim_update_rate
                #     float64 real_update_rate
                #     float64 fps
# /robot_description
    # std_msgs/msg/String
            # # This was originally provided as an example message.
            # # It is deprecated as of Foxy
            # # It is recommended to create your own semantically meaningful message.
            # # However if you would like to continue using this please use the equivalent in example_msgs.

            # string data

# /rosout
        # rcl_interfaces/msg/Log
                        # ##
                        # ## Severity level constants
                        # ##
                        # ## These logging levels follow the Python Standard
                        # ## https://docs.python.org/3/library/logging.html#logging-levels
                        # ## And are implemented in rcutils as well
                        # ## https://github.com/ros2/rcutils/blob/35f29850064e0c33a4063cbc947ebbfeada11dba/include/rcutils/logging.h#L164-L172
                        # ## This leaves space for other standard logging levels to be inserted in the middle in the future,
                        # ## as well as custom user defined levels.
                        # ## Since there are several other logging enumeration standard for different implementations,
                        # ## other logging implementations may need to provide level mappings to match their internal implementations.
                        # ##

                        # # Debug is for pedantic information, which is useful when debugging issues.
                        # byte DEBUG=10

                        # # Info is the standard informational level and is used to report expected
                        # # information.
                        # byte INFO=20

                        # # Warning is for information that may potentially cause issues or possibly unexpected
                        # # behavior.
                        # byte WARN=30

                        # # Error is for information that this node cannot resolve.
                        # byte ERROR=40

                        # # Information about a impending node shutdown.
                        # byte FATAL=50

                        # ##
                        # ## Fields
                        # ##

                        # # Timestamp when this message was generated by the node.
                        # builtin_interfaces/Time stamp
                        #     int32 sec
                        #     uint32 nanosec

                        # # Corresponding log level, see above definitions.
                        # uint8 level

                        # # The name representing the logger this message came from.
                        # string name

                        # # The full log message.
                        # string msg

                        # # The file the message came from.
                        # string file

                        # # The function the message came from.
                        # string function

                        # # The line in the file the message came from.
                        # uint32 line        

# /scan
    # sensor_msgs/msg/LaserScan
            # # Single scan from a planar laser range-finder
            # #
            # # If you have another ranging device with different behavior (e.g. a sonar
            # # array), please find or create a different message, since applications
            # # will make fairly laser-specific assumptions about this data

            # std_msgs/Header header # timestamp in the header is the acquisition time of
            #     builtin_interfaces/Time stamp
            #         int32 sec
            #         uint32 nanosec
            #     string frame_id
            #                             # the first ray in the scan.
            #                             #
            #                             # in frame frame_id, angles are measured around
            #                             # the positive Z axis (counterclockwise, if Z is up)
            #                             # with zero angle being forward along the x axis

            # float32 angle_min            # start angle of the scan [rad]
            # float32 angle_max            # end angle of the scan [rad]
            # float32 angle_increment      # angular distance between measurements [rad]

            # float32 time_increment       # time between measurements [seconds] - if your scanner
            #                             # is moving, this will be used in interpolating position
            #                             # of 3d points
            # float32 scan_time            # time between scans [seconds]

            # float32 range_min            # minimum range value [m]
            # float32 range_max            # maximum range value [m]

            # float32[] ranges             # range data [m]
            #                             # (Note: values < range_min or > range_max should be discarded)
            # float32[] intensities        # intensity data [device-specific units].  If your
            #                             # device does not provide intensities, please leave
            #                             # the array empty.

# /tf
    # tf2_msgs/msg/TFMessag
            # geometry_msgs/TransformStamped[] transforms
            #     #
            #     #
            #     std_msgs/Header header
            #         builtin_interfaces/Time stamp
            #             int32 sec
            #             uint32 nanosec
            #         string frame_id
            #     string child_frame_id
            #     Transform transform
            #         Vector3 translation
            #             float64 x
            #             float64 y
            #             float64 z
            #         Quaternion rotation
            #             float64 x 0
            #             float64 y 0
            #             float64 z 0
            #             float64 w 1

# /tf_static
    # tf2_msgs/msg/TFMessage
            # geometry_msgs/TransformStamped[] transforms
            #     #
            #     #
            #     std_msgs/Header header
            #         builtin_interfaces/Time stamp
            #             int32 sec
            #             uint32 nanosec
            #         string frame_id
            #     string child_frame_id
            #     Transform transform
            #         Vector3 translation
            #             float64 x
            #             float64 y
            #             float64 z
            #         Quaternion rotation
            #             float64 x 0
            #             float64 y 0
            #             float64 z 0
            #             float64 w 1
