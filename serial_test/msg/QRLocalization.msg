std_msgs/Header header
  #uint32 seq
  #time stamp 扫描到二维码的时间
  #string frame_id 应该是二维码系，apriltag
string child_frame_id #base_link
geometry_msgs/PoseWithCovariance pose #base_link在二维码系下的位姿
  #geometry_msgs/Pose pose
    #geometry_msgs/Point position
      #float64 x
      #float64 y
      #float64 z
    #geometry_msgs/Quaternion orientation
      #float64 x
      #float64 y
      #float64 z
      #float64 w
  #float64[36] covariance
geometry_msgs/Pose2D pose2d #二维码在世界系下的位置角度
    #float64 x
    #float64 y
    #float64 theta 弧度制