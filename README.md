# ekf_fusion_my
#process data中是因为rosbag数据不完全，所以预处理。主要包括读取yaml中的协方差，发布odom->base_link的tf，用service设置ekf的状态。
#serial test主要是二维码定位的msg，因为两个节点都用到了，所以单独开了一个功能包，防止编译不通过。
#robot_localization-melodic-devel是开源代码。
#开源代码基础上，修改了EKF：
#1、将new改为了智能指针
#2、将ekf的预测模型从全向模型改为了CTRA模型，角速度很小时退化成CV模型。
#3、修改后只适用于二维平面。
#4、原本的更新没变，但是当遇到measurement来自二维码定位时用IEKF，观测是#base_link在二维码系下的位姿 和 #二维码在世界系下的位置角度
#
# roslaunch robot_localization ekf_template.launch运行
