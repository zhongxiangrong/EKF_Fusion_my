//转换格式
//赋值协方差矩阵
//发布map到odom的tf（动态发布，还是静态？）
//发布imu，odom，pose
//请求初始位置

#include <memory>
//ROS头文件
#include <ros/ros.h>
//ROS标准msg头文件
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "xmlrpcpp/XmlRpcValue.h"
#include "xmlrpcpp/XmlRpcException.h"

#include "robot_localization/SetPose.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "robot_localization/QRLocalization.h"
#include "serial_test/QRLocalization.h"



class sub_pub
{
public:
    sub_pub(){
        sub_imu = n.subscribe("IMU", 2, &sub_pub::imuCb,this);
        sub_odom = n.subscribe("odom", 2, &sub_pub::odomCb,this);
        sub_pose = n.subscribe("tracked_pose", 2, &sub_pub::poseCb,this);
        sub_QR = n.subscribe("for_kf", 2, &sub_pub::QRCb,this);
        pub_imu = n.advertise<sensor_msgs::Imu>("imu_", 2);
        pub_odom = n.advertise<nav_msgs::Odometry>("odom_", 2);
        pub_odominworld = n.advertise<nav_msgs::Odometry>("odom_in_world", 2);
        pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_", 2);
        pub_QR = n.advertise<robot_localization::QRLocalization>("QR_pose_", 2);
        client = n.serviceClient<robot_localization::SetPose::Request>("set_pose");

        //读取yaml文件
        nh = ros::NodeHandle("~");
        nh.param("odom_to_base_link_publisher",enable_odom2base_publisher,true);
        nh.param("enable_set_pose",enable_set_pose,false);

        XmlRpc::XmlRpcValue config;
        nh.getParam("imu_orientation_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 3);
        for (int i = 0; i < 3; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> imu_orientation_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }
        
        config.clear();

        nh.getParam("imu_angular_velocity_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 3);
        for (int i = 0; i < 3; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> imu_angular_velocity_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }

        config.clear();

        nh.getParam("imu_linear_acceleration_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 3);
        for (int i = 0; i < 3; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> imu_linear_acceleration_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }

        config.clear();

        nh.getParam("odom_pose_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 6);
        for (int i = 0; i < 6; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> odom_pose_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }

        config.clear();

        nh.getParam("odom_twist_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 6);
        for (int i = 0; i < 6; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> odom_twist_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }

        config.clear();

        nh.getParam("slam_pose_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 6);
        for (int i = 0; i < 6; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> slam_pose_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }

       config.clear();

        nh.getParam("QR_pose_covariance", config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray && config.size() == 6);
        for (int i = 0; i < 6; i++)
        {
            try
            {
            std::ostringstream ostr;
            ostr << config[i];
            std::istringstream istr(ostr.str());
            istr >> QR_pose_covariance[i];
            }
            catch(XmlRpc::XmlRpcException &e){throw e;}
            catch(...){throw;}
        }

       setted_pose = false;
        bufferptr_ = std::make_unique<tf2_ros::Buffer>();
        tfListenerptr_ = std::make_shared<tf2_ros::TransformListener>(*bufferptr_);
    }

    ~sub_pub(){}

    // void pub_tf(){
    //     if(ok){
    //         odom_broadcaster.sendTransform(tf);
    //         ok = false;
    //     }
    //}

private:
    void imuCb(const sensor_msgs::Imu::Ptr &msg);
    void odomCb(const nav_msgs::Odometry::Ptr &msg);
    void poseCb(const geometry_msgs::PoseStamped::Ptr &msg);
    void QRCb(const serial_test::QRLocalization::Ptr &msg);


    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_QR;
    ros::Publisher pub_imu;
    ros::Publisher pub_odom;
    ros::Publisher pub_pose;
    ros::Publisher pub_QR;
    ros::Publisher pub_odominworld;
    ros::ServiceClient client;
    double imu_orientation_covariance[3];
    double imu_angular_velocity_covariance[3];
    double imu_linear_acceleration_covariance[3];
    double odom_pose_covariance[6];
    double odom_twist_covariance[6];
    double slam_pose_covariance[6];
    double QR_pose_covariance[6];
    bool setted_pose; //设置了初始位姿
    bool enable_odom2base_publisher;
    bool enable_set_pose;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped tf;
    //Eigen::Matrix<long double,3,1> sum{0,0,0};
    //Eigen::Matrix<long double,3,1> means;
    std::unique_ptr<tf2_ros::Buffer> bufferptr_;
    std::shared_ptr<tf2_ros::TransformListener> tfListenerptr_;
};

void sub_pub::imuCb(const sensor_msgs::Imu::Ptr &msg)
{  
    msg->header.frame_id = "imu";
    //姿态的协方差矩阵
    msg->orientation_covariance[0] = imu_orientation_covariance[0];
    msg->orientation_covariance[4] = imu_orientation_covariance[1];
    msg->orientation_covariance[8] = imu_orientation_covariance[2];
    //角速度的协方差矩阵
    msg->angular_velocity_covariance[0] = imu_angular_velocity_covariance[0];
    msg->angular_velocity_covariance[4] = imu_angular_velocity_covariance[1];
    msg->angular_velocity_covariance[8] = imu_angular_velocity_covariance[2];
    //线加速度的协方差矩阵
    msg->linear_acceleration_covariance[0] = imu_linear_acceleration_covariance[0];
    msg->linear_acceleration_covariance[4] = imu_linear_acceleration_covariance[1];
    msg->linear_acceleration_covariance[8] = imu_linear_acceleration_covariance[2];

    //std::cout <<"=======r,p,y======="<<std::endl;
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    //double r,p,y;
    //tf2::Matrix3x3 m_q(q);
    //m_q.getRPY(r,p,y);
    //std::cout << "纠正前：" << r*180/M_PI <<","<< p*180/M_PI <<","<< y*180/M_PI << std::endl;

    //i=i+1.0;Eigen::Matrix<long double,3,1> now{r,p,y};
    //sum = sum + now;
    //means = sum/i;
    //std::cout << "平均：" << means.transpose() << std::endl;
    

    //imu的姿态有问题，安装位置有关，大致估计了一个偏差角度
    tf2::Quaternion correct;
    correct.setRPY(0.00821609,0.001269,0.00109691);//水平系下的偏差角度
    //correct.setRPY(M_PI/1800,M_PI/900,M_PI/600);
    tf2::Quaternion corrected_qA = (correct.inverse()*q).normalize();
    //原本是水平系下的位姿，由于安装误差，想转为IMU传感系中.
    //tf2::Matrix3x3 corrected_qA_M(corrected_qA);
    //corrected_qA_M.getRPY(r,p,y);
    //std::cout << "纠正后:" << r*180/M_PI <<","<< p*180/M_PI <<","<< y*180/M_PI << std::endl;

    //有两个IMU坐标系，原本的位姿是A系下的，现在想转为B系下的
    //A绕z轴旋转90度，得到B系
    //先绕B坐标系的z轴转-90度，到A坐标系中，再根据A坐标系下得到的IMU位姿进行变换，再绕当前坐标系z轴旋转90度，到B坐标系中的IMU位姿表示
    tf2::Quaternion AtoBtrans;
    AtoBtrans.setRPY(0.0, 0.0, M_PI/2);
    tf2::Quaternion corrected_qB =  AtoBtrans.inverse() * corrected_qA *  AtoBtrans;
    //tf2::Matrix3x3 corrected_qB_M(corrected_qB);
    //corrected_qB_M.getRPY(r,p,y);
    //std::cout << "旋转90度后:" << r*180/M_PI <<","<< p*180/M_PI <<","<< y*180/M_PI << std::endl;
    
    // Eigen::Quaterniond q_test(correct_q.getW(),correct_q.getX(),correct_q.getY(),correct_q.getZ());
    // Eigen::Quaterniond q_test1(sqrt(2)/2 , 0 , 0 , sqrt(2)/2);
    // q_test1.normalize();
    // q_test.normalize();
    // Eigen::Quaterniond qqq=q_test1.inverse()*q_test*q_test1;
    // tf2::Quaternion q_test2(qqq.x(),qqq.y(),qqq.z(),qqq.w());
    // tf2::Matrix3x3 euler(q_test2);
    // euler.getRPY(r,p,y);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    // std::cout << "旋转90度后test1：" << r*180/M_PI <<","<< p*180/M_PI <<","<< y*180/M_PI << std::endl;


    //Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(10*M_PI/180,Eigen::Vector3d::UnitX()));
    //Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(20*M_PI/180,Eigen::Vector3d::UnitY()));
    //Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(30*M_PI/180,Eigen::Vector3d::UnitZ()));
    //Eigen::Quaterniond QQ_cor = yawAngle*pitchAngle*rollAngle;
    //QQ_cor.normalize();
    //把corrected_qB_M转为Geometry_msgs::Quaternion
    geometry_msgs::Quaternion msg_q;
    tf2::convert(corrected_qB,msg_q);
    msg->orientation = msg_q;


    //修改角速度
    //std::cout <<"=======vel:r,p,y======="<<std::endl;
    tf2::Vector3 V{msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z};
    V = tf2::quatRotate(correct.inverse(),V);
    //std::cout <<V1.getX()<< "," << V1.getY() <<","<< V1.getZ() <<std::endl;
    V = tf2::quatRotate(AtoBtrans,V);
    //std::cout <<V1.getX()<< "," << V1.getY() <<","<< V1.getZ() <<std::endl;
    geometry_msgs::Vector3 msg_v;
    tf2::convert(V,msg_v);
    msg->angular_velocity = msg_v;

    // //修改加速度
    // std::cout <<"=======acc:x,y,z======="<<std::endl;
    tf2::Vector3 A{msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
    A = tf2::quatRotate(correct.inverse(),A);
    A = tf2::quatRotate(AtoBtrans,A);
    geometry_msgs::Vector3 msg_a;
    tf2::convert(A,msg_a);
    msg->linear_acceleration = msg_a;

    pub_imu.publish(msg);
}

void sub_pub::odomCb(const nav_msgs::Odometry::Ptr &msg)
{  
    // msg->child_frame_id="base_link";
    // msg->pose.covariance[0] = odom_pose_covariance[0];
    // msg->pose.covariance[7] = odom_pose_covariance[1];
    // msg->pose.covariance[14]= odom_pose_covariance[2];
    // msg->pose.covariance[21]= odom_pose_covariance[3];
    // msg->pose.covariance[28]= odom_pose_covariance[4];
    // msg->pose.covariance[35] = odom_pose_covariance[5];
    // msg->twist.covariance[0] = odom_twist_covariance[0];
    // msg->twist.covariance[7] = odom_twist_covariance[1];
    // msg->twist.covariance[14]= odom_twist_covariance[2];
    // msg->twist.covariance[21]= odom_twist_covariance[3];
    // msg->twist.covariance[28]= odom_twist_covariance[4];
    // msg->twist.covariance[35] = odom_twist_covariance[5];
    // //msg->pose.pose.position.z=0.0;
    // pub_odom.publish(msg);
   
    if(enable_odom2base_publisher)
    {
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = msg->header.frame_id;//odom
        tf.child_frame_id = msg->child_frame_id;//base_link
        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.w = msg->pose.pose.orientation.w;
        tf.transform.rotation.x = msg->pose.pose.orientation.x;
        tf.transform.rotation.y = msg->pose.pose.orientation.y;
        tf.transform.rotation.z = msg->pose.pose.orientation.z;
        odom_broadcaster.sendTransform(tf);
    }  
    try
    {
        tf2::Transform targetFrameTrans;
        tf2::fromMsg(bufferptr_->lookupTransform("map","odom",ros::Time(0)).transform,
                   targetFrameTrans);
        ROS_WARN_STREAM_THROTTLE(2.0, "Using latest.\n");
        tf2::Transform sourceFrameTrans;
        tf2::convert(msg->pose.pose,sourceFrameTrans);
        tf2::Transform odom_in_map;
        odom_in_map.mult(targetFrameTrans,sourceFrameTrans);
        nav_msgs::Odometry pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = "map";
        pose.pose.pose.position.x = odom_in_map.getOrigin().getX();
        pose.pose.pose.position.y = odom_in_map.getOrigin().getY();
        pose.pose.pose.position.z = odom_in_map.getOrigin().getZ();
        tf2::convert(odom_in_map.getRotation(),pose.pose.pose.orientation);
        pub_odominworld.publish(pose);
        
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN_STREAM_THROTTLE(2.0, "Error was " << ex.what() << "\n");
    }

}

void sub_pub::poseCb(const geometry_msgs::PoseStamped::Ptr &msg)
{  

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header=msg->header;
    pose.pose.pose=msg->pose;
    //pose.pose.pose.position.x = pose.pose.pose.position.x;
    //pose.pose.pose.position.y = pose.pose.pose.position.y;
    pose.pose.covariance[0] = slam_pose_covariance[0];
    pose.pose.covariance[7] = slam_pose_covariance[1];
    pose.pose.covariance[14] = slam_pose_covariance[2];
    pose.pose.covariance[21] = slam_pose_covariance[3];
    pose.pose.covariance[28] = slam_pose_covariance[4];
    pose.pose.covariance[35] = slam_pose_covariance[5];
    //slam做真值，因为slam世界坐标系跟二维码世界坐标系不一样，所以要转换
    tf2::Vector3 origin;
    origin.setX(15.7511- 0.0175);
    origin.setY(3.0538 + 0.0138);
    origin.setZ(0.0);
    tf2::Transform QRMapToSLAMMaptf;
    QRMapToSLAMMaptf.setOrigin(origin);
    tf2::Quaternion q;
    q.setRPY(0,0,-3.9199);
    QRMapToSLAMMaptf.setRotation(q);
    tf2::Transform SLAMpose;
    tf2::convert(pose.pose.pose,SLAMpose);
    tf2::Transform QRMappose = QRMapToSLAMMaptf.inverse() * SLAMpose;
    pose.pose.pose.position.x = QRMappose.getOrigin().getX();
    pose.pose.pose.position.y = QRMappose.getOrigin().getY();
    pose.pose.pose.position.z = QRMappose.getOrigin().getZ();
    tf2::convert(QRMappose.getRotation() , pose.pose.pose.orientation);
    //为了更方便观察，我转为了欧拉角，弧度制，只用四元数为载体。
    double d,y;
    tf2::Matrix3x3 m(QRMappose.getRotation());m.getRPY(d,d,y);
    pose.pose.pose.position.z = y;
    pub_pose.publish(pose);


    //之前用slam位姿来初始化。现在用二维码了。
    //     //没有初始化位姿且要初始化位姿
    // if(!setted_pose && enable_set_pose)
    // {
    //     robot_localization::SetPose srv;
    //     srv.request.pose.pose.pose = pose.pose.pose;
    //     srv.request.pose.pose.covariance = pose.pose.covariance;
    //     srv.request.pose.header = pose.header;
    //     if(client.call(srv))
    //     {
    //         setted_pose = true;
    //         ROS_INFO("Set pose success");
    //     }
    // }

}

void sub_pub::QRCb(const serial_test::QRLocalization::Ptr &msg)
{
    msg->header.frame_id = "map";
    msg->child_frame_id = "base_link";


    if(!setted_pose && enable_set_pose)
    {
        robot_localization::SetPose srv;
        srv.request.pose.pose.pose = msg->pose.pose;
        srv.request.pose.pose.covariance = msg->pose.covariance;
        srv.request.pose.header = msg->header;
        if(client.call(srv))
        {
            setted_pose = true;
            ROS_INFO("Set pose success");
        }
    }

    // msg->pose2d.theta = 0.0;//录制时候出问题了，只能在这里修改。
    // double base_x = msg->pose.pose.position.x - msg->pose2d.x;
    // //std::cout << base_x << " = " << msg->pose.pose.position.x << " - " << msg->pose2d.x << std::endl;
    // double base_y = msg->pose.pose.position.y - msg->pose2d.y;
    // //std::cout << base_y << " = " << msg->pose.pose.position.y << " - " << msg->pose2d.y << std::endl;
    // msg->pose.pose.position.x = base_x;
    // msg->pose.pose.position.y = base_y;
    // // msg->pose.covariance[0] = QR_pose_covariance[0];
    // // msg->pose.covariance[7] = QR_pose_covariance[1];
    // // msg->pose.covariance[14] = QR_pose_covariance[2];
    // // msg->pose.covariance[21] = QR_pose_covariance[3];
    // // msg->pose.covariance[28] = QR_pose_covariance[4];
    // // msg->pose.covariance[35] = QR_pose_covariance[5];
    // pub_QR.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_data");
  sub_pub sub_pub;
    ros::Time::now();
    ros::spin();
    return 0;
}

