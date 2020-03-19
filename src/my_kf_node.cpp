#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

double dt_acc = 0.1, dt_uwb = 0.1;//取样间隔
Eigen::Vector3d acc_odom, uwb_measurement;
ros::Time filter_time_old_;
Eigen::Matrix3d Q_uwb, R_acc_odom;

bool isAcc2Update = false, isUwb2Update = false;

void acc_update(const sensor_msgs::ImuConstPtr& imu_data){
    static bool isAccInited = false;
    static ros::Time old_acc_stamp;//上一次加速度计的时间戳
    static Eigen::Vector3d s_previous, s_now, a_previous, a_now, vel_previous, vel_now;
    //如果是第一次接受到数据(默认：机器人的初始状态为静止)
    if(!isAccInited){
        //仅记下数据，并记为已初始化；记下时间戳
        isAccInited = true;
        s_previous << 0, 0, 0;
        vel_previous << 0, 0, 0;
        a_previous << imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z;
        R_acc_odom << imu_data->linear_acceleration_covariance[0], 0, 0,
                      0, imu_data->linear_acceleration_covariance[4], 0,
                      0, 0, imu_data->linear_acceleration_covariance[8];
        old_acc_stamp = imu_data->header.stamp;
    }else{//如果不是第一次接收到数据
        //判断时间戳，看是否超过dt
        if((imu_data->header.stamp - old_acc_stamp).toSec() > dt_acc){
            //如果超过：获取acc数据，跑公式，记录协方差矩阵
            a_now << imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z;
            R_acc_odom << imu_data->linear_acceleration_covariance[0], 0, 0,
                    0, imu_data->linear_acceleration_covariance[4], 0,
                    0, 0, imu_data->linear_acceleration_covariance[8];
            s_now << s_previous(1,1) + (vel_previous(1,1) + 0.25 * (a_now(1,1) + a_previous(1,1)) ) * dt_acc,
                     s_previous(2,1) + (vel_previous(2,1) + 0.25 * (a_now(2,1) + a_previous(2,1)) ) * dt_acc,
                     s_previous(3,1) + (vel_previous(3,1) + 0.25 * (a_now(3,1) + a_previous(3,1)) ) * dt_acc;
            vel_now << vel_previous(1,1) + (a_now(1,1) + a_previous(1,1)) * 0.5 * dt_acc,
                    vel_previous(2,1) + (a_now(2,1) + a_previous(2,1)) * 0.5 * dt_acc,
                    vel_previous(3,1) + (a_now(3,1) + a_previous(3,1)) * 0.5 * dt_acc;
//            s_now = s_previous + vel_previous + 0.25 * ( a_now + a_previous ) * dt;
//            vel_now = vel_previous + 0.5 * ( a_now + a_previous ) * dt;
            //返回状态
            acc_odom = s_now;
            //状态更换
            vel_previous = vel_now;
            s_previous = s_now;
            a_previous = a_now;
            //更新时间戳,更新isAcc2Update
            isAcc2Update = true;
            old_acc_stamp = imu_data->header.stamp;
        }else{

        }
        return;

    }



}

void uwb_update(const nav_msgs::OdometryConstPtr& uwb_data){
    static bool isUwbInited = false;
    static ros::Time old_uwb_stamp;//上一次uwb的时间戳
    if(isUwbInited == false){
        isUwbInited = true;
        old_uwb_stamp = uwb_data->header.stamp;
    }else{
        //协方差
        Q_uwb << uwb_data->pose.covariance.at(0), 0, 0,
                 0, uwb_data->pose.covariance.at(7), 0,
                 0, 0, uwb_data->pose.covariance.at(14);
        //位置数据
        uwb_measurement << uwb_data->pose.pose.position.x, uwb_data->pose.pose.position.y, uwb_data->pose.pose.position.z;
        //完成数据更新
        old_uwb_stamp = uwb_data->header.stamp;
        isUwb2Update = true;
    }
    return;
}

int main (int argc, char **argv){
    ros::init (argc, argv, "my_kf_node");

    ros::NodeHandle nh;

    //订阅加速度数据
    std::string acc_topic;
    ros::Subscriber acc_subscriber = nh.subscribe(acc_topic, 1, acc_update);

    //订阅UWB数据
    std::string uwb_topic;
    ros::Subscriber uwb_subscriber = nh.subscribe(uwb_topic, 1, uwb_update);

    //状态转移矩阵A， 控制矩阵B， 观测矩阵C
    //X_hat(k+1) = AX_(k) + Bu_(k)
    //P_hat(k+1) = A*P_(k)*A'+ R_(k+1)
    //K_(k+1) = P_hat(k+1)*C'/(C*P_hat(k+1)*C' + Q_(k+1))
    //X_(k+1) = X_hat(k+1) + K_(k+1) * (z_(k+1) - C*X_hat(k+1))
    //P_(k+1) = (I - K_(k+1))*P_hat(k+1)
    Eigen::Matrix3d A, B, C, P, P_hat;

    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    B << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    C << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    P << 0, 0, 0,
         0, 0, 0,
         0, 0, 0;

    P_hat = P;
    //KF
    Eigen::Vector3d X_hat, X;
    X_hat << 0,0,0;
    X = X_hat;

    while(ros::ok()){

        if(isAcc2Update){//运动预测部分
            X_hat = A * X + B * acc_odom;
            P_hat = A * P * A.transpose() + R_acc_odom;
            X = X_hat;//方便返回
            isAcc2Update = false;
        }
        if(isUwb2Update){//测量更新部分
            Eigen::Matrix3d K;
            K = P_hat * C.transpose() * (C * P_hat * C.transpose() + Q_uwb).inverse();
            X = X_hat + K * (uwb_measurement - K * C * X_hat);
            P = (Eigen::Matrix3d::Identity() - K * C) * P_hat;
            isUwb2Update = false;
        }

    }
    return 0;
}