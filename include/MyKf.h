//
// Created by ch on 20-3-18.
//

#ifndef MY_KF_MYKF_H
#define MY_KF_MYKF_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <vector>

class MyKf {
public:
    std::vector<double> getPose();//返回kf得到的pose
    bool isOK2GetPose();//能否获取位置信息
private:
    bool accOdomUpdate();
    void
};


#endif //MY_KF_MYKF_H
