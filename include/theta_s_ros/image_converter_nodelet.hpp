#ifndef __IMAGE_CONVERTER_NODELET_HPP
#define __IMAGE_CONVERTER_NODELET_HPP

#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include "opencv_theta_s/EquirectangularConversion/ThetaConversion.hpp"

namespace theta_s_ros {
class ImageConverterNodelet : public nodelet::Nodelet {
 public:
    virtual void onInit();
    void image_callback(const sensor_msgs::ImageConstPtr&);

 private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber image_sub;
    ros::Publisher image_pub;
};
}  // namespace theta_s_ros

#endif  // __IMAGE_CONVERTER_NODELET_HPP
