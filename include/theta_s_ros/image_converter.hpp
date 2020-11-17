#ifndef __IMAGE_CONVERTER_HPP
#define __IMAGE_CONVERTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageConverter {
public:
    ImageConverter();
    void image_callback(const sensor_msgs::ImageConstPtr&);
    void process();
private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber image_sub;
    ros::Publisher image_pub;
};

#endif // __IMAGE_CONVERTER_HPP