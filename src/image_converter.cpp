#include "theta_s_ros/image_converter.hpp"

#include "opencv_theta_s/EquirectangularConversion/ThetaConversion.hpp"

ImageConverter::ImageConverter() : private_nh("~") {
    image_sub = nh.subscribe("/camera/image_raw", 1, &ImageConverter::image_callback, this);
    image_pub = nh.advertise<sensor_msgs::Image>("/equirectangular/image_raw", 1);
}

void ImageConverter::image_callback(const sensor_msgs::ImageConstPtr& received_image) {
    ROS_DEBUG("start image_callback");
    double start_time = ros::Time::now().toSec();

    ROS_DEBUG("ros image to cv image");
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& ex) {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }
    cv::Mat cv_image(cv_image_ptr->image.rows, cv_image_ptr->image.cols, cv_image_ptr->image.type());
    cv_image = cv_image_ptr->image;

    ROS_DEBUG("fisheye image to equirectangular image");
    ThetaConversion(cv_image.cols, cv_image.rows).doConversion(cv_image);

    ROS_DEBUG("cv image to output ros image");
    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    output_image->header = received_image->header;
    image_pub.publish(output_image);

    ROS_INFO("[image_converter:image_callback] elapsed time : %f [sec]", ros::Time::now().toSec() - start_time);

    return;
}

void ImageConverter::process() {
    ROS_DEBUG("start process");
    ros::spin();
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "convert_ros_image_to_cv_image");
    ImageConverter image_convertor;
    image_convertor.process();
    return 0;
}
