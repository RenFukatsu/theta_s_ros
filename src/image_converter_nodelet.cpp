#include "theta_s_ros/image_converter_nodelet.hpp"

namespace theta_s_ros {
void ImageConverterNodelet::onInit() {
    NODELET_INFO_STREAM("ImageConverterNodelet Init");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    image_sub = nh.subscribe("/camera/image_raw", 1, &ImageConverterNodelet::image_callback, this);
    image_pub = nh.advertise<sensor_msgs::Image>("/equirectangular/image_raw", 1);
}

void ImageConverterNodelet::image_callback(const sensor_msgs::ImageConstPtr& received_image) {
    NODELET_INFO("start image_callback");
    double start_time = ros::Time::now().toSec();

    NODELET_INFO("ros image to cv image");
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& ex) {
        NODELET_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }
    cv::Mat cv_image(cv_image_ptr->image.rows, cv_image_ptr->image.cols, cv_image_ptr->image.type());
    cv_image = cv_image_ptr->image;

    NODELET_INFO("fisheye image to equirectangular image");
    ThetaConversion(cv_image.cols, cv_image.rows).doConversion(cv_image);

    NODELET_INFO("cv image to output ros image");
    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    output_image->header = received_image->header;
    image_pub.publish(output_image);

    NODELET_INFO("elapsed time : %f [sec]", ros::Time::now().toSec() - start_time);

    return;
}
} // namespace theta_s_ros
PLUGINLIB_EXPORT_CLASS(theta_s_ros::ImageConverterNodelet, nodelet::Nodelet)
