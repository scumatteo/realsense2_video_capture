#include <realsense2_video_capture/video_capture_node.h>

VideoCaptureNode::VideoCaptureNode() : _camera_sync(RGBDCameraSyncPolicy(this->_QUEUE_SIZE), this->_raw_image_sub, this->_depth_image_sub)
{
    sensor_msgs::CameraInfo camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/aligned_depth_to_color/camera_info");

    //topic subscription
    this->_raw_image_sub.subscribe(this->_node_handle, "/camera/color/image_raw", this->_QUEUE_SIZE);
    this->_depth_image_sub.subscribe(this->_node_handle, "/camera/aligned_depth_to_color/image_raw", this->_QUEUE_SIZE);

    //registration of sync callback
    this->_camera_sync.registerCallback(boost::bind(&VideoCaptureNode::onCameraMessage, this, _1, _2));
}

void VideoCaptureNode::onCameraMessage(const sensor_msgs::ImageConstPtr &bgr, const sensor_msgs::ImageConstPtr &depth)
{

    cv_bridge::CvImagePtr cv_bgr_ptr;
    cv_bridge::CvImagePtr cv_depth_ptr;

    try
    {

        //create raw image
        cv_bgr_ptr = cv_bridge::toCvCopy(bgr, sensor_msgs::image_encodings::BGR8); //convert to CvImage
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }
    try
    {

        //create depth image
        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1); //convert to CvImage
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge DEPTH exception: %s", e.what());
        return;
    }

    cv::Mat bgr_image = cv_bgr_ptr->image;
    cv::Mat depth_image = cv_depth_ptr->image;

    std::string rgb_path = _root_path + "sequence_1/rgb/" + std::to_string(cv_bgr_ptr->header.stamp.toNSec()) + ".png";
    std::string depth_path = _root_path + "sequence_1/depth/" + std::to_string(cv_depth_ptr->header.stamp.toNSec()) + ".png";
    cv::imwrite(rgb_path, bgr_image);
    cv::imwrite(depth_path, depth_image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_capture_node");
    VideoCaptureNode node;
    ros::spin();
    return 0;
}