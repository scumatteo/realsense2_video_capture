#pragma once

#ifndef VIDEO_CAPTURE_NODE_H
#define VIDEO_CAPTURE_NODE_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> RGBDCameraSyncPolicy;

class VideoCaptureNode
{
public:
    VideoCaptureNode();
    void onCameraMessage(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth);

private:
    static const int _QUEUE_SIZE = 1; //queue size for the subscribers

    //raw and depth images subscribers
    message_filters::Subscriber<sensor_msgs::Image> _raw_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_image_sub;

    ros::NodeHandle _node_handle;

    //synchronizer for raw and depth images
    message_filters::Synchronizer<RGBDCameraSyncPolicy> _camera_sync;
};

#endif