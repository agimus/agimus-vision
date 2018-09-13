#ifndef __TRACKER_BOX__NODE__HPP__
#define __TRACKER_BOX__NODE__HPP__

#include "agimus_vision/tracker_object/detector_apriltag.hpp"

#include "agimus_vision/AddAprilTagService.h"

#include <mutex>
#include <string>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>


class Node
{
    ros::NodeHandle _node_handle;

    // Names of the topics sending the images and infos
    std::string _image_topic;
    std::string _camera_info_topic;
    
    uint32_t _queue_size;
    std::unique_ptr< message_filters::Subscriber<sensor_msgs::Image> > _image_sub;
    std::unique_ptr< message_filters::Subscriber<sensor_msgs::CameraInfo> > _camera_info_sub;
    std::unique_ptr< message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> > _image_info_sync;

    // Names of the publication TF nodes   
    std::string _tf_parent_node;
    std::string _tf_node;
    std::string _broadcast_tf_postfix;

    // Images and parameters taken from the camera's topics
    std::mutex _image_lock;
    vpCameraParameters _cam_parameters;
    std_msgs::Header _image_header;
    vpImage<vpRGBa> _image;
    vpImage<unsigned char> _gray_image;
    bool _image_new;

    // Classes called to detect some object in the image and then track it
    std::map< int, std::pair< DetectorAprilTag, std::string > > _detectors;
    //std::unique_ptr< Tracker > _tracker;
    
    bool _debug_display;
    bool _broadcast_tf;
    bool _broadcast_topic;

    // Wait for the first available image
    void waitForImage();


    // Init the tracker with the AprilTag found previously
    void initTracking( int id );
    
    // Publish the object position rt the camera to TF
    void publish_pose_topic( const vpHomogeneousMatrix &cMo, const std::string &node_name, const ros::Time &timestamp );
    void publish_pose_tf( const vpHomogeneousMatrix &cMo, const std::string &node_name, const ros::Time &timestamp );

    std::vector< ros::ServiceServer > _services;
    ros::Publisher _publisherVision;

public:
    Node();

    // Callbacks to use the images
    void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);
    
    bool addAprilTagService( agimus_vision::AddAprilTagService::Request  &req,
                             agimus_vision::AddAprilTagService::Response &res );

    void spin();
};

#endif // __VISP_TRACKER_NODE_HPP__