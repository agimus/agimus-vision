#ifndef __TRACKER_BOX__NODE__HPP__
#define __TRACKER_BOX__NODE__HPP__

#include "agimus_vision/tracker_object/detector_apriltag.hpp"

#include "agimus_vision/AddAprilTagService.h"
#include "agimus_vision/SetChessboardService.h"

#include <mutex>
#include <string>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>

class vpDisplayX;

namespace agimus_vision {
namespace tracker_object {

/// Tracking of April tag.
///
/// - It advertises a service "add_april_tag_detector" (See Node::addAprilTagService).
/// - It publishes to /tf the tag pose if ROS param "broadcastTf" is \c true.
///   In this case, the child node name is postfixed with ROS param "broadcastTfPostfix".
/// - It publishes to /agimus/vision/tags/tf the tag pose if ROS param "broadcastTopic" is \c true.
class Node
{
    ros::NodeHandle _node_handle;

    // Names of the topics sending the images and infos
    std::string _image_topic;
    std::string _camera_info_topic;
    
    ros::Subscriber _image_sub;
    ros::Subscriber _camera_info_sub;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    // Names of the publication TF nodes   
    std::string _tf_camera_node;
    std::string _tf_node;
    std::string _broadcast_tf_postfix;

    // Images and parameters taken from the camera's topics
    std::mutex _image_lock;
    std::mutex _cam_param_lock;
    vpCameraParameters _cam_parameters;
    std_msgs::Header _image_header;
    vpImage<vpRGBa> _image;
    vpImage<unsigned char> _gray_image;
    bool _image_new;

    // Classes called to detect some object in the image and then track it
    struct DetectorAndName {
      DetectorPtr detector;
      /// Name of the parent node
      std::string parent_name;
      /// Name of the detected object's node
      std::string object_name;

      DetectorAndName (const DetectorPtr& d, const std::string& pn, const std::string& on)
        : detector(d), parent_name(pn), object_name(on)
      {}
    };
    std::map< int, DetectorAndName > _detectors;
    //std::unique_ptr< Tracker > _tracker;
    
    std::unique_ptr <vpDisplayX> _debug_display;
    bool _broadcast_tf;
    bool _broadcast_topic;

    // Wait for the first available image
    void waitForImage();


    // Init the tracker with the AprilTag found previously
    void initTracking( int id );
    
    std::vector< ros::ServiceServer > _services;
    ros::Publisher _publisherVision;
    ros::Publisher _detection_publisher;

public:
    Node();

    ~Node();

    /// Callback to update the camera information
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

    /// Callback to use the images
    void frameCallback(const sensor_msgs::ImageConstPtr& image);

    /// Process an image.
    void imageProcessing ();
    
    /// Add an April tag
    /// \include srv/AddAprilTagService.srv
    bool addAprilTagService( agimus_vision::AddAprilTagService::Request  &req,
                             agimus_vision::AddAprilTagService::Response &res );

    /// Setup detection of a chessboard
    /// \include srv/SetChessboardService.srv
    bool setChessboardService( agimus_vision::SetChessboardService::Request  &req,
                               agimus_vision::SetChessboardService::Response &res );

    /// Reset the tag poses
    bool resetTagPosesService( std_srvs::Trigger::Request  &req,
                               std_srvs::Trigger::Response &res );
    
    void spin();
};

}
}

#endif // __VISP_TRACKER_NODE_HPP__
