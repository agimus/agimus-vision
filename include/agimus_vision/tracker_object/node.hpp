#ifndef __TRACKER_BOX__NODE__HPP__
#define __TRACKER_BOX__NODE__HPP__

#include <mutex>
#include <string>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <agimus_vision/TrackerConfig.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>

#include "agimus_vision/tracker_object/fwd.hpp"
#include "agimus_vision/tracker_object/tracker.hpp"
#include "agimus_vision/tracker_object/detector_apriltag.hpp"

#include "agimus_vision/AddAprilTagService.h"
#include "agimus_vision/AddObjectTracking.h"
#include "agimus_vision/SetChessboardService.h"

class vpDisplay;
namespace image_transport { class Publisher; }

namespace agimus_vision {
namespace tracker_object {

/// Agimus vision ROS node.
///
/// - It advertises the services
///   \c add_april_tag_detector (See Node::addAprilTagService)
///   and \c add_object_tracking (See Node::addObjectTracking).
/// - It publishes to /tf the tag pose if ROS param \c broadcastTf is \c true.
///   In this case, the child node name is postfixed with ROS param \c broadcastTfPostfix
/// - It publishes to \c /agimus/vision/tags/tf the tag pose if ROS param \c broadcastTopic is \c true.
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
    vpImage<unsigned char> _gray_image;

    // Classes called to detect some object in the image and then track it
    std::shared_ptr<DetectorAprilTagWrapper> _aprilTagDetector;
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
    std::vector<Tracker> _trackers;
    
    std::unique_ptr <vpDisplay> _debug_display;
    std::unique_ptr <image_transport::Publisher> _debug_publisher;

    bool _broadcast_tf;
    bool _broadcast_topic;

    std::vector< ros::ServiceServer > _services;
    ros::Publisher _publisherVision;
    ros::Publisher _detection_publisher;
    dynamic_reconfigure::Server<TrackerConfig> tracker_reconfigure;

public:
    Node();

    ~Node();

    /// Callback to update the camera information
    /// \todo the camera parameters should be propagated to the downstream algos.
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

    /// Callback to use the images
    void frameCallback(const sensor_msgs::ImageConstPtr& image);

    /// Reconfiguration callback
    void trackerReconfigureCallback(TrackerConfig &config, uint32_t level);

    /// Process an image.
    void imageProcessing ();
    
    /// Add an April tag
    /// \include srv/AddAprilTagService.srv
    bool addAprilTagService( agimus_vision::AddAprilTagService::Request  &req,
                             agimus_vision::AddAprilTagService::Response &res );

    /// Add object tracking service
    /// \include srv/AddObjectTracking.srv
    bool addObjectTracking( agimus_vision::AddObjectTracking::Request  &req,
                            agimus_vision::AddObjectTracking::Response &res );

    /// Setup detection of a chessboard
    /// \include srv/SetChessboardService.srv
    bool setChessboardService( agimus_vision::SetChessboardService::Request  &req,
                               agimus_vision::SetChessboardService::Response &res );

    /// Reset the tag poses
    bool resetTagPosesService( std_srvs::Trigger::Request  &req,
                               std_srvs::Trigger::Response &res );

    void initAprilTagDetector ();
    
    void spin();
};

}
}

#endif // __VISP_TRACKER_NODE_HPP__
