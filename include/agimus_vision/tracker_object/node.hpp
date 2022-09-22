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

//Use message filters to sync color and depth image 
#include "tf/message_filter.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// #include <visp3/core/vpConfig.h>
// #ifdef VISP_HAVE_MODULE_SENSOR
// #include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpLaserScanner.h>
// #endif
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/vision/vpPose.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImagePoint.h>


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
    std::string _depth_image_topic;
    std::string _camera_info_topic;
    std::string _depth_camera_info_topic;


    // Some params related to depth image info
    float _depth_scale_param; // to use on Visp's compute Pose using RGBD info
    float _depth_rgb_distance_param; // distance btw depth and rgb sensor
    // ros::Subscriber _image_sub;
    // ros::Subscriber _depth_image_sub;

    //Change to add depth
    message_filters::Subscriber<sensor_msgs::Image> _image_sub;      
    message_filters::Subscriber<sensor_msgs::Image> _deph_image_sub; 
    // ros::Subscriber _depth_image_sub;
    // ros::Subscriber _image_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    
    sensor_msgs::ImageConstPtr rgbImage;
    sensor_msgs::ImageConstPtr depthImage_;

    
    //end change to add depth


    ros::Subscriber _camera_info_sub;
    ros::Subscriber _depth_camera_info_sub;
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
    vpCameraParameters _depth_cam_parameters;
    std_msgs::Header _image_header;
    vpImage<unsigned char> _gray_image;
    vpImage<uint16_t> _depth_image;
    vpImage<unsigned char> _depth_image_8bit;
    
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
    void depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& depth_camera_info);

    /// Callback to use the images
    // void frameCallback(const sensor_msgs::ImageConstPtr &image);
    void frameCallback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &depth_image);

     /// Callback to use the depth images
    // void depthFrameCallback(const sensor_msgs::ImageConstPtr& image);

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

    vpImage<uint16_t> toVispImageFromDepth(const sensor_msgs::Image& src);
    vpImage<uint16_t> toVispImageFromCVDepth(cv::Mat &depthImage);
};

}
}

#endif // __VISP_TRACKER_NODE_HPP__
