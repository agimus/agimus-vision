#ifndef __VISP_TRACKER_NODE_HPP__
#define __VISP_TRACKER_NODE_HPP__

#include <mutex>
#include <string>
#include <memory>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>

#include <visp3/blob/vpDot2.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>


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

    // Images and parameters taken from the camera's topics
    std::mutex _image_lock;
    vpCameraParameters _camera_param;
    std_msgs::Header _image_header;
    vpImage<vpRGBa> _image;
    vpImage<unsigned char> _gray_image;
    bool _image_new;

    // AprilTag coordinates
    std::vector< vpPoint > _april_tag_points;
    // April tag detector and computed pose
    vpDetectorAprilTag _april_tag_detector;
    vpHomogeneousMatrix _cMo_tag;
    bool _new_cMo_tag;

    // Tracker, its settings and the computed pose
    vpMbEdgeKltTracker _edge_klt_tracker;
    vpMe _me;
    vpKltOpencv _klt_settings;
    vpHomogeneousMatrix _cMo_tracker;
    bool _new_cMo_tracker;


    bool _debug_display;

    // Wait for the first available image
    void waitForImage();

    // Search the image to find an AprilTag with the specified id
    int aprilTagSearch( int id );

    // Init the tracker with the AprilTag found previously
    void initTracking( int id );
    
    // Compute the pose of the tag from its 3D points, 2D image's points and camera parameters
    void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                     bool init, vpHomogeneousMatrix &cMo);

    // Publish the object position rt the camera
    void publish_pose();

public:
    Node();

    // Callbacks to use the images
    void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);

    // Method to send info to ROS

    void spin();
};

#endif // __VISP_TRACKER_NODE_HPP__