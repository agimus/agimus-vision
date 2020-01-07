#include "agimus_vision/tracker_object/node.hpp"
#include "agimus_vision/tracker_object/detector_apriltag.hpp"
#include "agimus_vision/tracker_object/detector_chessboard.hpp"

#include "agimus_vision/ImageDetectionResult.h"

#include <algorithm>
#include <functional>
#include <iostream>

#include <ros/package.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImagePoint.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/3dpose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace agimus_vision {
namespace tracker_object {


geometry_msgs::Transform getTransformAtTimeOrNewest (
    tf2_ros::Buffer& buf,
    const std::string& parent,
    const std::string& node,
    const ros::Time& stamp,
    bool& ok)
{
  static ros::Time origin(0);
  try {
    ok=true;
    return buf.lookupTransform(parent, node, stamp ).transform;
  } catch (const tf2::ExtrapolationException&) {
    ok=true;
    return buf.lookupTransform(parent, node, origin).transform;
  } catch (const tf2::TransformException& e) {
    ROS_WARN(e.what());
    ok=false;
    return geometry_msgs::Transform();
  }
}

using tf2::convert;

void convert (const vpHomogeneousMatrix &Min, tf2::Transform& out)
{
  tf2::convert(visp_bridge::toGeometryMsgsTransform(Min), out);
}

Node::Node()
 : _node_handle{}
 , _tf_buffer{}
 , _tf_listener{_tf_buffer}
 , _image_new{ false }
 , _debug_display{ nullptr }
{
    // Get parameters for the node
    _node_handle.param<std::string>("imageTopic", _image_topic, "/camera/rgb/image_rect_color");
    _node_handle.param<std::string>("cameraInfoTopic", _camera_info_topic, "/camera/rgb/camera_info");

    // Initialize camera parameters
    ROS_INFO_STREAM("Wait for camera info message on " << _camera_info_topic);
    sensor_msgs::CameraInfoConstPtr cam_info_msg =
      ros::topic::waitForMessage <sensor_msgs::CameraInfo> (_camera_info_topic,
          _node_handle);
    if (!cam_info_msg) return;
    cameraInfoCallback(cam_info_msg);
   
    // Use those parameters to create the camera 
    _image_sub = _node_handle.subscribe(_image_topic, 1,
        &Node::frameCallback, this);
    _camera_info_sub = _node_handle.subscribe(_camera_info_topic, 1,
        &Node::cameraInfoCallback, this);

    // TF node of the camera seeing the tags
    _node_handle.param<std::string>("cameraFrame", _tf_camera_node, "rgbd_rgb_optical_frame");

    // Display the tags seen by the camera
    if (_node_handle.param<bool>("debugDisplay", false))
        _debug_display.reset( new vpDisplayX{} );

    // Broadcasting methods
    _node_handle.param<bool>("broadcastTf", _broadcast_tf, false);
    _node_handle.param<std::string>("broadcastTfPostfix", _broadcast_tf_postfix, "");
    _node_handle.param<bool>("broadcastTopic", _broadcast_topic, false);
    
    // TODO: Switch for detector types and tracker init 
    std::string object_type{};
    _node_handle.param<std::string>( "objectType", object_type, "apriltag" );
    std::for_each( object_type.begin(), object_type.end(), [](char &c){ c = (char)::tolower( c ); } );

    _services.push_back( _node_handle.advertiseService( "reset_tag_poses", &Node::resetTagPosesService, this ) );

    if( object_type == "apriltag" )
    {
        DetectorAprilTag::Apriltag_detector.setAprilTagPoseEstimationMethod( vpDetectorAprilTag::vpPoseEstimationMethod::BEST_RESIDUAL_VIRTUAL_VS );

        _services.push_back( _node_handle.advertiseService( "add_april_tag_detector", &Node::addAprilTagService, this ) );
    }
    else if( object_type == "chessboard" )
    {
        _services.push_back( _node_handle.advertiseService( "set_chessboard_detector", &Node::setChessboardService, this ) );
    }

    _publisherVision = _node_handle.advertise< geometry_msgs::TransformStamped >( "/agimus/vision/tags", 100 );
    _detection_publisher = _node_handle.advertise< agimus_vision::ImageDetectionResult >( "/agimus/vision/detection", 100 );
}

Node::~Node()
{}

void Node::waitForImage()
{
    ros::Rate rate{50};
    while(ros::ok())
    {
        if(_image_new)
            return;
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_DELAYED_THROTTLE(10, "Waiting for images");
    }
}

void Node::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    std::lock_guard<std::mutex> lock(_cam_param_lock);
    _cam_parameters = visp_bridge::toVispCameraParameters(*camera_info);
}

void Node::frameCallback(const sensor_msgs::ImageConstPtr& image)
{
    std::unique_lock<std::mutex> lock(_image_lock, std::try_to_lock);
    if(!lock.owns_lock())
      return;
    if(_image_header.seq + 1 < image->header.seq) {
      // Delayed ignores the first _image_header which isn't initialized.
      ROS_INFO_DELAYED_THROTTLE(5, "Some images were dropped.");
    }

    _image_header = image->header;
    
    if(image->encoding != sensor_msgs::image_encodings::MONO8)
    {
      ROS_ERROR_STREAM("The input image must be grayscale.");
      ros::shutdown();
    } 

    _gray_image = visp_bridge::toVispImage(*image);
    _image_new = true;

    imageProcessing();
}

void Node::imageProcessing()
{
    ros::Time time_begin (ros::Time::now());
    static tf2_ros::TransformBroadcaster broadcaster;

    if( _debug_display )
    {
      if ( !_debug_display->isInitialised())
      {
        _debug_display->init(_image);
        vpDisplay::setTitle(_image, "Visual display");
      }
      vpDisplay::display(_image);
    }

    if( !_detectors.empty() && (_detectors.begin())->second.detector->analyseImage( _gray_image ) )
    {
      agimus_vision::ImageDetectionResult result;
      result.header = _image_header;

      auto timestamp = _image_header.stamp;

      for( auto &detector : _detectors )
      {
        const DetectorPtr& detector_ptr = detector.second.detector;
        const std::string& parent_name = detector.second.parent_name;
        const std::string& object_name = detector.second.object_name;

        if( detector_ptr->detect() )
        {
          // c: camera
          // o: object
          // p: parent
          tf2::Transform cMo;
          convert(detector_ptr->getLastCMO(), cMo);

          geometry_msgs::Transform cMo_msg;
          tf2::convert(cMo, cMo_msg);

          result.ids      .push_back (detector_ptr->id());
          result.residuals.push_back (detector_ptr->error());
          result.poses    .push_back (cMo_msg);

          bool ok;
          tf2::Transform pMc;
          convert(getTransformAtTimeOrNewest
              (_tf_buffer, parent_name, _tf_camera_node, timestamp, ok),
              pMc);
          if (!ok) continue;

          geometry_msgs::TransformStamped pMo_msg;
          tf2::convert(pMc * cMo, pMo_msg.transform);

          pMo_msg.child_frame_id = object_name;
          pMo_msg.header.frame_id = parent_name;
          pMo_msg.header.stamp = timestamp;

          if( _broadcast_topic )
            _publisherVision.publish( pMo_msg );
          pMo_msg.child_frame_id += _broadcast_tf_postfix;
          if( _broadcast_tf )
            broadcaster.sendTransform( pMo_msg );

          if( _debug_display )
            detector_ptr->drawDebug( _image );
        }
      }
      if (!result.ids.empty())
        _detection_publisher.publish(result);

      ros::Time time_end (ros::Time::now());
      ros::Duration delay = time_end - timestamp;
      if (delay > ros::Duration(_node_handle.param<double>("max_delay", 0.3)))
        ROS_WARN_STREAM("Image " << _image_header.seq << "\n"
            "Input delay     : " << time_begin - timestamp << "\n"
            "Computation time: " << time_end - time_begin << "\n"
            "Output delay    : " << delay);
    }

    if( _debug_display )
      vpDisplay::flush(_image);
}

void Node::spin()
{
    ROS_INFO_STREAM("Spinning.");
    ros::spin();
}

bool Node::addAprilTagService( agimus_vision::AddAprilTagService::Request  &req,
                               agimus_vision::AddAprilTagService::Response &res )
{
    if( _detectors.count( req.id ) != 0 )
    {
        ROS_INFO_STREAM( "Id:" << req.id << " already in use." );
        res.success = false;
        return false;
    }

    ROS_INFO_STREAM( "Id: " << req.id << '(' << req.size << "m) now being tracked." );
    DetectorPtr detector (new DetectorAprilTag( _cam_parameters, req.id, req.size ));


    detector->residualThreshold ( _node_handle.param<double>("residualThreshold", 1e-4) );
    detector->poseThreshold     ( _node_handle.param<double>("poseThreshold"    , 1e-3) );

    _detectors.emplace( req.id,
        DetectorAndName ( detector, req.parent_node_name, req.node_name )
        );
    res.success = true;
    return true;
}

bool Node::setChessboardService( agimus_vision::SetChessboardService::Request  &req,
                                 agimus_vision::SetChessboardService::Response &res )
{
    int id = 0;
    if( _detectors.count( id ) != 0 )
    {
        _detectors.erase( id );
        ROS_INFO_STREAM( "Erasing previous chessboard." );
    }

    _detectors.emplace( id, DetectorAndName (
          DetectorPtr(new DetectorChessboard( _cam_parameters, req.width, req.height, req.size_mm / 1000.0)),
          req.parent_node_name,
          req.node_name )
        );
    res.success = true;
    return true;
}

bool Node::resetTagPosesService( std_srvs::Trigger::Request  &,
                                 std_srvs::Trigger::Response &res )
{
    std::lock_guard<std::mutex> lock(_image_lock);
    for( auto &detector : _detectors )
      detector.second.detector->resetState();
    res.success = true;
    return true;
}

}
}
