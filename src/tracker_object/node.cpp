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
#include <image_transport/image_transport.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/3dpose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>



using namespace message_filters;


namespace agimus_vision
{
  namespace tracker_object
  {

    geometry_msgs::Transform getTransformAtTimeOrNewest(
        tf2_ros::Buffer &buf,
        const std::string &parent,
        const std::string &node,
        const ros::Time &stamp,
        bool &ok)
    {
      static ros::Time origin(0);
      try
      {
        ok = true;
        return buf.lookupTransform(parent, node, stamp).transform;
      }
      catch (const tf2::ExtrapolationException &)
      {
        ok = true;
        return buf.lookupTransform(parent, node, origin).transform;
      }
      catch (const tf2::TransformException &e)
      {
        ROS_WARN(e.what());
        ok = false;
        return geometry_msgs::Transform();
      }
    }

    using tf2::convert;

    void convert(const vpHomogeneousMatrix &Min, tf2::Transform &out)
    {
      tf2::convert(visp_bridge::toGeometryMsgsTransform(Min), out);
    }

    Node::Node()
        : _node_handle{"~"}, _tf_buffer{}, _tf_listener{_tf_buffer}, _debug_display{nullptr}
    {
      // Get parameters for the node
      _node_handle.param<std::string>("imageTopic", _image_topic, "/camera/rgb/image_rect_color");
      _node_handle.param<std::string>("depthImageTopic", _depth_image_topic, "/camera/depth/image_rect");
      _node_handle.param<std::string>("cameraInfoTopic", _camera_info_topic, "/camera/rgb/camera_info");
      _node_handle.param<std::string>("depthCameraInfoTopic", _depth_camera_info_topic, "/camera/depth/camera_info");

      // Initialize camera parameters
      ROS_INFO_STREAM("Wait for camera info message on " << _camera_info_topic);
      sensor_msgs::CameraInfoConstPtr cam_info_msg =
          ros::topic::waitForMessage<sensor_msgs::CameraInfo>(_camera_info_topic,
                                                              _node_handle);
      if (!cam_info_msg)
        return;
      cameraInfoCallback(cam_info_msg);

      ROS_INFO_STREAM("Wait for depth camera info message on " << _depth_camera_info_topic);
      sensor_msgs::CameraInfoConstPtr depth_cam_info_msg =
          ros::topic::waitForMessage<sensor_msgs::CameraInfo>(_depth_camera_info_topic,
                                                              _node_handle);
      if (!depth_cam_info_msg)
        return;
      depthCameraInfoCallback(depth_cam_info_msg);

      // Use those parameters to create the camera
      _camera_info_sub = _node_handle.subscribe(_camera_info_topic, 1,
                                                &Node::cameraInfoCallback, this);

      _depth_camera_info_sub = _node_handle.subscribe(_depth_camera_info_topic, 1,
                                                      &Node::depthCameraInfoCallback, this);

      _image_sub.subscribe(_node_handle, _image_topic, 1);
      ROS_INFO("Subcribed to the topic: %s", _image_topic.c_str());

      _deph_image_sub.subscribe(_node_handle, _depth_image_topic, 1);
      ROS_INFO("Subcribed to the topic: %s", _depth_image_topic.c_str());

      sync_.reset(new Sync(MySyncPolicy(50), _image_sub, _deph_image_sub));
      sync_->registerCallback(boost::bind(&Node::frameCallback, this, _1, _2));

      // TF node of the camera seeing the tags
      _node_handle.param<std::string>("cameraFrame", _tf_camera_node, "rgbd_rgb_optical_frame");

      // Broadcasting methods
      _node_handle.param<bool>("broadcastTf", _broadcast_tf, false);
      _node_handle.param<std::string>("broadcastTfPostfix", _broadcast_tf_postfix, "");
      _node_handle.param<bool>("broadcastTopic", _broadcast_topic, false);

      //some others parameters
      _node_handle.param<float>("depthScale", _depth_scale_param, (float)0.001); //0.1 for tiago's orbbec
      _node_handle.param<float>("depthRGBDistance", _depth_rgb_distance_param, (float)0.000); //0.047 for tiago's orbbec

      // TODO: Switch for detector types and tracker init
      std::string object_type{};
      _node_handle.param<std::string>("objectType", object_type, "apriltag");
      std::for_each(object_type.begin(), object_type.end(), [](char &c)
                    { c = (char)::tolower(c); });

      _services.push_back(_node_handle.advertiseService("reset_tag_poses", &Node::resetTagPosesService, this));

      if (object_type == "apriltag")
      {
        _services.push_back(_node_handle.advertiseService("add_april_tag_detector", &Node::addAprilTagService, this));
        _services.push_back(_node_handle.advertiseService("add_object_tracking", &Node::addObjectTracking, this));
      }
      else if (object_type == "chessboard")
      {
        _services.push_back(_node_handle.advertiseService("set_chessboard_detector", &Node::setChessboardService, this));
      }

      _publisherVision = _node_handle.advertise<geometry_msgs::TransformStamped>("/agimus/vision/tags", 100);
      _detection_publisher = _node_handle.advertise<agimus_vision::ImageDetectionResult>("/agimus/vision/detection", 100);

      tracker_reconfigure.setCallback(boost::bind(&Node::trackerReconfigureCallback, this, _1, _2));
    }

    Node::~Node()
    {
    }

    void Node::trackerReconfigureCallback(TrackerConfig &config, uint32_t level)
    {
      for (Tracker &tracker : _trackers)
        tracker.reconfigure(config, level);
    }

    void Node::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info)
    {
      std::lock_guard<std::mutex> lock(_cam_param_lock);
      _cam_parameters = visp_bridge::toVispCameraParameters(*camera_info);
    }

    void Node::depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info)
    {
      std::lock_guard<std::mutex> lock(_cam_param_lock);
      _depth_cam_parameters = visp_bridge::toVispCameraParameters(*camera_info);
    }

    // void Node::frameCallback(const sensor_msgs::ImageConstPtr &image)
    void Node::frameCallback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &depth_image)
    {
      std::unique_lock<std::mutex> lock(_image_lock, std::try_to_lock);
      if (!lock.owns_lock())
        return;
      if (_image_header.seq + 1 < image->header.seq)
      {
        // Delayed ignores the first _image_header which isn't initialized.
        ROS_INFO_DELAYED_THROTTLE(5, "Some images were dropped.");
      }

      _image_header = image->header;

      if (image->encoding != sensor_msgs::image_encodings::MONO8 && image->encoding != sensor_msgs::image_encodings::RGB8 && image->encoding != sensor_msgs::image_encodings::RGBA8 && image->encoding != sensor_msgs::image_encodings::BGR8 && image->encoding != sensor_msgs::image_encodings::BGRA8)
      {
        ROS_ERROR_STREAM("The input image must be grayscale.");
        ros::shutdown();
      }


      //Distance between two sensors
      //For realsense D435: -0.015 but 0 if enable depth and rgb registration
      //For Tiago Orbec   : 0.047
      // ROS_WARN_STREAM("_depth_rgb_distance_param:" + std::to_string(_depth_rgb_distance_param));
      double rgbDepthSensorDist;
      if (_image_topic.find("xtion") != std::string::npos)
        //  rgbDepthSensorDist = -0.00;
         rgbDepthSensorDist = -_depth_rgb_distance_param;
      else
          rgbDepthSensorDist = 0.0;

      try
      {

      cv::Mat ColorMat = (cv::Mat_<double>(3, 3) << _cam_parameters.get_px(), 0, _cam_parameters.get_u0(),
                          0, _cam_parameters.get_py(), _cam_parameters.get_v0(),
                          0, 0, 1);

      cv::Mat DepthMat = (cv::Mat_<double>(3, 3) << _depth_cam_parameters.get_px(), 0, _depth_cam_parameters.get_u0(),
                          0, _depth_cam_parameters.get_py(), _depth_cam_parameters.get_v0(),
                          0, 0, 1);

      cv::Mat Rt = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, rgbDepthSensorDist,
                                              0.0, 1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0, 0.0,
                                              0.0, 0.0, 0.0, 1.0);

      

      //copy depth image to opencv mat
      cv_bridge::CvImagePtr cvDepthPtr;
      cvDepthPtr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);

      cv::Mat frame_depth, frame_depth_reg;
      cvDepthPtr->image.copyTo(frame_depth);

      //rgb and depth has same size as default, so does not matter to use frame frame_depth.size()
      cv::rgbd::registerDepth(DepthMat, ColorMat, cv::Mat(), Rt, frame_depth,
                              frame_depth.size(), frame_depth_reg, true /* dilate */);

      _gray_image = visp_bridge::toVispImage(*image);
      _depth_image = toVispImageFromCVDepth(frame_depth_reg);

      }
      catch (const std::exception &e)
      {
        std::cerr << e.what() << '\n';
      }


      imageProcessing();
    }

    vpImage<uint16_t> Node::toVispImageFromDepth(const sensor_msgs::Image &src)
    {

      vpImage<uint16_t> dst(src.height, src.width);

      if (src.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {

        memcpy(dst.bitmap, &(src.data[0]), dst.getHeight() * src.step * sizeof(unsigned char));
      }
      // ROS_WARN_STREAM(dst);
      return dst;
    }

    vpImage<uint16_t> Node::toVispImageFromCVDepth(cv::Mat &depthImage)
    {
      vpImage<uint16_t> dst(depthImage.rows, depthImage.cols);
      memcpy(dst.bitmap, &depthImage.data[0], depthImage.rows * depthImage.cols * sizeof(uint16_t));

      return dst;
    }

    void Node::imageProcessing()
    {
      ros::Time time_begin(ros::Time::now());
      static tf2_ros::TransformBroadcaster broadcaster;

      // Display the tags seen by the camera
      bool debug_display = _node_handle.param<bool>("debugDisplay", false);
      if (debug_display && !_debug_display)
      {
        _debug_display.reset(new vpDisplayOpenCV{});
        _debug_display->init(_gray_image);
        vpDisplay::setTitle(_gray_image, "Visual display");
      }
      else if (!debug_display && _debug_display)
      {
        vpDisplay::close(_gray_image);
        _debug_display.reset(NULL);
      }

      if (debug_display)
        vpDisplay::display(_gray_image);

      if (_aprilTagDetector)
        _aprilTagDetector->imageReady = false;

      auto timestamp = _image_header.stamp;

      for (Tracker &tracker : _trackers)
      {
        tracker.process(_gray_image, _depth_image, timestamp.toSec(), _depth_scale_param);
        if (tracker.hasPose())
        {
          // c: camera
          // o: object
          vpHomogeneousMatrix cMo_vp;
          tracker.getPose(cMo_vp);
          tf2::Transform cMo_tf;
          convert(cMo_vp, cMo_tf);

          geometry_msgs::TransformStamped cMo_msg;
          tf2::convert(cMo_tf, cMo_msg.transform);

          cMo_msg.child_frame_id = tracker.name();
          cMo_msg.header.frame_id = _tf_camera_node;
          cMo_msg.header.stamp = timestamp;

          if (_broadcast_topic)
            _publisherVision.publish(cMo_msg);
          if (_broadcast_tf)
            broadcaster.sendTransform(cMo_msg);
        }
        if (debug_display)
          tracker.drawDebug(_gray_image);
      }

      agimus_vision::ImageDetectionResult result;
      result.header = _image_header;

      for (auto &detector : _detectors)
      {
        const DetectorPtr &detector_ptr = detector.second.detector;
        const std::string &parent_name = detector.second.parent_name;
        const std::string &object_name = detector.second.object_name;

        if (detector_ptr->analyseImage(_gray_image) && detector_ptr->detectOnDepthImage(_depth_image, _depth_scale_param))
        {
          // c: camera
          // o: object
          // p: parent
          tf2::Transform cMo;
          convert(detector_ptr->getLastCMO(), cMo);

          geometry_msgs::Transform cMo_msg;
          tf2::convert(cMo, cMo_msg);

          result.ids.push_back(detector_ptr->id());
          result.residuals.push_back(detector_ptr->error());
          result.poses.push_back(cMo_msg);

          bool ok;
          tf2::Transform pMc;
          convert(getTransformAtTimeOrNewest(_tf_buffer, parent_name, _tf_camera_node, timestamp, ok),
                  pMc);
          if (!ok)
            continue;

          geometry_msgs::TransformStamped pMo_msg;
          tf2::convert(pMc * cMo, pMo_msg.transform);

          pMo_msg.child_frame_id = object_name;
          pMo_msg.header.frame_id = parent_name;
          pMo_msg.header.stamp = timestamp;

          if (_broadcast_topic)
            _publisherVision.publish(pMo_msg);
          pMo_msg.child_frame_id += _broadcast_tf_postfix;
          if (_broadcast_tf)
            broadcaster.sendTransform(pMo_msg);

          if (debug_display)
            detector_ptr->drawDebug(_gray_image);
        }
      }
      if (!result.ids.empty())
        _detection_publisher.publish(result);

      ros::Time time_end(ros::Time::now());
      ros::Duration delay = time_end - timestamp;
      if (delay > ros::Duration(_node_handle.param<double>("max_delay", 0.3)))
        ROS_WARN_STREAM("Image " << _image_header.seq << "\n"
                                                         "Input delay     : "
                                 << time_begin - timestamp << "\n"
                                                              "Computation time: "
                                 << time_end - time_begin << "\n"
                                                             "Output delay    : "
                                 << delay);

      if (debug_display)
      {
        if (_node_handle.param<bool>("publishDebugDisplay", false))
        {
          if (!_debug_publisher)
          {
            image_transport::ImageTransport it(_node_handle);
            _debug_publisher.reset(new image_transport::Publisher(it.advertise("debug", 1)));
          }
          RGBaImage_t img;
          vpDisplay::getImage(_gray_image, img);
          sensor_msgs::Image msg = visp_bridge::toSensorMsgsImage(img);
          _debug_publisher->publish(msg);
        }
        else
          vpDisplay::flush(_gray_image);
      }
    }

    void Node::spin()
    {
      ROS_INFO_STREAM("Spinning.");
      ros::spin();
    }

    bool Node::addAprilTagService(agimus_vision::AddAprilTagService::Request &req,
                                  agimus_vision::AddAprilTagService::Response &res)
    {
      if (_detectors.count(req.id) != 0)
      {
        ROS_INFO_STREAM("Id:" << req.id << " already in use.");
        res.success = false;
        return false;
      }

      initAprilTagDetector();

      ROS_INFO_STREAM("Id: " << req.id << '(' << req.size << "m) now being tracked.");
      DetectorPtr detector(new DetectorAprilTag(
          _aprilTagDetector, _cam_parameters, _depth_cam_parameters, req.id, req.size));

      detector->residualThreshold(_node_handle.param<double>("residualThreshold", 1e-4));
      detector->poseThreshold(_node_handle.param<double>("poseThreshold", 1e-3));

      std::unique_lock<std::mutex> lock(_image_lock);
      _detectors.emplace(req.id,
                         DetectorAndName(detector, req.parent_node_name, req.node_name));
      res.success = true;
      return true;
    }

    bool Node::addObjectTracking(agimus_vision::AddObjectTracking::Request &req,
                                 agimus_vision::AddObjectTracking::Response &res)
    {
      if (req.tags.size() == 0)
      {
        res.message = "At least one tag must be specified.";
        res.success = false;
        return true;
      }

      initAprilTagDetector();

      // Setup initialization step
      std::shared_ptr<initializationStep::AprilTag> aprilTag(
          new initializationStep::AprilTag(_aprilTagDetector));
      aprilTag->cameraParameters(_cam_parameters);
      for (const agimus_vision::AprilTag &tag : req.tags)
      {
        vpTranslationVector t(tag.oMt.translation.x, tag.oMt.translation.y, tag.oMt.translation.z);
        vpQuaternionVector q(tag.oMt.rotation.x, tag.oMt.rotation.y, tag.oMt.rotation.z, tag.oMt.rotation.w);
        aprilTag->addTag(tag.id, tag.size, vpHomogeneousMatrix(t, q));
      }
      std::shared_ptr<TrackingStep> tracking;

      std::transform(req.tracker_type.begin(), req.tracker_type.end(),
                     req.tracker_type.begin(), ::tolower);

      if (req.tracker_type == "apriltag")
      {
        // Same initialization and tracking step
        tracking = aprilTag;
      }
      else
      {
        // Setup tracking step
        int type;
        if (req.tracker_type == "edgetracker")
        {
          type = vpMbGenericTracker::EDGE_TRACKER;
        }
        else if (req.tracker_type == "edgeklttracker")
        {
          type = vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER;
        }
        else
        {
          res.message = "Invalid tracker type.";
          res.success = false;
          return true;
        }

        // std::shared_ptr<trackingStep::ModelBased> modelBased(
        //     new trackingStep::ModelBased(type,
        //                                  req.model_path,
        //                                  _cam_parameters,
        //                                  _node_handle.param<double>("tracker/projection_error_threshold", 40),
        //                                  req.visp_xml_config_file));
        // modelBased->tracker().setDisplayFeatures(true);

        // tracking = modelBased;
      }

      Tracker tracker(aprilTag, tracking,
                      req.object_name,
                      _node_handle.param<int>("detection_subsampling", 5));
      double cutFrequency;
      if (_node_handle.getParam("cut_frequency", cutFrequency))
        tracker.filtering(std::make_shared<filteringStep::PositionLowPassOrder>(cutFrequency, 1));

      std::unique_lock<std::mutex> lock(_image_lock);
      _trackers.push_back(tracker);
      ROS_INFO_STREAM("Object " << req.object_name << " now being tracked using " << req.tracker_type);
      res.success = true;
      return true;
    }

    bool Node::setChessboardService(agimus_vision::SetChessboardService::Request &req,
                                    agimus_vision::SetChessboardService::Response &res)
    {
      int id = 0;
      if (_detectors.count(id) != 0)
      {
        _detectors.erase(id);
        ROS_INFO_STREAM("Erasing previous chessboard.");
      }

      _detectors.emplace(id, DetectorAndName(
                                 DetectorPtr(new DetectorChessboard(_cam_parameters, req.width, req.height, req.size_mm / 1000.0)),
                                 req.parent_node_name,
                                 req.node_name));
      res.success = true;
      return true;
    }

    bool Node::resetTagPosesService(std_srvs::Trigger::Request &,
                                    std_srvs::Trigger::Response &res)
    {
      std::unique_lock<std::mutex> lock(_image_lock);
      for (auto &detector : _detectors)
        detector.second.detector->resetState();
      res.success = true;
      return true;
    }

    void Node::initAprilTagDetector()
    {
      if (_aprilTagDetector)
        return;

      _aprilTagDetector.reset(new DetectorAprilTagWrapper(
          vpDetectorAprilTag::TAG_36h11));

      _aprilTagDetector->detector.setAprilTagPoseEstimationMethod(
          vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS);
      _aprilTagDetector->detector.setAprilTagNbThreads(
          _node_handle.param<int>("apriltag/nb_threads", 4));
      _aprilTagDetector->detector.setAprilTagQuadDecimate(
          _node_handle.param<float>("apriltag/quad_decimate", 1.));
    }

  }
}
