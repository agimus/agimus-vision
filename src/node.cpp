#include "node.hpp"

#include <functional>
#include <iostream>

#include <ros/package.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/vision/vpPose.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>


Node::Node()
 : _node_handle{}
 , _image_new{ false }
 , _queue_size{ 10 }
 , _april_tag_detector{}
 , _edge_klt_tracker{}
 , _april_tag_points{ vpPoint{ 0.062, 0.082, 0.0 }, vpPoint{ 0.062, 0.018, 0.0 }, vpPoint{ 0.127, 0.018, 0.0 }, vpPoint{ 0.127, 0.082, 0.0 } }
 , _debug_display{ true }
{
    // Get parameters for the node
    _node_handle.param<std::string>("/imageTopic", _image_topic, "/camera/rgb/image_rect_color");
    _node_handle.param<std::string>("/cameraInfoTopic", _camera_info_topic, "/camera/rgb/camera_info");
    _node_handle.param<std::string>("/positionNode", _tf_node, "box");
    _node_handle.param<std::string>("/positionParentNode", _tf_parent_node, "rgbd_rgb_optical_frame");

    // Use those parameters to create the camera 
    _image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>{_node_handle, _image_topic, _queue_size}); 
    _camera_info_sub.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>{_node_handle, _camera_info_topic, _queue_size});
    _image_info_sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>{*_image_sub, *_camera_info_sub, _queue_size});
    _image_info_sync->registerCallback(std::bind(&Node::frameCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Edge and keypoints tracker settings
    _me.setMaskSize(5);
    _me.setMaskNumber(180);
    _me.setRange(8);
    _me.setThreshold(10000);
    _me.setMu1(0.5);
    _me.setMu2(0.5);
    _me.setSampleStep(4);
    _edge_klt_tracker.setMovingEdge(_me);

    _klt_settings.setMaxFeatures(300);
    _klt_settings.setWindowSize(5);
    _klt_settings.setQuality(0.015);
    _klt_settings.setMinDistance(8);
    _klt_settings.setHarrisFreeParameter(0.01);
    _klt_settings.setBlockSize(3);
    _klt_settings.setPyramidLevels(3);
    _edge_klt_tracker.setKltOpencv(_klt_settings);
    _edge_klt_tracker.setKltMaskBorder(5);
}

void Node::waitForImage()
{
    ros::Rate rate{50};
    while(ros::ok())
    {
        if(_image_new)
            return;
        ros::spinOnce();
        rate.sleep();
    }
}


void Node::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    std::lock_guard<std::mutex> lock(_image_lock);
    _image_header = image->header;
    _image = visp_bridge::toVispImageRGBa(*image);
    vpImageConvert::convert(_image, _gray_image);
    _camera_param = visp_bridge::toVispCameraParameters(*camera_info);

    _image_new = true;
}

void Node::spin()
{
    vpDisplayX disp{};

    waitForImage();
    {
        std::lock_guard<std::mutex> lock(_image_lock);
        disp.init(_gray_image);
        vpDisplay::setTitle(_gray_image, "Test");
    }

    ros::Rate rate(30);
    bool init_done{ false };

    int step{ 0 };

    while(ros::ok())
    {
        vpDisplay::display(_gray_image);
        _new_cMo_tag = _new_cMo_tracker = false;

        switch( step )
        {
            case 0: // Search the AprilTag with id: 0
                {
                    int n{ aprilTagSearch( 0 ) };
                    if( n == - 1)
                        break;
                    initTracking( n );
                    step = 1;
                }
                break;
            case 1: // Track the box
                try
                {
                    int n{ aprilTagSearch( 0 ) };
                    vpTranslationVector vec{};
                    if( n != -1 )
                    {
                        std::vector<vpImagePoint> p{ _april_tag_detector.getPolygon(n) };
                        computePose( _april_tag_points, p, _camera_param, false, _cMo_tag );
                        _new_cMo_tag = true;
                        _edge_klt_tracker.setPose(_gray_image, _cMo_tag);
                    }

                    _edge_klt_tracker.track(_gray_image);
                    _edge_klt_tracker.getPose(_cMo_tracker);
                    _new_cMo_tracker = true;
                    if(_debug_display)
                    {
                        _edge_klt_tracker.display(_gray_image, _cMo_tracker, _camera_param, vpColor::red, 2, true);
                        vpDisplay::displayFrame(_gray_image, _cMo_tracker, _camera_param, 0.025, vpColor::none, 3);
                    }

                    if( _new_cMo_tag )
                    {
                        auto vec = _cMo_tag.getTranslationVector();
                        auto vec2 = _cMo_tracker.getTranslationVector();
                        if( vec2[ 2 ] < 0.15 || ( ( vec.euclideanNorm() > 0.01 ) && ( vec2 - vec ).euclideanNorm() > 0.025 ) )
                            step = 0;
                    }
                }
                catch(...)
                {
                    step = 0;
                }
                break;
        }

        publish_pose();

        vpDisplay::flush(_gray_image);
        ros::spinOnce();
        rate.sleep();
    }
}

int Node::aprilTagSearch( int id )
{
    _april_tag_detector.detect(_gray_image);

    auto endsWith = []( std::string str, std::string end){
        for( size_t i{ 0 } ; i <= end.size() ; ++i )
            if( str[ str.size() - end.size() + i ] != end[ i ] )
                return false;
        return true;
    };

    for( size_t i{ 0 } ; i < _april_tag_detector.getNbObjects() ; ++i )
        if( endsWith( _april_tag_detector.getMessage( i ), " " + std::to_string( id ) ) )
            return i;

    return -1;
}

void Node::initTracking( int id )
{
    _edge_klt_tracker.setCameraParameters(_camera_param);
    _edge_klt_tracker.loadModel(ros::package::getPath("agimus-vision") + "/objects/euclidBox.cao");
    _edge_klt_tracker.setDisplayFeatures(true);
    _edge_klt_tracker.setAngleAppear( vpMath::rad(70) );
    _edge_klt_tracker.setAngleDisappear( vpMath::rad(80) );

    std::vector<vpImagePoint> p{ _april_tag_detector.getPolygon( id ) };
    computePose( _april_tag_points, p, _camera_param, true, _cMo_tag );
    _new_cMo_tag = true;
    _edge_klt_tracker.initFromPose( _gray_image, _cMo_tag );
}

void Node::computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                       bool init, vpHomogeneousMatrix &cMo)
{
    vpPose pose;
    
    // Compute the 2D coord. of the points (in meters) to match the 3D coord. for the pose estimation 
    for(unsigned int i{ 0 } ; i < point.size() ; i++) 
    {
        double x{ 0. }, y{ 0. };
        vpPixelMeterConversion::convertPointWithoutDistortion(cam, ip[i], x, y);
        // x, y are the coordinates in the image plane, oX, oY, oZ in the world, and X, Y, Z in the camera ref. frame
        point[i].set_x(x);
        point[i].set_y(y);
        pose.addPoint(point[i]);
    }

    if(init)
    {
        vpHomogeneousMatrix cMo_dem;
        vpHomogeneousMatrix cMo_lag;
        pose.computePose(vpPose::DEMENTHON, cMo_dem);
        pose.computePose(vpPose::LAGRANGE, cMo_lag);
        if(pose.computeResidual(cMo_dem) < pose.computeResidual(cMo_lag))
            cMo = cMo_dem;
        else
            cMo = cMo_lag;
    }
 
    pose.computePose(vpPose::VIRTUAL_VS, cMo);
}

void Node::publish_pose()
{
    static tf::TransformBroadcaster broadcaster;
    
    vpHomogeneousMatrix cMo;

    if(_new_cMo_tag && _new_cMo_tracker)
        cMo = _cMo_tag; // TODO: Mean of the two cMo?
    else if(_new_cMo_tag)
        cMo = _cMo_tag;
    else if(_new_cMo_tracker)
        cMo = _cMo_tracker;
    else
        return;

    // Visp -> tf
    auto translation = cMo.getTranslationVector();
    tf::Vector3 t{translation[0], translation[1], translation[2]};
    auto rotation = cMo.getThetaUVector();
    tf::Quaternion q{ tf::Vector3{ rotation.getU()[0], rotation.getU()[1], rotation.getU()[2] }, 
            rotation.getTheta() };
    
    tf::Transform transform{ q, t };
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _tf_parent_node, _tf_node));
}
