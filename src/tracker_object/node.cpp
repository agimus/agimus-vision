#include "tracker_object/node.hpp"
#include "tracker_object/detector_apriltag.hpp"
#include "tracker_object/detector_chessboard.hpp"

#include <algorithm>
#include <functional>
#include <iostream>

#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImagePoint.h>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/3dpose.h>


Node::Node()
 : _node_handle{}
 , _image_new{ false }
 , _queue_size{ 10 }
 , _debug_display{ true }
{
    // Get parameters for the node
    _node_handle.param<std::string>("/imageTopic", _image_topic, "/camera/rgb/image_rect_color");
    _node_handle.param<std::string>("/cameraInfoTopic", _camera_info_topic, "/camera/rgb/camera_info");
    
    // Use those parameters to create the camera 
    _image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>{_node_handle, _image_topic, _queue_size}); 
    _camera_info_sub.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>{_node_handle, _camera_info_topic, _queue_size});
    _image_info_sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>{*_image_sub, *_camera_info_sub, _queue_size});
    _image_info_sync->registerCallback(std::bind(&Node::frameCallback, this, std::placeholders::_1, std::placeholders::_2));

    waitForImage();

    // TF nodes to publish the position of the tracked object
    _node_handle.param<std::string>("/objectNode", _tf_node, "object");
    _node_handle.param<std::string>("/parentNode", _tf_parent_node, "rgbd_rgb_optical_frame");

    _node_handle.param<bool>("/debugDisplay", _debug_display, true);

    
    // TODO: Switch for detector types and tracker init 
    std::string object_type{};
    _node_handle.param<std::string>( "/objectType", object_type, "apriltag" );
    std::for_each( object_type.begin(), object_type.end(), [](char &c){ c = ::tolower( c ); } );

    if( object_type == "apriltag" )
    {
        int id{};
        double tag_size_meters{};

        _node_handle.param<int>( "/aprilTagId", id, 0 );
        _node_handle.param<double>("/aprilTagSizeMillimeters", tag_size_meters, 80.0); 
            tag_size_meters /= 1000.;
        _detector.reset( new DetectorAprilTag( _cam_parameters, id, tag_size_meters ) );
    }
    else if( object_type == "chessboard" )
    {
        int chessboard_h{}, chessboard_w{};
        double chessboard_square_size_meters{};

        _node_handle.param<int>("/chessboardH", chessboard_h, 6);
        _node_handle.param<int>("/chessboardW", chessboard_w, 9);
        _node_handle.param<double>("/chessboardSquareSizeMillimeters", chessboard_square_size_meters, 23.5); 
            chessboard_square_size_meters /= 1000.;

        _detector.reset( new DetectorChessboard( _cam_parameters, chessboard_w, chessboard_h, chessboard_square_size_meters ) );
    }
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
    
    _image = vpImage<vpRGBa>{ image->height, image->width };
    for(unsigned int i{ 0 } ; i < _image.getHeight() ; ++i )
        for(unsigned int j{ 0 } ; j < _image.getWidth() ; ++j)
        {
            if(image->encoding == sensor_msgs::image_encodings::MONO8)
                _image[i][j] = vpRGBa{image->data[i * image->step + j], image->data[i * image->step + j], image->data[i * image->step + j]};
            else if(image->encoding == sensor_msgs::image_encodings::RGB8)
                _image[i][j] = vpRGBa{image->data[i * image->step + j * 3 + 0], image->data[i * image->step + j * 3 + 1], image->data[i * image->step + j * 3 + 2]};
            else if(image->encoding == sensor_msgs::image_encodings::RGBA8)
                _image[i][j] = vpRGBa{image->data[i * image->step + j * 4 + 0], image->data[i * image->step + j * 4 + 1], image->data[i * image->step + j * 4 + 2]};
            else if(image->encoding == sensor_msgs::image_encodings::BGR8)
                _image[i][j] = vpRGBa{image->data[i * image->step + j * 3 + 2], image->data[i * image->step + j * 3 + 1], image->data[i * image->step + j * 3 + 0]};
            else if(image->encoding == sensor_msgs::image_encodings::BGRA8)
                _image[i][j] = vpRGBa{image->data[i * image->step + j * 4 + 2], image->data[i * image->step + j * 4 + 1], image->data[i * image->step + j * 4 + 0]};
            else
                _image[i][j] = vpRGBa{0, 0, 0};
        }

    vpImageConvert::convert(_image, _gray_image);
    _cam_parameters = visp_bridge::toVispCameraParameters(*camera_info);

    _image_new = true;
}

void Node::spin()
{
    vpDisplayX disp{};

    waitForImage();
    {
        std::lock_guard<std::mutex> lock(_image_lock);
        disp.init(_image);
        vpDisplay::setTitle(_image, "Visual display");
    }

    ros::Rate rate(30);
    bool init_done{ false };


    while(ros::ok())
    {
        vpDisplay::display(_image);

        if( _detector->detect( _gray_image ) )
        {
            publish_pose( _detector->getLastCMO() );
            
            if( _debug_display )
                _detector->drawDebug( _image );
        }

        vpDisplay::flush(_image);
        ros::spinOnce();
        rate.sleep();
    }
}

void Node::publish_pose( const vpHomogeneousMatrix &cMo )
{
    static tf2_ros::TransformBroadcaster broadcaster;
    
    // Visp -> TF
    auto transform = visp_bridge::toGeometryMsgsTransform( cMo );
    geometry_msgs::TransformStamped transform_stamped{};
    transform_stamped.header.frame_id = _tf_parent_node;
    transform_stamped.child_frame_id = _tf_node;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.transform = transform;

    broadcaster.sendTransform( transform_stamped );
}
