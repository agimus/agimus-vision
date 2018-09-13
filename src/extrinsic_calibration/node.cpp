#include "agimus_vision/extrinsic_calibration/node.hpp"

#include <functional>
#include <iostream>
#include <algorithm>
#include <locale>
#include <cmath>

#include <tf/transform_listener.h>

#include <visp_bridge/3dpose.h>

namespace agimus_vision {
namespace extrinsic_calibration {

Node::Node()
 : _node_handle{}
{
    // Get parameters for the node
    _node_handle.param<std::string>( "/handEffectorNode", _hand_effector_node, "gripper_right_base_link" ); 
    _node_handle.param<std::string>( "/headEffectorNode", _head_effector_node, "head_2_link" ); 
    _node_handle.param<std::string>( "/cameraFrameNode", _camera_frame_node, "rgbd_rgb_optical_frame" ); 
    _node_handle.param<std::string>( "/objectNode", _object_node, "chessboard" );
    _node_handle.param<double>( "/wMeDistThreshold", _wMe_dist_threshold, 5. );
    _node_handle.param<double>( "/cMoDistThreshold", _cMo_dist_threshold, 5. );
    _node_handle.param<int>( "/nbPosesNeeded", _nb_poses_needed, 12 );

    _services.push_back( _node_handle.advertiseService( "addPoseToCalibration", &Node::addPoseToCalibration, this ) );
    _services.push_back( _node_handle.advertiseService( "computeCalibration", &Node::computeCalibration, this ) );
}


void Node::spin()
{
    static tf2_ros::Buffer tf_buffer{};
    static tf2_ros::TransformListener listener{ tf_buffer };

    ros::Rate rate(30);

    bool askUser{ false };
    vpHomogeneousMatrix wMe_old{}, cMo_old{};
    uint32_t wMe_seq{}, cMo_seq{};

    while(ros::ok())
    {
        askUser = false;

        try
        {
            auto now = ros::Time::now() - ros::Duration( 0.1 );

            // Visp wants the coord of the obj in the frame of the camera. 
            // But lookuptransform gives us the inverse, aka the transform from the cam to the obj
            // So, we need to inverse the parameters target and source
            auto wMe_stamped = tf_buffer.lookupTransform( _hand_effector_node, _head_effector_node, now, rate.expectedCycleTime() );
            auto cMo_stamped = tf_buffer.lookupTransform( _camera_frame_node, _object_node, now, rate.expectedCycleTime() );

            if( wMe_seq != wMe_stamped.header.seq )
            {
                _wMe = visp_bridge::toVispHomogeneousMatrix( wMe_stamped.transform );
                _wMe_dist = homogeneousTransformMatrixDistanceMetric( _wMe, wMe_old );
                wMe_old = _wMe;
            }

            if( cMo_seq != cMo_stamped.header.seq )
            {
                _cMo = visp_bridge::toVispHomogeneousMatrix( cMo_stamped.transform );
                _cMo_dist = homogeneousTransformMatrixDistanceMetric( _cMo, cMo_old );
                cMo_old = _cMo;
            }

            if( _wMe_dist <= _wMe_dist_threshold && _cMo_dist <= _cMo_dist_threshold )
                askUser = true;
        }
        catch(tf::TransformException ex)
        {
            std::cout << "\r" << ex.what() << std::flush;
        }
        
        if( askUser )
            std::cout << "\r" << "You can add this pose to the calibration process by calling the addPoseToCalibration service." << std::flush;
        else
            std::cout << "\r" << "The robot or the object is moving too much, you cannot add this pose to the calibration process." << std::flush;

        ros::spinOnce();
        rate.sleep();
    }
    
}

bool Node::addPoseToCalibration( std_srvs::Empty::Request  &,
                                 std_srvs::Empty::Response & )
{
    if( _wMe_dist <= _wMe_dist_threshold && _cMo_dist <= _cMo_dist_threshold )
    {
        _calib.addPose( _cMo, _wMe );
        ROS_INFO( "Successfully added the current pose to the calibration process." );
        
        return true;
    }
    
    ROS_ERROR( "Cannot add the current pose for the calibration (too much movement)." );

    return false;
}

bool Node::computeCalibration( std_srvs::Empty::Request  &,
                               std_srvs::Empty::Response & )
{
    if( _calib.getNbPose() >= _nb_poses_needed )
    {
        auto eMc_computed = _calib.getEMC();
        
        tf2_ros::Buffer tf_buffer{};
        tf2_ros::TransformListener listener{ tf_buffer }; 
    
        auto eMc_stamped = tf_buffer.lookupTransform( _head_effector_node, _camera_frame_node, ros::Time::now(), ros::Duration( 1. ) );
        auto eMc_current = visp_bridge::toVispHomogeneousMatrix( eMc_stamped.transform );

        std::cout << "Current eMc transform is:\n" << eMc_current << "\n-----" << std::endl;
        std::cout << "Computed eMc transform is:\n" << eMc_computed << std::endl;

        return true;
    }

    ROS_ERROR( "Not enough poses to compute. %d/%d", _calib.getNbPose(), _nb_poses_needed );

    return false;
}

double Node::homogeneousTransformMatrixDistanceMetric( const vpHomogeneousMatrix &a, const vpHomogeneousMatrix &b )
{
    auto l = []( const vpHomogeneousMatrix &M ){
        return std::sqrt( std::pow( M[0][3], 2. ) + std::pow( M[1][3], 2. ) + std::pow( M[2][3], 2. ) );
    };

    auto theta = []( const vpHomogeneousMatrix &M ){
        double num{ std::sqrt( std::pow( M[2][1] - M[1][2], 2. ) + std::pow( M[0][2] - M[2][0], 2. ) + std::pow( M[1][0] - M[0][1], 2. ) ) };
        double denom{ M[0][0] + M[1][1] + M[2][2] - 1 };

        return std::atan( num / denom );
    };

    auto strengh = [&l, &theta]( const vpHomogeneousMatrix &M ){
        return std::sqrt( std::pow( l( M ), 2. ) + std::pow( 2.632 * theta( M ), 2. ) );
    };

    return strengh( a.inverse() * b );
}

}
}

