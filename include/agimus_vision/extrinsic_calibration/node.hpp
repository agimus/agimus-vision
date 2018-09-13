#ifndef __EXTRINSIC_CALIBRATION__NODE__HPP__
#define __EXTRINSIC_CALIBRATION__NODE__HPP__

#include "agimus_vision/extrinsic_calibration/extrinsic_calibration.hpp"

#include "agimus_vision/EmptyService.h"

#include <mutex>
#include <string>
#include <memory>

#include <ros/ros.h>

namespace agimus_vision {
namespace extrinsic_calibration {

class Node
{
    ros::NodeHandle _node_handle;

    std::string _hand_effector_node;
    std::string _head_effector_node;
    std::string _camera_frame_node;
    std::string _object_node;

    vpHomogeneousMatrix _wMe, _cMo;
    double _wMe_dist, _cMo_dist;
    double _wMe_dist_threshold, _cMo_dist_threshold;

    ExtrinsicCalibration _calib;
    int _nb_poses_needed;

    std::vector< ros::ServiceServer > _services;


    double homogeneousTransformMatrixDistanceMetric( const vpHomogeneousMatrix &a, const vpHomogeneousMatrix &b );
    
public:
    Node();

    void spin();

    bool addPoseToCalibration( agimus_vision::EmptyService::Request  &rq,
                               agimus_vision::EmptyService::Response &res );

    bool computeCalibration( agimus_vision::EmptyService::Request  &rq,
                             agimus_vision::EmptyService::Response &res );
};

}
}

#endif
