#ifndef __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 

#include "tracker_object/detector.hpp"

#include <visp3/detection/vpDetectorAprilTag.h>


class DetectorAprilTag : public Detector
{
    vpDetectorAprilTag _apriltag_detector;
    const int _tag_id;
    const double _tag_size_meters;

    std::vector< vpPoint > compute3DPoints() const;

public:
    DetectorAprilTag( const vpCameraParameters &cam_parameters, const int tag_id, const double tag_size_meters );

    bool detect( const vpImage< unsigned char > &gray_image );

    void drawDebug( vpImage< vpRGBa > &image ) const;
};

#endif // __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
