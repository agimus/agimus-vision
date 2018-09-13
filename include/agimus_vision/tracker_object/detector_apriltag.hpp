#ifndef __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 

#include "agimus_vision/tracker_object/detector.hpp"

#include <visp3/detection/vpDetectorAprilTag.h>

namespace agimus_vision {
namespace tracker_object {

class DetectorAprilTag : public Detector
{


    const int _tag_id;
    const double _tag_size_meters;

    std::vector< vpPoint > compute3DPoints() const;

public:
    DetectorAprilTag( const vpCameraParameters &cam_parameters, const int tag_id, const double tag_size_meters );

    bool analyseImage( const vpImage< unsigned char > &gray_image );

    bool detect();

    void drawDebug( vpImage< vpRGBa > &image ) const;

    static vpDetectorAprilTag Apriltag_detector;
};

}
}

#endif // __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
