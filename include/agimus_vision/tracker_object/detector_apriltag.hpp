#ifndef __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 

#include "agimus_vision/tracker_object/detector.hpp"
#include "agimus_vision/tracker_object/fwd.hpp"

#include <visp3/detection/vpDetectorAprilTag.h>

namespace agimus_vision {
namespace tracker_object {

struct DetectorAprilTagWrapper
{
  vpDetectorAprilTag detector;
  bool imageReady;

  DetectorAprilTagWrapper(const vpDetectorAprilTag::vpAprilTagFamily &tagFamily = vpDetectorAprilTag::TAG_36h11) :
    detector (tagFamily),
    imageReady (false)
  {}

  bool detect (const GrayImage_t& I)
  {
    if (imageReady) return detector.getNbObjects() > 0;
    imageReady = true;
    return detector.detect (I);
  }

  vpDetectorAprilTag* operator-> () { return &detector; }

  const vpDetectorAprilTag* operator-> () const { return &detector; }

  operator vpDetectorAprilTag& () { return detector; }

  operator const vpDetectorAprilTag& () const { return detector; }

};

class DetectorAprilTag : public Detector
{
    DetectorAprilTagWrapperPtr _detector;
    const int _tag_id;
    const double _tag_size_meters;

    std::vector< vpPoint > compute3DPoints() const;

public:
    DetectorAprilTag( DetectorAprilTagWrapperPtr detector,
        const vpCameraParameters &cam_parameters,
        const int tag_id,
        const double tag_size_meters );

    bool analyseImage( const GrayImage_t& I );

    bool detect();

    void drawDebug( GrayImage_t& I ) const;

    size_t id() const
    {
      return _tag_id;
    }

    static std::array< vpPoint, 4 > compute3DPoints(const double size);
};

}
}

#endif // __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
