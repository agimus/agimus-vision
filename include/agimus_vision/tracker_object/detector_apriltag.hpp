#ifndef __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 

#include "agimus_vision/tracker_object/detector.hpp"
#include "agimus_vision/tracker_object/fwd.hpp"

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpImageConvert.h>
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
    float _depth_scale;
    std::vector< vpPoint > compute3DPoints() const;
    std::map<int, double> tags_size;
public:
    DetectorAprilTag( DetectorAprilTagWrapperPtr detector,
        const vpCameraParameters &cam_parameters,
        const vpCameraParameters &depth_cam_parameters,
        const int tag_id,
        const double tag_size_meters );

    bool analyseImage( const GrayImage_t& I );

    bool detectOnDepthImage(const DepthMap_t& I, float depthScale);

    bool detect();

    

    void drawDebug( GrayImage_t& I ) const;

    size_t id() const
    {
      return _tag_id;
    }
    
    float depth_scale()
    {
      return _depth_scale;
    }

    static std::array< vpPoint, 4 > compute3DPoints(const double size);
};

}
}

#endif // __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__ 
