#ifndef __TRACKER_OBJECT__DETECTOR_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_HPP__ 

#include <memory>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpImage.h>

namespace agimus_vision {
namespace tracker_object {

class Detector
{
protected:

    enum State { no_object, newly_acquired_object, already_acquired_object };
    State _state;
    
    vpCameraParameters _cam_parameters;

    std::vector< vpImagePoint > _image_points;

    vpHomogeneousMatrix _cMo;

    double _error;

    bool computePose();

    virtual std::vector< vpPoint > compute3DPoints() const;

public:
    Detector( const vpCameraParameters &cam_parameters );

    virtual size_t id() const = 0;
    
    virtual bool analyseImage( const vpImage< unsigned char > &gray_image );

    virtual bool detect();

    virtual void drawDebug( vpImage< vpRGBa > &image ) const;

    vpHomogeneousMatrix getLastCMO() const
    {
      return _cMo;
    }

    double error() const
    {
      return _error;
    }
};

typedef std::shared_ptr<Detector> DetectorPtr;

}
}

#endif // __TRACKER_OBJECT__DETECTOR_HPP__ 
