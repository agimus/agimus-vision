#ifndef __TRACKER_OBJECT__DETECTOR_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_HPP__ 

#include <memory>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpImage.h>

#include "agimus_vision/tracker_object/fwd.hpp"

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

    /// See residualThreshold(double)
    double _residual_thr;
    /// See poseThreshold(double)
    double _pose_thr;

    bool computePose();

    bool computePosetOnDepthImage(const DepthMap_t& I);

    /// Compare the pose computed used vpPose::DEMENTHON_VIRTUAL_VS and
    /// vpPose::LAGRANGE_VIRTUAL_VS.
    /// If both pose have a projection residual below _residual_thr and
    /// if the difference between \b cMo_dem and \b cMo_lag is below _pose_thr,
    /// then a warning is issued.
    bool dementhon_over_lagrange (
        const vpHomogeneousMatrix &cMo_dem, const double& res_cMo_dem,
        const vpHomogeneousMatrix &cMo_lag, const double& res_cMo_lag);

    virtual std::vector< vpPoint > compute3DPoints() const;

public:
    Detector( const vpCameraParameters &cam_parameters );

    virtual size_t id() const = 0;
    
    virtual bool analyseImage( const vpImage< unsigned char > &gray_image );

    virtual bool detect();

    virtual void drawDebug( GrayImage_t& I ) const;

    vpHomogeneousMatrix getLastCMO() const
    {
      return _cMo;
    }

    double error() const
    {
      return _error;
    }

    /// Threshold on the projection residual (of the tag back onto the image).
    void residualThreshold (const double thr)
    {
      _residual_thr = thr;
    }

    /// Threshold on the distance between two transforms.
    /// Above this threshold, they are considered different.
    /// The distance is the norm of the inverse of the exponential map.
    void poseThreshold (const double thr)
    {
      _pose_thr = thr;
    }

    void resetState()
    {
      _state = no_object;
    }
};

typedef std::shared_ptr<Detector> DetectorPtr;

}
}

#endif // __TRACKER_OBJECT__DETECTOR_HPP__ 
