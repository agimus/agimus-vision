#include "agimus_vision/tracker_object/detector.hpp"

#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpPoseException.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include <ros/console.h>

namespace agimus_vision {
namespace tracker_object {

Detector::Detector( const vpCameraParameters &cam_parameters )
  : _state{ no_object }
  , _cam_parameters{ cam_parameters }
  , _residual_thr (1e-4)
  , _pose_thr (1e-3)
{}

bool Detector::analyseImage( const vpImage< unsigned char > &/*gray_image*/ )
{
    return false;
}

bool Detector::detect()
{
    return false;
}

void Detector::drawDebug( GrayImage_t&/*I*/ ) const
{}

// -----
// PRIVATE
// -----



bool Detector::computePose()
{
    if( _state == no_object )
        return false;

    vpPose pose{};
    std::vector< vpPoint > points{ compute3DPoints() };

    // Compute the 2D coord. of the points (in meters) to match the 3D coord. for the pose estimation 
    for( unsigned int i{ 0 } ; i < points.size() ; i++ ) 
    {
        double x{ 0. }, y{ 0. };
        vpPixelMeterConversion::convertPointWithoutDistortion( _cam_parameters, _image_points[ i ], x, y );
        
        // x, y are the coordinates in the image plane, oX, oY, oZ in the world, and X, Y, Z in the camera ref. frame
        points[ i ].set_x( x );
        points[ i ].set_y( y );
        pose.addPoint( points[ i ] );
    }

    if( _state == newly_acquired_object )
    {
        vpHomogeneousMatrix cMo_dem{}, cMo_lag{};
 
        enum { DEM_OK = 1, LAG_OK = 2 };
        short status = 0;
        double res_cMo_dem = -1, res_cMo_lag = -1;
        try {
          pose.computePose( vpPose::DEMENTHON_VIRTUAL_VS, cMo_dem );
          res_cMo_dem = pose.computeResidual( cMo_dem );
          status = DEM_OK;
        } catch (const vpPoseException& exc) {
          ROS_WARN_STREAM("Could not apply DEMENTHON: " << exc.what());
        }

        try {
          pose.computePose( vpPose:: LAGRANGE_VIRTUAL_VS, cMo_lag );
          res_cMo_lag = pose.computeResidual( cMo_lag );
          status |= LAG_OK;
        } catch (const vpPoseException& exc) {
          ROS_WARN_STREAM("Could not apply LAGRANGE: " << exc.what());
        }

        bool dem_over_lag;
        switch (status) {
          case 0:
            return false;
          case DEM_OK:
            dem_over_lag = true;
            break;
          case LAG_OK:
            dem_over_lag = false;
            break;
          case DEM_OK | LAG_OK:
            dem_over_lag = dementhon_over_lagrange (cMo_dem, res_cMo_dem, cMo_lag, res_cMo_lag);
            break;
        }
        if( dem_over_lag ) {
            assert(res_cMo_dem >= 0);
            _cMo = cMo_dem;
            _error = res_cMo_dem;
        } else {
            assert(res_cMo_lag >= 0);
            _cMo = cMo_lag;
            _error = res_cMo_lag;
        }
        return true;
    }
 
    assert(_state == already_acquired_object);
    pose.computePose( vpPose::VIRTUAL_VS, _cMo );
    _error = pose.computeResidual( _cMo );
    return true;
}

bool Detector::dementhon_over_lagrange (
        const vpHomogeneousMatrix &cMo_dem, const double& res_cMo_dem,
        const vpHomogeneousMatrix &cMo_lag, const double& res_cMo_lag)
{
  if (res_cMo_dem < _residual_thr && res_cMo_lag < _residual_thr)
  {
    vpColVector diff = vpExponentialMap::inverse (cMo_dem.inverse() * cMo_lag);
#if VISP_VERSION_INT >= VP_VERSION_INT(3,3,0)
    if (diff.frobeniusNorm() > _pose_thr)
#else
    if (diff.euclideanNorm() > _pose_thr)
#endif
      ROS_WARN_STREAM_THROTTLE(1, "The computation of pose of id " << id() <<
          " is ambiguous.\n"
          "Lagrange residual: " << res_cMo_lag << "\n"
          "Dementhon residual: " << res_cMo_dem << "\n"
          "Transform difference: " << diff.t());
    // TODO Always prefer Lagrange ? (from source code, it seems it has no ambiguities...)
    // return false;
    return res_cMo_dem < res_cMo_lag;
  } else
    return res_cMo_dem < res_cMo_lag;
}

std::vector< vpPoint > Detector::compute3DPoints() const
{
    return std::vector< vpPoint >{};
}

}
}
