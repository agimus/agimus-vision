#include "tracker_object/detector.hpp"

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpPixelMeterConversion.h>


Detector::Detector( const vpCameraParameters &cam_parameters )
  : _cam_parameters{ cam_parameters }
  , _state{ no_object }
{}

bool Detector::analyseImage( const vpImage< unsigned char > &gray_image )
{
    return false;
}

bool Detector::detect()
{
    return false;
}

void Detector::drawDebug( vpImage< vpRGBa > &image ) const
{}

vpHomogeneousMatrix Detector::getLastCMO() const
{
    return _cMo;
}

// -----
// PRIVATE
// -----

void Detector::computePose()
{
    if( _state == no_object )
        return;

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

        pose.computePose( vpPose::DEMENTHON, cMo_dem );
        pose.computePose( vpPose::LAGRANGE, cMo_lag );

        if( pose.computeResidual( cMo_dem ) < pose.computeResidual( cMo_lag ) )
            _cMo = cMo_dem;
        else
            _cMo = cMo_lag;
    }
 
    pose.computePose( vpPose::VIRTUAL_VS, _cMo );
    _state = already_acquired_object;
}
    
std::vector< vpPoint > Detector::compute3DPoints() const
{
    return std::vector< vpPoint >{};
}
