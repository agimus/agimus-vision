#include "agimus_vision/extrinsic_calibration/extrinsic_calibration.hpp"

#include <ros/ros.h>


ExtrinsicCalibration::ExtrinsicCalibration()
{}

void ExtrinsicCalibration::addPose( const vpHomogeneousMatrix& cMo, const vpHomogeneousMatrix& wMe )
{
    _cMo_vec.push_back( cMo );
    _wMe_vec.push_back( wMe );
}

vpHomogeneousMatrix ExtrinsicCalibration::getEMC()
{
    vpHomogeneousMatrix eMc;
    vpCalibration::calibrationTsai( _cMo_vec, _wMe_vec, eMc );

    return eMc;
}

void ExtrinsicCalibration::saveEMCToFile( std::string filename )
{
    auto eMc{ getEMC() };

    auto endsWith = []( std::string str, std::string suffix ){
        return str.size() >= suffix.size() && str.compare( str.size() - suffix.size(), suffix.size(), suffix ) == 0;
    };
    
    if( endsWith( filename, "txt" ) )
    {
        std::ofstream file{ filename };
        eMc.save( file );
    }
    else if( endsWith( filename, "yaml" ) )
    {
        vpPoseVector pose{ eMc };
        pose.saveYAML( filename, pose );
    }
    else
        ROS_ERROR("Could not save the eMc pose: wrong file extension (only supports .txt and .yaml)");
}

unsigned int ExtrinsicCalibration::getNbPose() const 
{
    return (unsigned int)_cMo_vec.size();
}
