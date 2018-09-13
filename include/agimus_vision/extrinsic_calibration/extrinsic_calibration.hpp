#ifndef __EXTRINSIC_CALIBRATION__EXTRINSIC_CALIBRATION__HPP__
#define __EXTRINSIC_CALIBRATION__EXTRINSIC_CALIBRATION__HPP__

#include <vector>
#include <string>

#include <visp3/vision/vpCalibration.h>

namespace agimus_vision {
namespace extrinsic_calibration {

// Compute the transformation between the end effector containing a camera and the camera frame
class ExtrinsicCalibration
{
    std::vector<vpHomogeneousMatrix> _cMo_vec; // Camera to Object
    std::vector<vpHomogeneousMatrix> _wMe_vec; // World (or, for us, base effector) to Effector containing the camera

public:
    ExtrinsicCalibration();

    void addPose( const vpHomogeneousMatrix& cMo, const vpHomogeneousMatrix& wMe );

    vpHomogeneousMatrix getEMC();

    void saveEMCToFile( std::string filename );

    unsigned int getNbPose() const; 
};

}
}

#endif
