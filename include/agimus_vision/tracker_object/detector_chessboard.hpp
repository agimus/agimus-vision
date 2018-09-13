#ifndef __TRACKER_OBJECT__DETECTOR_CHESSBOARD_HPP__ 
#define __TRACKER_OBJECT__DETECTOR_CHESSBOARD_HPP__ 

#include "agimus_vision/tracker_object/detector.hpp"

#include <visp3/detection/vpDetectorAprilTag.h>

#include <opencv2/core/core.hpp>

namespace agimus_vision {
namespace tracker_object {

class DetectorChessboard : public Detector
{
    const cv::Size _chessboard_size;
    const double _chessboard_square_size_meters;

    std::vector< cv::Point2f > _image_points_cv;

    std::vector< vpPoint > compute3DPoints() const;

public:
    DetectorChessboard( const vpCameraParameters &cam_parameters, const int chessboard_nb_corners_w, 
                        const int chessboard_nb_corners_h, const double chessboard_square_size_meters );

    bool analyseImage( const vpImage< unsigned char > &gray_image );

    bool detect();

    void drawDebug( vpImage< vpRGBa > &image ) const;
};

}
}

#endif // __TRACKER_OBJECT__DETECTOR_CHESSBOARD_HPP__ 
