#include "tracker_object/detector_chessboard.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpPoint.h>


DetectorChessboard::DetectorChessboard( const vpCameraParameters &cam_parameters, const int chessboard_nb_corners_w, 
                                        const int chessboard_nb_corners_h, const double chessboard_square_size_meters )
  : Detector( cam_parameters )
  , _chessboard_size{ chessboard_nb_corners_w, chessboard_nb_corners_h } 
  , _chessboard_square_size_meters{ chessboard_square_size_meters }
{}
    
bool DetectorChessboard::detect( const vpImage< unsigned char > &gray_image )
{
  _image_points.clear();
  _image_points_cv.clear();

  cv::Mat mat_image{};
  vpImageConvert::convert( gray_image, mat_image );

  if( cv::findChessboardCorners( mat_image, _chessboard_size, _image_points_cv,
          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE ) )
  {
    cv::cornerSubPix( mat_image, _image_points_cv, cv::Size(11,11), cv::Size(-1,-1),
        cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1 ) );

    for( unsigned int i{ 0 } ; i < _image_points_cv.size() ; ++i )
      _image_points.emplace_back( _image_points_cv[ i ].x, _image_points_cv[ i ].y );

    if( _state == no_object )
        _state = newly_acquired_object;

    computePose();
    return true;
  }

  _state = no_object;
  return false;
}
    
    
void DetectorChessboard::drawDebug( vpImage< vpRGBa > &image ) const
{
  if( _state == no_object )
    return;

  cv::Mat mat_image{};
  vpImageConvert::convert( image, mat_image );

  cv::drawChessboardCorners( mat_image, _chessboard_size, _image_points_cv, true );
  
  vpImageConvert::convert( mat_image, image );
  vpDisplay::display( image );
}


std::vector< vpPoint > DetectorChessboard::compute3DPoints() const
{
  std::vector< vpPoint > points{};

  vpPoint pt{};
  pt.set_oZ( 0.0 );

  for( int i{ 0 } ; i < _chessboard_size.width ; ++i ) 
    for( int j{ 0 } ; j < _chessboard_size.height ; ++j )
    {
      pt.set_oX( i * _chessboard_square_size_meters );
      pt.set_oY( j * _chessboard_square_size_meters );
      points.push_back(pt);
    }

  return points;
}
