#include "agimus_vision/tracker_object/detector_apriltag.hpp"

#include <ros/param.h>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMeterPixelConversion.h>

namespace agimus_vision
{
    namespace tracker_object
    {

        DetectorAprilTag::DetectorAprilTag(DetectorAprilTagWrapperPtr detector,
                                           const vpCameraParameters &cam_parameters,
                                           const int tag_id,
                                           const double tag_size_meters)
            : Detector(cam_parameters), _detector{detector}, _tag_id{tag_id}, _tag_size_meters{tag_size_meters}
        {
        }

        bool DetectorAprilTag::analyseImage(const vpImage<unsigned char> &gray_image)
        {
            return _detector->detect(gray_image);
        }

        bool DetectorAprilTag::detect()
        {
            _image_points.clear();
            ROS_WARN_STREAM("detector_apriltag::detect");
            for (size_t i{0}; i < _detector->detector.getNbObjects(); ++i)
            {
                std::string tag = " " + std::to_string(_tag_id);
                std::string msg = _detector->detector.getMessage(i);
                size_t stag = tag.size();
                size_t smsg = msg.size();

                // Checks whether msg ends with tag
                if (stag <= smsg && msg.compare(smsg - stag, stag, tag) == 0)
                {
                    _image_points = _detector->detector.getPolygon(i);

                    if (_state == no_object)
                        _state = newly_acquired_object;
                    else
                        _state = already_acquired_object;

                    if (computePose())
                        return true;

                    // Pose could not be computed.
                    _state = no_object;
                    return false;
                }
            }

            _state = no_object;
            return false;
        }

        bool DetectorAprilTag::detectOnDepthImage(const DepthMap_t &D)
        {
            ROS_WARN_STREAM("detector_apriltag::detectOnDepthImage");
            ROS_WARN_STREAM("depth image height:" + D.getHeight());
            vpPose pose;
            vpImage<float> depthMap;

            std::vector<int> tags_id = _detector->detector.getTagsId();
            std::vector<std::vector<vpImagePoint>> tags_corners = _detector->detector.getPolygon();
            std::map<int, double> tags_size;
            tags_size[-1] = 0.1725; //temp hardcoded


            for (std::vector<int>::const_iterator i = tags_id.begin(); i != tags_id.end(); ++i)
                ROS_WARN_STREAM(*i);

            std::vector<std::vector<vpPoint>> tags_points3d = _detector->detector.getTagsPoints3D(tags_id, tags_size);
            for (int i = 0; i < tags_corners.size(); i++)
            {
                vpHomogeneousMatrix cMo;
                std::string tag = " " + std::to_string(_tag_id);
                std::string msg = _detector->detector.getMessage(i);
                size_t stag = tag.size();
                size_t smsg = msg.size();

                // Checks whether msg ends with tag
                if (stag <= smsg && msg.compare(smsg - stag, stag, tag) == 0)
                {
                    double confidence_index;

                    if (_state == no_object)
                        _state = newly_acquired_object;
                    else
                        _state = already_acquired_object;

                    depthMap.resize(D.getHeight(), D.getWidth());
                    for (unsigned int i = 0; i < D.getHeight(); i++)
                    {
                        for (unsigned int j = 0; j < D.getWidth(); j++)
                        {
                            if (D[i][j])
                            {
                                float Z = D[i][j] * 1.0;
                                depthMap[i][j] = Z;
                                std::cout << Z << " ";
                            }
                            else
                            {
                                std::cout << 0 << " ";
                                depthMap[i][j] = 0;
                            }
                        }
                    }

                    if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], _cam_parameters, tags_points3d[i], cMo, &confidence_index))
                    {
                        // if (confidence_index > 0.8)
                        // {
                        //     std::cout << "DISPLAY " << std::endl;
                        //     // vpDisplay::displayFrame(vRGBFusionImage, cMo, colorCamInfoVisp, tagSize / 2, vpColor::none, 3);
                        // }
                        // else if (confidence_index > 0.25)
                        // {
                        //     std::cout << "DISPLAY2 " << std::endl;
                        //     // vpDisplay::displayFrame(vRGBFusionImage, cMo, colorCamInfoVisp, tagSize / 2, vpColor::orange, 3);
                        // }
                        // else
                        // {
                        //     std::cout << "DISPLAY3 " << std::endl;
                        //     // vpDisplay::displayFrame(vRGBFusionImage, cMo, colorCamInfoVisp, tagSize / 2, vpColor::red, 3);
                        // }

                        // std::cout << "Tag ID:" << tags_id[i] << std::endl;
                        return true;
                        // cMo.print();
                        // std::cout << std::endl;
                        // std::stringstream ss;
                        // ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
                        // vpDisplay::displayText(vRGBFusionImage, 35 + i * 15, 20, ss.str(), vpColor::red);
                    }

                    // Pose could not be computed.
                    _state = no_object;
                    return false;
                }
            }

            return false;
        }

        void DetectorAprilTag::drawDebug(GrayImage_t &I) const
        {
            if (_state == no_object)
                return;

            std::array<vpColor, 4> colors{{vpColor::red, vpColor::green, vpColor::blue, vpColor::cyan}};

            for (unsigned int i{0}; i < 4; ++i)
                vpDisplay::displayLine(I, _image_points[i], _image_points[i == 3 ? 0 : i + 1], colors[i], 3);

            vpDisplay::displayFrame(I, _cMo, _cam_parameters, _tag_size_meters * 2, vpColor::none);

            // FIXME this should be fixed as the display is not correct
            /*
    vpPoint a{ - _tag_size_meters / 2. - 0.022, - _tag_size_meters / 2. - 0.066, 0.0 }; a.project( _cMo );
    vpPoint b{ - _tag_size_meters / 2. - 0.022 + 0.05, - _tag_size_meters / 2. - 0.066, 0.0 }; b.project( _cMo ); 
    vpPoint c{ - _tag_size_meters / 2. - 0.022, - _tag_size_meters / 2. - 0.066 + 0.05, 0.0 }; c.project( _cMo ); 
    vpPoint d{ - _tag_size_meters / 2. - 0.022, - _tag_size_meters / 2. - 0.066, 0.0 + 0.05 }; d.project( _cMo ); 

    vpImagePoint pointA, pointB, pointC, pointD;
    vpMeterPixelConversion::convertPoint( _cam_parameters, a.get_x(), a.get_y(), pointA );
    vpMeterPixelConversion::convertPoint( _cam_parameters, b.get_x(), b.get_y(), pointB );
    vpMeterPixelConversion::convertPoint( _cam_parameters, c.get_x(), c.get_y(), pointC );
    vpMeterPixelConversion::convertPoint( _cam_parameters, d.get_x(), d.get_y(), pointD );

    vpDisplay::displayArrow( I, pointA, pointB, vpColor::red );
    vpDisplay::displayArrow( I, pointA, pointC, vpColor::green );
    vpDisplay::displayArrow( I, pointA, pointD, vpColor::blue );
    */
        }

        std::vector<vpPoint> DetectorAprilTag::compute3DPoints() const
        {
            std::array<vpPoint, 4> pts(compute3DPoints(_tag_size_meters));
            return std::vector<vpPoint>(pts.begin(), pts.end());
        }

        std::array<vpPoint, 4> DetectorAprilTag::compute3DPoints(const double size)
        {
            std::array<vpPoint, 4> points;

            for (vpPoint &pt : points)
                pt.set_oZ(0.);
            points[0].set_oX(-size / 2.);
            points[0].set_oY(-size / 2.);
            points[1].set_oX(size / 2.);
            points[1].set_oY(-size / 2.);
            points[2].set_oX(size / 2.);
            points[2].set_oY(size / 2.);
            points[3].set_oX(-size / 2.);
            points[3].set_oY(size / 2.);

            return points;
        }

    }
}
