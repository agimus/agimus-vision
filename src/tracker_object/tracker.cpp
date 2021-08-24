#include "agimus_vision/tracker_object/tracker.hpp"

#include <visp3/core/vpXmlParser.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpPose.h>
#include <string>
#include <vector>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpRobust.h>
// #include <visp3/gui/vpDisplayGDI.h>
// #include <visp3/gui/vpDisplayX.h>
namespace agimus_vision
{
  namespace tracker_object
  {

    void Tracker::process(const GrayImage_t &I, const DepthMap_t &D, const double time, float depthScale)
    {
      vpHomogeneousMatrix cMo;
      if (n_ > 0)
        --n_;
      if (state_ == state_detection)
      {
        if (n_ > 0)
          return;
        state_ = initialization_->detect(I);
        n_ = detectionSubsampling_;

        // Initialize the tracker with the result of the detection
        if (state_ == state_tracking)
        {
          initialization_->getPose(cMo);
          tracking_->init(I, cMo);
          if (filtering_)
            filtering_->reset();
        }
      }

      if (state_ == state_tracking)
      {
        state_ = tracking_->track(I, D, depthScale);
        if (filtering_)
        {
          vpHomogeneousMatrix cMo;
          tracking_->getPose(cMo);
          filtering_->filter(cMo, time);
        }
      }
    }

    void Tracker::drawDebug(GrayImage_t &I)
    {
      if (state_ == state_detection)
        initialization_->drawDebug(I);
      else if (state_ == state_tracking)
        tracking_->drawDebug(I);
    }

    namespace initializationStep
    {

      class AprilTagParser : public vpXmlParser
      {
      public:
        typedef enum
        {
          apriltag,
          nb_threads,
          quad_decimate,
          quad_sigma,
        } dataToParse;

        AprilTagParser()
        {
          nodeMap["apriltag"] = apriltag;

          nodeMap["nb_threads"] = nb_threads;
          nodeMap["quad_decimate"] = quad_decimate;
          nodeMap["quad_sigma"] = quad_sigma;
        }

        void configure(vpDetectorAprilTag &detector)
        {
          std::cout << "*********** Parsing XML for april tag detection ************\n";

          if (setParameters.count(nb_threads))
          {
            detector.setAprilTagNbThreads(nbThreads);
            std::cout << "apriltag : nb_threads : " << nbThreads << "\n";
          }
          if (setParameters.count(quad_decimate))
          {
            detector.setAprilTagQuadDecimate(quadDecimate);
            std::cout << "apriltag : quad_decimate : " << quadDecimate << "\n";
          }
          if (setParameters.count(quad_sigma))
          {
            detector.setAprilTagQuadSigma(quadSigma);
            std::cout << "apriltag : quad_sigma : " << quadSigma << "\n";
          }
          std::cout << std::flush;
        }

      private:
        void readMainClass(xmlDocPtr doc, xmlNodePtr node)
        {
          for (xmlNodePtr tmpNode = node->xmlChildrenNode; tmpNode != NULL; tmpNode = tmpNode->next)
          {
            if (tmpNode->type == XML_ELEMENT_NODE)
            {
              std::map<std::string, int>::iterator iter = this->nodeMap.find((const char *)tmpNode->name);
              if (iter == nodeMap.end())
              {
                continue;
              }
              switch (iter->second)
              {
              case apriltag:
                readAprilTag(doc, tmpNode);
                break;
              default:
                break;
              }
            }
          }
        }

        void writeMainClass(xmlNodePtr node)
        {
          (void)node;
        }

        void readAprilTag(xmlDocPtr doc, xmlNodePtr node)
        {
          for (xmlNodePtr child = node->children; child != NULL; child = child->next)
          {
            if (child->type == XML_ELEMENT_NODE)
            {
              std::map<std::string, int>::iterator iter = this->nodeMap.find((const char *)child->name);
              if (iter == nodeMap.end())
              {
                continue;
              }
              switch (iter->second)
              {
              case nb_threads:
                nbThreads = xmlReadIntChild(doc, child);
                setParameters.insert(nb_threads);
                break;
              case quad_decimate:
                quadDecimate = xmlReadFloatChild(doc, child);
                setParameters.insert(quad_decimate);
                break;
              case quad_sigma:
                quadSigma = xmlReadFloatChild(doc, child);
                setParameters.insert(quad_sigma);
                break;
              default:
                break;
              }
            }
          }
        }

        int nbThreads;
        float quadDecimate, quadSigma;

        std::set<int> setParameters;
      };

      void AprilTag::configure(vpDetectorAprilTag &detector,
                               const std::string &configFile)
      {
        AprilTagParser parser;
        parser.parse(configFile);
        parser.configure(detector);
      }

      State AprilTag::detect(const GrayImage_t &I)
      {
        // Check if the object is detected.
        if (!detectTags(I))
          return state_detection;

        // Pose estimation
        vpHomogeneousMatrix cMt;
        try
        {
          const DetectedTag &detectedTag(detectedTags_[0]);
          if (detector_->detector.getPose(detectedTag.i, detectedTag.tag->size, cam_, cMt))
          {
            cMo_ = cMt * detectedTag.tag->oMt.inverse();
            return state_tracking;
          }
        }
        catch (const vpException &e)
        {
          std::cerr << e.what() << std::endl;
        }
        return state_detection;
      }

      void AprilTag::init(const GrayImage_t &, const vpHomogeneousMatrix &cMo)
      {
        cMo_ = cMo;
      }

      State AprilTag::track(const GrayImage_t &I, const vpImage<uint16_t> &D, float depthScale)
      {

        if (!detectTags(I))
          return state_detection;
        vpPose pose;
        tags_size[-1] = 0.0845;
        vpImage<float> depthMap;
        vpImage<unsigned char> depthImage;
        std::vector<int> tags_id;
        
        bool bGot15 = false;
        vpHomogeneousMatrix new_cMo_;
        depthMap.resize(D.getHeight(), D.getWidth());
        for (unsigned int i = 0; i < D.getHeight(); i++)
        {
          for (unsigned int j = 0; j < D.getWidth(); j++)
          {
            if (D[i][j])
            {
              float Z = D[i][j] * depthScale;
              depthMap[i][j] = Z;
            }
            else
            {
              depthMap[i][j] = 0;
            }
          }
        }

      
        for (const DetectedTag &dtag : detectedTags_)
        {

          std::vector<vpImagePoint> imagePoints = detector_->detector.getPolygon(dtag.i);
          std::array<vpPoint, 4> tPs = DetectorAprilTag::compute3DPoints(dtag.tag->size);
          std::vector<vpPoint> points3d;
          std::vector<vpImagePoint> points2d;
        
          for (unsigned int j = 0; j < tPs.size(); j++)
          {

            double x{0.}, y{0.};
            vpPixelMeterConversion::convertPointWithoutDistortion(cam_, imagePoints[j], x, y);
             
            // x, y are the coordinates in the image plane, oX, oY, oZ in the world,
            // and X, Y, Z in the camera ref. frame
            tPs[j].set_x(x);
            tPs[j].set_y(y);

            tPs[j].oP = dtag.tag->oMt * tPs[j].oP;

            //addd 2d points to vectors
            // if (dtag.tag->id == 15)
            // {
            points2d.push_back(imagePoints[j]);
            points3d.push_back(tPs[j]);
            // }
            // pose.addPoint(tPs[j]);
            
            if ((points3d.size() > 3) && (points2d.size() == points3d.size()))
            {
              double confidence_index;
              vpHomogeneousMatrix tag_cMo_;
              vpPolygon polygon(points2d);
              vpRect bb = polygon.getBoundingBox();
              unsigned int top = static_cast<unsigned int>(std::max(0, static_cast<int>(bb.getTop())));
              unsigned int bottom = static_cast<unsigned int>(std::min(static_cast<int>(depthMap.getHeight()) - 1, static_cast<int>(bb.getBottom())));
              unsigned int left = static_cast<unsigned int>(std::max(0, static_cast<int>(bb.getLeft())));
              unsigned int right = static_cast<unsigned int>(std::min(static_cast<int>(depthMap.getWidth()) - 1, static_cast<int>(bb.getRight())));

              vpDisplay::displayRectangle(I, top, left, right - left, bottom - top, vpColor::red, false);
              vpDisplay::flush(I);
              std::vector<vpPoint> pose_points;
              if (computePlanarPoseFromRGBD(depthMap, points2d, cam_, points3d, tag_cMo_, pose_points, &confidence_index))
              {
                // ROS_WARN_STREAM("For tag " + std::to_string(dtag.tag->id) + 
                //                  ", the points added: " + std::to_string(pose_points.size()));
                pose.addPoints(pose_points);
              }
            }
          }
        }
      // ROS_WARN_STREAM("Total number of points added: " + std::to_string(pose.npt));
      if (pose.npt > 3)
        pose.computePose(vpPose::VIRTUAL_VS, cMo_);

      return state_tracking;
      }

      bool AprilTag::detectTags(const GrayImage_t &I)
      {
        if (!detector_->detect(I))
          return false;

        // Check if the object is detected.
        detectedTags_.clear();
        for (std::size_t i = 0; i < detector_->detector.getNbObjects(); i++)
          for (Tag &tag : tags_)
            if (tag.message == detector_->detector.getMessage(i))
              detectedTags_.push_back(DetectedTag({i, &tag}));
        return !detectedTags_.empty();
      }

      void AprilTag::drawDebug(GrayImage_t &I)
      {
        if (detectedTags_.empty())
          return;

        std::array<vpColor, 4> colors{{vpColor::red, vpColor::green, vpColor::blue, vpColor::cyan}};
        //Fix Bug: while tracking, if tag is out of view, agimus_vision stopped => add try catch
        try
        {
          for (const DetectedTag &dtag : detectedTags_)
          {
            std::vector<vpImagePoint> &points(detector_->detector.getPolygon(dtag.i));
            for (unsigned int i{0}; i < 4; ++i)
              vpDisplay::displayLine(I, points[i], points[(i + 1) % 3], colors[i], 3);
          }
          vpDisplay::displayFrame(I, cMo_, cam_, detectedTags_[0].tag->size * 2, vpColor::none);
        }
        catch (vpException e)
        {
          std::cout << "Catch an exception: " << e << std::endl;
        }
      }

      bool AprilTag::addTag(int id, double size, vpHomogeneousMatrix oMt)
      {
        for (Tag tag : tags_)
          if (tag.id == id)
            return false;

        Tag tag;
        tag.id = id;
        tag.message = "36h11 id: " + std::to_string(id);
        tag.oMt = oMt;
        tag.size = size;
        tags_.push_back(tag);
        tags_size[id] = size;
        return true;
      }

      vpHomogeneousMatrix AprilTag::compute3d3dTransformation(const std::vector<vpPoint> &p,
                                                              const std::vector<vpPoint> &q)
      {
        double N = static_cast<double>(p.size());

        vpColVector p_bar(3, 0.0);
        vpColVector q_bar(3, 0.0);
        for (size_t i = 0; i < p.size(); i++)
        {
          for (unsigned int j = 0; j < 3; j++)
          {
            p_bar[j] += p[i].oP[j];
            q_bar[j] += q[i].oP[j];
          }
        }

        for (unsigned int j = 0; j < 3; j++)
        {
          p_bar[j] /= N;
          q_bar[j] /= N;
        }

        vpMatrix pc(static_cast<unsigned int>(p.size()), 3);
        vpMatrix qc(static_cast<unsigned int>(q.size()), 3);

        for (unsigned int i = 0; i < static_cast<unsigned int>(p.size()); i++)
        {
          for (unsigned int j = 0; j < 3; j++)
          {
            pc[i][j] = p[i].oP[j] - p_bar[j];
            qc[i][j] = q[i].oP[j] - q_bar[j];
          }
        }

        vpMatrix pct_qc = pc.t() * qc;
        vpMatrix U = pct_qc, V;
        vpColVector W;
        U.svd(W, V);

        vpMatrix Vt = V.t();
        vpMatrix R = U * Vt;

        double det = R.det();
        if (det < 0)
        {
          Vt[2][0] *= -1;
          Vt[2][1] *= -1;
          Vt[2][2] *= -1;

          R = U * Vt;
        }

        vpColVector t = p_bar - R * q_bar;

        vpHomogeneousMatrix cMo;
        for (unsigned int i = 0; i < 3; i++)
        {
          for (unsigned int j = 0; j < 3; j++)
          {
            cMo[i][j] = R[i][j];
          }
          cMo[i][3] = t[i];
        }

        return cMo;
      }
      void AprilTag::estimatePlaneEquationSVD(const std::vector<double> &point_cloud_face,
                                              vpColVector &plane_equation_estimated, vpColVector &centroid,
                                              double &normalized_weights)
      {
        unsigned int max_iter = 10;
        double prev_error = 1e3;
        double error = 1e3 - 1;
        unsigned int nPoints = static_cast<unsigned int>(point_cloud_face.size() / 3);

        vpColVector weights(nPoints, 1.0);
        vpColVector residues(nPoints);
        vpMatrix M(nPoints, 3);
        vpRobust tukey;
        tukey.setMinMedianAbsoluteDeviation(1e-4);
        vpColVector normal;

        plane_equation_estimated.resize(4, false);
        for (unsigned int iter = 0; iter < max_iter && std::fabs(error - prev_error) > 1e-6; iter++)
        {
          if (iter != 0)
          {
            tukey.MEstimator(vpRobust::TUKEY, residues, weights);
          }

          // Compute centroid
          double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
          double total_w = 0.0;

          for (unsigned int i = 0; i < nPoints; i++)
          {
            centroid_x += weights[i] * point_cloud_face[3 * i + 0];
            centroid_y += weights[i] * point_cloud_face[3 * i + 1];
            centroid_z += weights[i] * point_cloud_face[3 * i + 2];
            total_w += weights[i];
          }

          centroid_x /= total_w;
          centroid_y /= total_w;
          centroid_z /= total_w;

          // Minimization
          for (unsigned int i = 0; i < nPoints; i++)
          {
            M[static_cast<unsigned int>(i)][0] = weights[i] * (point_cloud_face[3 * i + 0] - centroid_x);
            M[static_cast<unsigned int>(i)][1] = weights[i] * (point_cloud_face[3 * i + 1] - centroid_y);
            M[static_cast<unsigned int>(i)][2] = weights[i] * (point_cloud_face[3 * i + 2] - centroid_z);
          }

          vpColVector W;
          vpMatrix V;
          vpMatrix J = M.t() * M;
          J.svd(W, V);

          double smallestSv = W[0];
          unsigned int indexSmallestSv = 0;
          for (unsigned int i = 1; i < W.size(); i++)
          {
            if (W[i] < smallestSv)
            {
              smallestSv = W[i];
              indexSmallestSv = i;
            }
          }

          normal = V.getCol(indexSmallestSv);

          // Compute plane equation
          double A = normal[0], B = normal[1], C = normal[2];
          double D = -(A * centroid_x + B * centroid_y + C * centroid_z);

          // Update plane equation
          plane_equation_estimated[0] = A;
          plane_equation_estimated[1] = B;
          plane_equation_estimated[2] = C;
          plane_equation_estimated[3] = D;

          // Compute error points to estimated plane
          prev_error = error;
          error = 0.0;
          for (unsigned int i = 0; i < nPoints; i++)
          {
            residues[i] = std::fabs(A * point_cloud_face[3 * i] + B * point_cloud_face[3 * i + 1] +
                                    C * point_cloud_face[3 * i + 2] + D) /
                          sqrt(A * A + B * B + C * C);
            error += weights[i] * residues[i];
          }
          error /= total_w;
        }

        // Update final weights
        tukey.MEstimator(vpRobust::TUKEY, residues, weights);

        // Update final centroid
        centroid.resize(3, false);
        double total_w = 0.0;

        for (unsigned int i = 0; i < nPoints; i++)
        {
          centroid[0] += weights[i] * point_cloud_face[3 * i];
          centroid[1] += weights[i] * point_cloud_face[3 * i + 1];
          centroid[2] += weights[i] * point_cloud_face[3 * i + 2];
          total_w += weights[i];
        }

        centroid[0] /= total_w;
        centroid[1] /= total_w;
        centroid[2] /= total_w;

        // Compute final plane equation
        double A = normal[0], B = normal[1], C = normal[2];
        double D = -(A * centroid[0] + B * centroid[1] + C * centroid[2]);

        // Update final plane equation
        plane_equation_estimated[0] = A;
        plane_equation_estimated[1] = B;
        plane_equation_estimated[2] = C;
        plane_equation_estimated[3] = D;

        normalized_weights = total_w / nPoints;
      }
      double AprilTag::computeZMethod1(const vpColVector &plane_equation, double x, double y)
      {
        return -plane_equation[3] / (plane_equation[0] * x + plane_equation[1] * y + plane_equation[2]);
      }

      bool AprilTag::validPose(const vpHomogeneousMatrix &cMo)
      {
        bool valid = true;

        for (unsigned int i = 0; i < cMo.getRows() && valid; i++)
        {
          for (unsigned int j = 0; j < cMo.getCols() && valid; j++)
          {
            if (vpMath::isNaN(cMo[i][j]))
            {
              valid = false;
            }
          }
        }

        return valid;
      }

      bool AprilTag::computePlanarPoseFromRGBD(const vpImage<float> &depthMap, const std::vector<vpImagePoint> &corners,
                                   const vpCameraParameters &colorIntrinsics, const std::vector<vpPoint> &point3d,
                                   vpHomogeneousMatrix &cMo, std::vector<vpPoint> &pose_points, double *confidence_index)
      {
        if (corners.size() != point3d.size())
        {
          throw(vpException(vpException::fatalError, "Cannot compute pose from RGBD, 3D (%d) and 2D (%d) data doesn't have the same size",
                            point3d.size(), corners.size()));
        }
        // std::vector<vpPoint> pose_points;
        if (confidence_index != NULL)
        {
          *confidence_index = 0.0;
        }

        for (size_t i = 0; i < point3d.size(); i++)
        {
          pose_points.push_back(point3d[i]);
        }

        vpPolygon polygon(corners);
        vpRect bb = polygon.getBoundingBox();
        unsigned int top = static_cast<unsigned int>(std::max(0, static_cast<int>(bb.getTop())));
        unsigned int bottom = static_cast<unsigned int>(std::min(static_cast<int>(depthMap.getHeight()) - 1, static_cast<int>(bb.getBottom())));
        unsigned int left = static_cast<unsigned int>(std::max(0, static_cast<int>(bb.getLeft())));
        unsigned int right = static_cast<unsigned int>(std::min(static_cast<int>(depthMap.getWidth()) - 1, static_cast<int>(bb.getRight())));

        std::vector<double> points_3d;
        points_3d.reserve((bottom - top) * (right - left));
        for (unsigned int idx_i = top; idx_i < bottom; idx_i++)
        {
          for (unsigned int idx_j = left; idx_j < right; idx_j++)
          {
            vpImagePoint imPt(idx_i, idx_j);
            if (depthMap[idx_i][idx_j] > 0 && polygon.isInside(imPt))
            {
              double x = 0, y = 0;
              vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
              double Z = depthMap[idx_i][idx_j];
              points_3d.push_back(x * Z);
              points_3d.push_back(y * Z);
              points_3d.push_back(Z);
            }
          }
        }

        unsigned int nb_points_3d = static_cast<unsigned int>(points_3d.size() / 3);

        if (nb_points_3d > 4)
        {
          std::vector<vpPoint> p, q;

          // Plane equation
          vpColVector plane_equation, centroid;
          double normalized_weights = 0;
          estimatePlaneEquationSVD(points_3d, plane_equation, centroid, normalized_weights);

          for (size_t j = 0; j < corners.size(); j++)
          {
            const vpImagePoint &imPt = corners[j];
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(colorIntrinsics, imPt.get_u(), imPt.get_v(), x, y);
            double Z = computeZMethod1(plane_equation, x, y);
            if (Z < 0)
            {
              Z = -Z;
            }
            p.push_back(vpPoint(x * Z, y * Z, Z));

            pose_points[j].set_x(x);
            pose_points[j].set_y(y);
          }

          for (size_t i = 0; i < point3d.size(); i++)
          {
            q.push_back(point3d[i]);
          }

          cMo = compute3d3dTransformation(p, q);

          if (validPose(cMo))
          {
            vpPose pose;
            pose.addPoints(pose_points);
            if (pose.computePose(vpPose::VIRTUAL_VS, cMo))
            {
              if (confidence_index != NULL)
              {
                *confidence_index = std::min(1.0, normalized_weights * static_cast<double>(nb_points_3d) / polygon.getArea());
              }
              return true;
            }
          }
        }

        return false;
      }
    } //end namespace

    namespace trackingStep
    {

      ModelBased::ModelBased(int trackerType,
                             const std::string &modelFile,
                             const vpCameraParameters &cam,
                             double projectionErrorThr,
                             const std::string &configFile)
      {
        tracker().setTrackerType(trackerType);
        if (!configFile.empty())
#if VISP_VERSION_INT >= VP_VERSION_INT(3, 2, 1)
          tracker().loadConfigFile(configFile);
#else
#define _STRINGIFY(x) #x
#define _TOSTRING(x) _STRINGIFY(x)
          std::cerr << "Cannot load ViSP config file " << configFile << " because "
                                                                        "installed ViSP version (" _TOSTRING(VISP_VERSION) ") is inferior to 3.2.1."
                    << std::endl;
#undef _TOSTRING
#undef _STRINGIFY
#endif

        // camera calibration params
        tracker().setCameraParameters(cam);
        // model definition
        tracker().loadModel(modelFile);
        projectionErrorThreshold(projectionErrorThr);
      }

      void ModelBased::init(const GrayImage_t &I,
                            const vpHomogeneousMatrix &cMo)
      {
        tracker_.initFromPose(I, cMo);
      }

      State ModelBased::track(const GrayImage_t &I)
      {
        try
        {
          tracker_.track(I);
        }
        catch (const vpException &e)
        {
          std::cerr << e.what() << std::endl;
          return state_detection;
        }

        vpCameraParameters cam;
        tracker_.getCameraParameters(cam);

        // Detect tracking error
        double projection_error = tracker_.computeCurrentProjectionError(I,
                                                                         tracker_.getPose(), cam);
        if (projection_error > projErrorThr_)
          return state_detection;

        return state_tracking;
      }

      void ModelBased::getPose(vpHomogeneousMatrix &cMo) const
      {
        tracker_.getPose(cMo);
      }

      void ModelBased::drawDebug(GrayImage_t &I)
      {
        vpCameraParameters cam;
        tracker_.getCameraParameters(cam);

        // Display
        tracker_.display(I, tracker_.getPose(), cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, tracker_.getPose(), cam, 0.025, vpColor::none, 3);
        vpDisplay::displayText(I, 40, 20, "State: tracking in progress", vpColor::red);
      }

    }

    namespace filteringStep
    {
      void PositionLowPassFirstOrder::filter(const vpHomogeneousMatrix &M, const double time)
      {
        if (lastT_ < 0)
        {
          lastT_ = time;
          M_ = M;
          vel_.resize(6, true);
          return;
        }
        double dt = time - lastT_;
        const double alpha = 1 / (1 + 1 / (2 * M_PI * f_ * dt));

        vpColVector vel = vpExponentialMap::inverse(M_.inverse() * M);
        M_ = M_ * vpExponentialMap::direct(alpha * vel);
      }
      void PositionLowPassFirstOrder::reconfigure(TrackerConfig &config, uint32_t level)
      {
        f_ = config.groups.filters.low_pass.cut_frequency;
      }

      void PositionLowPassOrder::filter(const vpHomogeneousMatrix &M, const double time)
      {
        M_ = M;
        for (PositionLowPassFirstOrder &f : filters_)
        {
          f.filter(M_, time);
          f.getPose(M_);
        }
        lastT_ = time;
      }

      void PositionLowPassOrder::reset()
      {
        FilteringStep::reset();
        for (PositionLowPassFirstOrder &f : filters_)
          f.reset();
      }

      void PositionLowPassOrder::reconfigure(TrackerConfig &config, uint32_t level)
      {
        for (PositionLowPassFirstOrder &f : filters_)
          f.reconfigure(config, level);
      }
    }

  }
}
