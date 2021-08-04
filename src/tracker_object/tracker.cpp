#include "agimus_vision/tracker_object/tracker.hpp"

#include <visp3/core/vpXmlParser.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpPose.h>
#include <string>
#include <vector>

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

        tags_size[-1] = 0.0845;
        vpImage<float> depthMap;
        vpImage<unsigned char> depthImage;
        vpImageConvert::convert(D, depthImage);
        std::vector<int> tags_id;
        std::vector<vpPoint> points3d;
        std::vector<vpImagePoint> points2d;

        depthMap.resize(depthImage.getHeight(), depthImage.getWidth());
        for (unsigned int i = 0; i < depthImage.getHeight(); i++)
        {
          for (unsigned int j = 0; j < depthImage.getWidth(); j++)
          {
            if (depthImage[i][j])
            {
              float Z = depthImage[i][j] * depthScale;
              depthMap[i][j] = Z;
            }
            else
            {
              depthMap[i][j] = 0;
            }
          }
        }
        
        for (const DetectedTag &dtag : detectedTags_)
          tags_id.push_back(dtag.tag->id);

        
        std::vector<std::vector<vpPoint>> tags_points3d = detector_->detector.getTagsPoints3D(tags_id, tags_size);
        

        for (int i = 0; i < tags_id.size(); i++)
        {
          for (const DetectedTag &dtag : detectedTags_)
          {

            if (dtag.tag->id == tags_id[i])
            {
              //addd 3d points to vectors
              for (unsigned int j = 0; j < tags_points3d[i].size(); j++)
              {
                tags_points3d[i][j].oP = dtag.tag->oMt * tags_points3d[i][j].oP;
                points3d.push_back(tags_points3d[i][j]);
              }

              //addd 2d points to vectors

              std::vector<vpImagePoint> imagePoints = detector_->detector.getPolygon(dtag.i);
              for (int j = 0; j < imagePoints.size(); j++)
                points2d.push_back(imagePoints[j]);
            }
          }
        }

        if ((points3d.size() > 0) && (points2d.size() == points3d.size()))
        {
          double confidence_index;
          vpHomogeneousMatrix new_cMo_;
          if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, points2d, cam_, points3d, new_cMo_, &confidence_index))
          {
              
              vpMatrix newMat = (vpMatrix)new_cMo_;
              vpMatrix oldMat = (vpMatrix)cMo_;
              vpTranslationVector old_trans;
              vpTranslationVector new_trans;
              vpTranslationVector trans;

              new_cMo_.extract(new_trans);
              cMo_.extract(old_trans);
              trans = old_trans - new_trans;

              double trans_dist = sqrt(trans.sumSquare());
              double norm_new = newMat.frobeniusNorm();
              double norm_old = oldMat.frobeniusNorm();
              double distance = abs(norm_new - norm_old);
        
              
              if (distance < 1.0 && trans_dist < 1.0)
                cMo_ = new_cMo_;
              else
              {
                ROS_WARN_STREAM("Frob distance:" + std::to_string(distance));
                ROS_WARN_STREAM("trans distance:" + std::to_string(trans_dist));
              }

              
          }
        }

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

    }

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
