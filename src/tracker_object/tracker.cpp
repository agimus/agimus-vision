#include "agimus_vision/tracker_object/tracker.hpp"

#include <visp3/core/vpXmlParser.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpPose.h>

namespace agimus_vision
{
  namespace tracker_object
  {

    void Tracker::process(const GrayImage_t &I, const DepthMap_t &D, const double time)
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
        state_ = tracking_->track(I, D);
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
        // ROS_WARN_STREAM("AprilTag::detect");
        // Check if the object is detected.
        if (!detectTags(I))
          return state_detection;

        // Pose estimation
         ROS_WARN_STREAM("AprilTag::detect Pose estimation");
        vpHomogeneousMatrix cMt;
        try
        {
          const DetectedTag &detectedTag(detectedTags_[0]);
          if (detector_->detector.getPose(detectedTag.i, detectedTag.tag->size, cam_, cMt))
          {
            cMo_ = cMt * detectedTag.tag->oMt.inverse();
            //  cMo_ = cMt;
            // ROS_WARN_STREAM("cMo_:");
            // ROS_WARN_STREAM(cMo_);
            // ROS_WARN_STREAM("cMt:");
            // ROS_WARN_STREAM(cMt);
            // ROS_WARN_STREAM("detectedTag:");
            // ROS_WARN_STREAM(detectedTag.tag->oMt.inverse());
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

     


      State AprilTag::track(const GrayImage_t &I, const vpImage<uint16_t> &D)
      {

        if (!detectTags(I))
          return state_detection;

        // Pose estimation
        std::map<int, double> tags_size;

        //default tag size
        tags_size[-1] = 0.04;

        

        vpPose pose;
        vpImage<float> depthMap;
        vpImage<unsigned char> depthImage;
        vpImageConvert::convert(D, depthImage);
        float depthScale = 0.1;
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
        std::vector<int> tags_id = detector_->detector.getTagsId();
        std::vector<std::vector<vpPoint>> tags_points3d = detector_->detector.getTagsPoints3D(tags_id, tags_size);
        std::vector<std::vector<vpImagePoint>> tags_corners = detector_->detector.getPolygon();
      

      // std::vector<int>::iterator it;
      // for (it = tags_id.begin(); it != tags_id.end(); it++)
      // {
      //   // ROS_WARN_STREAM(*it);
      //   // tags_size[*it] = 0.0845;
      //   if (*it == 100 || *it == 101)
      //     tags_size[*it]= 0.0415;
      //   else 
      //     tags_size[*it]= 0.0845;

      // }

          //  tags_size[100] = 0.0415;
          //  tags_size[101] = 0.0415;
          //  tags_size[230] = 0.04;
          //  tags_size[23] = 0.04;

      for (const auto& [key, value] : tags_size) {
        ROS_WARN_STREAM(std::to_string(key) + " = " + std::to_string(value));
      }

        for (int j=0; j < detectedTags_.size(); j++){
          for (int i = 0; i < tags_corners.size(); i++)
          {
            double confidence_index;
            
            if (detectedTags_[j].tag->id == tags_id[i])
            {
              for( unsigned int k = 0; k < tags_points3d[i].size(); k++ ) 
                tags_points3d[i][k].oP = detectedTags_[j].tag->oMt * tags_points3d[i][k].oP;
              
              if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam_, tags_points3d[i], cMo_, &confidence_index))
              {
                ROS_WARN_STREAM("tag:" + std::to_string(tags_id[i]) + ". Confidence: " + std::to_string(confidence_index));
              }
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
