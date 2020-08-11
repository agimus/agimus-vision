#include "agimus_vision/tracker_object/tracker.hpp"

#include <visp3/core/vpXmlParser.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpPose.h>

namespace agimus_vision {
namespace tracker_object {

void Tracker::process (const GrayImage_t& I, const double time)
{
  vpHomogeneousMatrix cMo;
  if (n_ > 0) --n_;
  if (state_ == state_detection) {
    if (n_ > 0) return;
    state_ = initialization_->detect(I);
    n_ = detectionSubsampling_;

    // Initialize the tracker with the result of the detection
    if (state_ == state_tracking) {
      initialization_->getPose (cMo);
      tracking_->init(I, cMo);
      if (filtering_) filtering_->reset();
    }
  }

  if (state_ == state_tracking) {
    state_ = tracking_->track(I);
    if (filtering_) {
      vpHomogeneousMatrix cMo;
      tracking_->getPose(cMo);
      filtering_->filter(cMo, time);
    }
  }
}

void Tracker::drawDebug( GrayImage_t& I )
{
  if (state_ == state_detection)
    initialization_->drawDebug(I);
  else if (state_ == state_tracking)
    tracking_->drawDebug(I);
}

namespace initializationStep {

class AprilTagParser : public vpXmlParser
{
  public:
    typedef enum{
      apriltag,
      nb_threads,
      quad_decimate,
      quad_sigma,
    } dataToParse;

    AprilTagParser ()
    {
      nodeMap["apriltag"] = apriltag;

      nodeMap["nb_threads"] = nb_threads;
      nodeMap["quad_decimate"] = quad_decimate;
      nodeMap["quad_sigma"] = quad_sigma;
    }

    void configure(vpDetectorAprilTag& detector)
    {
      std::cout << "*********** Parsing XML for april tag detection ************\n";

      if(setParameters.count(nb_threads)) {
        detector.setAprilTagNbThreads (nbThreads);
        std::cout << "apriltag : nb_threads : " << nbThreads << "\n";
      }
      if(setParameters.count(quad_decimate)) {
        detector.setAprilTagQuadDecimate (quadDecimate);
        std::cout << "apriltag : quad_decimate : " << quadDecimate << "\n";
      }
      if(setParameters.count(quad_sigma)) {
        detector.setAprilTagQuadSigma (quadSigma);
        std::cout << "apriltag : quad_sigma : " << quadSigma << "\n";
      }
      std::cout << std::flush;
    }

  private:
    void readMainClass (xmlDocPtr doc, xmlNodePtr node)
    {
      for (xmlNodePtr tmpNode = node->xmlChildrenNode; tmpNode != NULL; tmpNode = tmpNode->next) {
        if(tmpNode->type == XML_ELEMENT_NODE) {
          std::map<std::string, int>::iterator iter = this->nodeMap.find((const char*)tmpNode->name);
          if(iter == nodeMap.end()) {
            continue;
          }
          switch (iter->second){
            case apriltag:
              readAprilTag (doc, tmpNode);
              break;
            default:
              break;
          }
        }
      }
    }

    void writeMainClass (xmlNodePtr node)
    { (void) node; }

    void readAprilTag (xmlDocPtr doc, xmlNodePtr node)
    {
      for (xmlNodePtr child = node->children; child != NULL; child = child->next) {
        if(child->type == XML_ELEMENT_NODE) {
          std::map<std::string, int>::iterator iter = this->nodeMap.find((const char*)child->name);
          if(iter == nodeMap.end()) {
            continue;
          }
          switch (iter->second){
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

void AprilTag::configure(vpDetectorAprilTag& detector,
        const std::string& configFile)
{
  AprilTagParser parser;
  parser.parse(configFile);
  parser.configure(detector);
}

State AprilTag::detect(const GrayImage_t& I)
{
  // Check if the object is detected.
  std::size_t i;
  if (!detectTags(I, i))
    return state_detection;

  // Pose estimation
  vpHomogeneousMatrix cMt;
  try {
    if (detector_->detector.getPose(i, detectedTag_->size, cam_, cMt)) {
      cMo_ = cMt * detectedTag_->oMt.inverse();
      return state_tracking;
    }
  } catch (const vpException& e) {
    std::cerr << e.what() << std::endl;
  }
  return state_detection;
}

void AprilTag::init(const GrayImage_t &, const vpHomogeneousMatrix& cMo)
{
  cMo_ = cMo;
}

State AprilTag::track(const GrayImage_t &I)
{
  std::size_t i;
  if (!detectTags(I, i))
    return state_detection;

  // Pose estimation
  vpHomogeneousMatrix cMt = cMo_ * detectedTag_->oMt;

  std::vector<vpImagePoint> imagePoints = detector_->detector.getPolygon (i);
  std::array< vpPoint, 4 > points = DetectorAprilTag::compute3DPoints(detectedTag_->size);

  vpPose pose;
  // Compute the 2D coord. of the points (in meters) to match the 3D coord. for the pose estimation
  for( unsigned int i = 0; i < points.size(); i++ ) {
    double x{0.}, y{0.};
    vpPixelMeterConversion::convertPointWithoutDistortion (cam_, imagePoints[i], x, y);

    // x, y are the coordinates in the image plane, oX, oY, oZ in the world,
    // and X, Y, Z in the camera ref. frame
    points[i].set_x (x);
    points[i].set_y (y);
    pose.addPoint (points[i]);
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMt);
  cMo_ = cMt * detectedTag_->oMt.inverse();
  return state_tracking;
}

bool AprilTag::detectTags(const GrayImage_t& I, std::size_t& i)
{
  if (!detector_->detect (I))
    return false;

  // Check if the object is detected.
  detectedTag_ = NULL;
  bool ok = false;
  for (i = 0; i < detector_->detector.getNbObjects(); i++) {
    for (Tag& tag : tags_) {
      ok = (tag.message == detector_->detector.getMessage(i));
      if (ok) {
        detectedTag_ = &tag;
        break;
      }
    }
    // TODO the pose of each detected tag should be estimated. The one with the
    // smallest reprojection error should be kept.
    if (ok) return true;
  }
  return false;
}

void AprilTag::drawDebug(GrayImage_t& I)
{
  if(detectedTag_ == NULL) return;

  std::array< vpColor, 4 > colors{{ vpColor::red, vpColor::green, vpColor::blue, vpColor::cyan }};

  for (std::size_t k = 0; k < detector_->detector.getNbObjects(); k++) {
    if (detectedTag_->message == detector_->detector.getMessage(k)) {
      std::vector< vpImagePoint > & points (detector_->detector.getPolygon(k));
      for( unsigned int i{ 0 } ; i < 4 ; ++i )
        vpDisplay::displayLine(I, points[i], points[(i+1)%3], colors[i], 3);

      vpDisplay::displayFrame(I, cMo_, cam_, detectedTag_->size * 2, vpColor::none );
    }
  }

}

bool AprilTag::addTag (int id, double size, vpHomogeneousMatrix oMt)
{
  for (Tag tag : tags_)
    if (tag.id == id) return false;

  Tag tag;
  tag.id = id;
  tag.message = "36h11 id: " + std::to_string(id);
  tag.oMt = oMt;
  tag.size = size;
  tags_.push_back (tag);
  return true;
}

}

namespace trackingStep {

ModelBased::ModelBased(int trackerType,
    const std::string& modelFile,
    const vpCameraParameters &cam,
    double projectionErrorThr,
    const std::string& configFile)
{
  tracker().setTrackerType(trackerType);
  if (!configFile.empty())
#if VISP_VERSION_INT >= VP_VERSION_INT(3,2,1)
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
  projectionErrorThreshold (projectionErrorThr);
}

void ModelBased::init(const GrayImage_t &I,
    const vpHomogeneousMatrix& cMo)
{
  tracker_.initFromPose(I, cMo);
}

State ModelBased::track(const GrayImage_t &I)
{
  try{
    tracker_.track(I);
  } catch (const vpException& e) {
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

void ModelBased::getPose (vpHomogeneousMatrix& cMo) const
{
  tracker_.getPose(cMo);
}

void ModelBased::drawDebug( GrayImage_t &I )
{
  vpCameraParameters cam;
  tracker_.getCameraParameters(cam);

  // Display
  tracker_.display(I, tracker_.getPose(), cam, vpColor::red, 2);
  vpDisplay::displayFrame(I, tracker_.getPose(), cam, 0.025, vpColor::none, 3);
  vpDisplay::displayText(I, 40, 20, "State: tracking in progress", vpColor::red);
}

}

namespace filteringStep {
void VelocityLowPassFirstOrder::filter(const vpHomogeneousMatrix& M, const double time)
{
  if (lastT_ < 0) {
    lastT_ = time;
    M_ = M;
    vel_.resize(6, true);
    return;
  }
  double dt = time - lastT_;
  const double alpha = 1 / (1 + 1/(2*M_PI*f_*dt));

  vpColVector vel = vpExponentialMap::inverse(M_.inverse() * M, dt);

  vel_ = vel_ + alpha * (vel - vel_);
  M_ = M_ * vpExponentialMap::direct(vel_, dt);
}
void VelocityLowPassFirstOrder::reconfigure(TrackerConfig& config, uint32_t level)
{
  f_ = config.groups.filters.low_pass.cut_frequency;
}

void VelocityLowPassOrder::filter(const vpHomogeneousMatrix& M, const double time)
{
  M_ = M;
  for (VelocityLowPassFirstOrder& f : filters_) {
    f.filter(M_, time);
    f.getPose(M_);
  }
  lastT_ = time;
}

void VelocityLowPassOrder::reset()
{
  FilteringStep::reset();
  for (VelocityLowPassFirstOrder& f : filters_) f.reset();
}

void VelocityLowPassOrder::reconfigure(TrackerConfig& config, uint32_t level)
{
  for (VelocityLowPassFirstOrder& f : filters_) f.reconfigure(config, level);
}
}

}
}
