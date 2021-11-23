#ifndef __TRACKER_OBJECT__TRACKER_HPP__
#define __TRACKER_OBJECT__TRACKER_HPP__

#include <agimus_vision/tracker_object/fwd.hpp>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>

#include <agimus_vision/tracker_object/detector_apriltag.hpp>
#include <agimus_vision/TrackerConfig.h>

namespace agimus_vision {
namespace tracker_object {

class Reconfigurable
{
public:
  virtual void reconfigure(TrackerConfig& /*config*/, uint32_t /*level*/) {}
};

/// Base class for object detection.
class InitializationStep
{
public:
  /// Detect if the image contains the object.
  virtual State detect(const GrayImage_t& I) = 0;

  /// Get an estimate of the object pose.
  virtual void getPose (vpHomogeneousMatrix& cMo) const = 0;

  /// Draw debugging information. A display must have been setup beforehand.
  virtual void drawDebug( GrayImage_t& I ) { (void)I; }
};

/// Base class for object tracking.
class TrackingStep
{
public:
  /// Initialize tracking of an object.
  /// \param I the image onto which the object was detected.
  /// \param cMo the estimated object pose.
  virtual void init(const GrayImage_t& I, const vpHomogeneousMatrix& cMo) = 0;

  /// Track the object.
  /// \param I a new image.
  virtual State track(const GrayImage_t& I, const DepthMap_t& D, float depthScale) = 0;

  /// Get the object pose
  virtual void getPose (vpHomogeneousMatrix& cMo) const = 0;

  /// \copydoc InitializationStep::drawDebug(GrayImage_t&)
  virtual void drawDebug(GrayImage_t& I ) { (void)I; }
};

class FilteringStep : public Reconfigurable
{
public:
  virtual void reset() { lastT_ = -1; }
  virtual void filter(const vpHomogeneousMatrix& M, const double time) = 0;

  inline void getPose (vpHomogeneousMatrix& cMo) const { cMo = M_; }

protected:
  double lastT_;
  vpHomogeneousMatrix M_;
};

/// Object tracking algorithm.
/// It contains a InitializationStep object and a TrackingStep object.
class Tracker : public Reconfigurable
{
  public:
    Tracker () : state_ (state_detection),
      detectionSubsampling_(1), n_(0)
    {}

    Tracker(std::shared_ptr<InitializationStep> init, std::shared_ptr<TrackingStep> track,
        const std::string& name = "",
        int subsampling = 1)
      : initialization_ (init),
      tracking_ (track),
      state_ (state_detection),
      detectionSubsampling_(subsampling), n_(0),
      name_ (name)
    {}

    inline void filtering(std::shared_ptr<FilteringStep> filter)
    {
      filtering_ = filter;
    }

    /// Process an image.
    /// If the object was not detected in the previous image, use the
    /// initialization step to detect it.
    /// If the object was detected in the previous image, use the
    /// tracking step.
    void process (const GrayImage_t& I, const vpImage<uint16_t>& D, 
                  const double time, float depthScale);

    /// Whether an object pose could be computed.
    bool hasPose () const
    {
      return state_ == state_tracking;
    }

    /// Get the object pose, if any. This method does nothing if \c hasPose()
    /// return \c false.
    inline void getPose (vpHomogeneousMatrix& cMo) const
    {
      if (state_ == state_tracking) {
        if (filtering_)
          filtering_->getPose(cMo);
        else
          tracking_->getPose(cMo);
      }
    }

    /// \copydoc InitializationStep::drawDebug(GrayImage_t&)
    void drawDebug( GrayImage_t& I );

    /// Set the object name
    void name (const std::string& name)
    {
      name_ = name;
    }

    /// Get the object name
    const std::string& name () const
    {
      return name_;
    }

    void detectionSubsampling (int n)
    {
      detectionSubsampling_ = n;
    }

    void reconfigure(TrackerConfig& config, uint32_t level)
    {
      if (filtering_ /*&& filter using level */) filtering_->reconfigure(config, level);
    }

  private:
    std::shared_ptr<InitializationStep> initialization_;
    std::shared_ptr<TrackingStep> tracking_;
    std::shared_ptr<FilteringStep> filtering_;

    State state_;
    int detectionSubsampling_;
    int n_;
    std::string name_;
};

namespace initializationStep {

/// Detect and track AprilTags.
class AprilTag : public InitializationStep, public TrackingStep
{
  public:
    static void configure(vpDetectorAprilTag& detector,
        const std::string& configFile);

    AprilTag (std::shared_ptr<DetectorAprilTagWrapper> detector)
      : detector_ (detector)
    {}

    /// Detect one of the provided AprilTag.
    State detect(const GrayImage_t& I);

    /// Initialize tracking of a set of AprilTag.
    /// \param I unused
    void init(const GrayImage_t &I, const vpHomogeneousMatrix& cMo);

    /// tracks previously seen AprilTags.
    /// \todo Three improvements:
    ///       - Very effective: Add ability to use a region of interest
    ///         around the tracked tag.
    ///       - Easy: Use only VIRTUAL_VS to estimate
    ///         the pose. Take example of what is done in DetectorAprilTag.
    ///       - Easy: Use a ModelBased tracker that contains only the edges of
    ///         tag. This should be done outside of this class.
    State track(const GrayImage_t &I, const DepthMap_t &D, float depthScale);

    void getPose (vpHomogeneousMatrix& cMo) const
    {
      cMo = cMo_;
    }

    void drawDebug( GrayImage_t& I );

    /// \return true if the tag was added and false if it already existed.
    bool addTag (int id, double size, vpHomogeneousMatrix oMt);

    void cameraParameters (const vpCameraParameters& cam)
    {
      cam_ = cam;
    }
    void depthCameraParameters (const vpCameraParameters& cam)
    {
      depth_cam_ = cam;
    }

    std::shared_ptr<DetectorAprilTagWrapper> detector()
    {
      return detector_;
    }


  private:
    /// Fill the member \c detectedTags_
    bool detectTags(const GrayImage_t& I);

    struct Tag {
      vpHomogeneousMatrix oMt;
      double size;
      int id;
      std::string message;
    };

    std::shared_ptr<DetectorAprilTagWrapper> detector_;
    vpCameraParameters cam_;
    vpCameraParameters depth_cam_;
    std::vector<Tag> tags_;

    struct DetectedTag {
      std::size_t i;
      Tag* tag;
    };
    std::vector<DetectedTag> detectedTags_;
    std::map<int, double> tags_size;
    vpHomogeneousMatrix cMo_;
};

}

namespace trackingStep {

typedef initializationStep::AprilTag AprilTag;

/// Track an object based on:
/// - the edges, based on a model provided by the user,
/// - and optionally the KLT features that are detected online.
///
/// It uses the ViSP vpMbGenericTracker class.
class ModelBased : public TrackingStep
{
  public:
    ModelBased(int trackerType,
        const std::string& modelFile,
        const vpCameraParameters &cam,
        double projectionErrorThreshold,
        const std::string& configFile = "");

    ModelBased () :
      tracker_ (),
      projErrorThr_ (40.)
    {}

    void init(const vpImage< unsigned char > &gray_image,
        const vpHomogeneousMatrix& cMo);

    State track(const vpImage< unsigned char > &gray_image);

    void getPose (vpHomogeneousMatrix& cMo) const;

    void drawDebug( GrayImage_t &I );

    void projectionErrorThreshold (double thr)
    {
      projErrorThr_ = thr;
    }

    vpMbGenericTracker& tracker()
    {
      return tracker_;
    }

    const vpMbGenericTracker& tracker() const
    {
      return tracker_;
    }

  private:
    ModelBased(const ModelBased&) {}

    vpMbGenericTracker tracker_;
    double projErrorThr_;
};

}

namespace filteringStep {
class PositionLowPassFirstOrder : public FilteringStep
{
public:
  void filter(const vpHomogeneousMatrix& M, const double time);

  PositionLowPassFirstOrder(double cutFrequency) : f_ (cutFrequency) {}

  void reconfigure(TrackerConfig& config, uint32_t level);

private:
  double f_;
  vpColVector vel_;
};

class PositionLowPassOrder : public FilteringStep
{
public:
  void reset();

  void filter(const vpHomogeneousMatrix& M, const double time);

  PositionLowPassOrder(double cutFrequency, int order) : filters_(order, cutFrequency) {}

  void reconfigure(TrackerConfig& config, uint32_t level);

private:
  std::vector<PositionLowPassFirstOrder> filters_;
};

}

}
}

#endif // __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__
