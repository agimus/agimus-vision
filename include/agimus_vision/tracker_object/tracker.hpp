#ifndef __TRACKER_OBJECT__TRACKER_HPP__
#define __TRACKER_OBJECT__TRACKER_HPP__

#include <agimus_vision/tracker_object/fwd.hpp>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <agimus_vision/tracker_object/detector_apriltag.hpp>

namespace agimus_vision {
namespace tracker_object {

class InitializationStep
{
public:
  virtual State detect(const GrayImage_t& I) = 0;

  virtual void getPose (vpHomogeneousMatrix& cMo) const = 0;

  virtual void drawDebug( GrayImage_t& I ) { (void)I; }
};

class TrackingStep
{
public:
  virtual void init(const GrayImage_t& I, const vpHomogeneousMatrix& cMo) = 0;

  virtual State track(const GrayImage_t& I) = 0;

  virtual void getPose (vpHomogeneousMatrix& cMo) const = 0;

  virtual void drawDebug(GrayImage_t& I ) { (void)I; }
};

class Tracker
{
  public:
    Tracker () : state_ (state_detection)
    {}

    Tracker(std::shared_ptr<InitializationStep> init, std::shared_ptr<TrackingStep> track)
      : initialization_ (init),
      tracking_ (track),
      state_ (state_detection)
    {}

    void process (const GrayImage_t& I);

    bool hasPose () const
    {
      return state_ == state_tracking;
    }

    inline void getPose (vpHomogeneousMatrix& cMo) const
    {
      if (state_ == state_tracking) tracking_->getPose(cMo);
    }

    void drawDebug( GrayImage_t& I );

    void name (const std::string& name)
    {
      name_ = name;
    }

    const std::string& name () const
    {
      return name_;
    }

  private:
    std::shared_ptr<InitializationStep> initialization_;
    std::shared_ptr<TrackingStep> tracking_;

    State state_;
    std::string name_;
};

namespace initializationStep {

class AprilTag : public InitializationStep, public TrackingStep
{
  public:
    static void configure(vpDetectorAprilTag& detector,
        const std::string& configFile);

    AprilTag (std::shared_ptr<DetectorAprilTagWrapper> detector)
      : detector_ (detector),
      detectedTag_ (NULL)
    {}

    State detect(const GrayImage_t& I);

    void init(const GrayImage_t &I, const vpHomogeneousMatrix& cMo);

    State track(const GrayImage_t &I);

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

    std::shared_ptr<DetectorAprilTagWrapper> detector()
    {
      return detector_;
    }

  private:
    /// \param i index of tag in detector. It is valid only if the return value
    ///        is true.
    bool detectTags(const GrayImage_t& I, std::size_t& i);

    struct Tag {
      vpHomogeneousMatrix oMt;
      double size;
      int id;
      std::string message;
    };

    std::shared_ptr<DetectorAprilTagWrapper> detector_;
    vpCameraParameters cam_;

    std::vector<Tag> tags_;

    Tag* detectedTag_;
    vpHomogeneousMatrix cMo_;
};

}

namespace trackingStep {

typedef initializationStep::AprilTag AprilTag;

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

}
}

#endif // __TRACKER_OBJECT__DETECTOR_APRILTAG_HPP__
