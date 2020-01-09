//! \example tutorial-mb-generic-tracker-apriltag-webcam.cpp
#include <fstream>
#include <ios>
#include <iostream>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <agimus_vision/tracker_object/tracker.hpp>

using namespace agimus_vision::tracker_object;

initializationStep::AprilTag* createAprilTag (
    const std::string& config_file,
    const vpCameraParameters &cam)
{
  std::shared_ptr<DetectorAprilTagWrapper> detector
    (new DetectorAprilTagWrapper(vpDetectorAprilTag::TAG_36h11));

  if (!config_file.empty())
    initializationStep::AprilTag::configure(*detector, config_file);

  double tagSize = 0.06;
  initializationStep::AprilTag* aprilTag (new initializationStep::AprilTag(detector));
  aprilTag->cameraParameters (cam);
  aprilTag->addTag (20, tagSize,
      vpHomogeneousMatrix (std::vector<double>({
          1., 0., 0., -0.062875,
          0., 1., 0.,  0.008625,
          0., 0., 1.,  0.022   ,
          })));
  aprilTag->addTag (21, tagSize,
      vpHomogeneousMatrix (std::vector<double>({
            1., 0., 0., 0.062875,
            0., 1., 0., 0.008625,
            0., 0., 1., 0.022   ,
            0., 0., 0., 1.      ,
            })));
  aprilTag->addTag (22, tagSize,
      vpHomogeneousMatrix (std::vector<double>({
            -1., 0.,  0.,  0.062875,
             0., 1.,  0.,  0.008625,
             0., 0., -1., -0.022,
            })));
  aprilTag->addTag (23, tagSize,
      vpHomogeneousMatrix (std::vector<double>({
            -1., 0.,  0., -0.062875,
             0., 1.,  0.,  0.008625,
             0., 0., -1., -0.022,
            })));
  return aprilTag;
}

trackingStep::ModelBased* createModelBased (int type,
    const std::string& config_file,
    const std::string& model_file,
    const vpCameraParameters &cam,
    double projection_error_threshold)
{
  trackingStep::ModelBased* mb (new trackingStep::ModelBased (type,
        model_file, cam, projection_error_threshold, config_file));
  mb->tracker().setDisplayFeatures(true);
  return mb;
}

int main(int argc, const char **argv)
{
  int opt_device = 0;
  std::string opt_intrinsic_file = "";
  std::string opt_camera_name = "";
  std::string opt_config_file = "";
  std::string model_file = "";
  bool apriltag_tracking = false;
#ifdef VISP_HAVE_OPENCV
  bool opt_use_texture = false;
#endif
  double opt_projection_error_threshold = 40.;

#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
#else
  bool display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--model" && i + 1 < argc) {
      model_file = argv[i + 1];
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--config_file" && i + 1 < argc) {
      opt_config_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--track_apriltag") {
      apriltag_tracking = true;
    } else if (std::string(argv[i]) == "--display_off") {
      display_off = true;
#ifdef VISP_HAVE_OPENCV
    } else if (std::string(argv[i]) == "--texture") {
      opt_use_texture = true;
#endif
    } else if (std::string(argv[i]) == "--projection_error" && i + 1 < argc) {
      opt_projection_error_threshold = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0] << " [--input <camera id>]"
                                           " [--intrinsic <xml intrinsic file>] [--camera_name <camera name in xml file>]"
                                           " [--config_file <xml mbt config file>]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_off]";
#endif
      std::cout << " [--texture] [--projection_error <30 - 100>] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpCameraParameters cam;
  bool camIsInit = false;
#ifdef VISP_HAVE_PUGIXML
  vpXmlParserCamera parser;
  if (!opt_intrinsic_file.empty() && !opt_camera_name.empty()) {
    parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
    camIsInit = true;
  }
#endif

  try {
    vpImage<unsigned char> I;

    //! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use device " << device.str() << " (v4l2 grabber)" << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    std::cout << "Use device " << opt_device << " (OpenCV grabber)" << std::endl;
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    if (!camIsInit) {
      cam.initPersProjWithoutDistortion(600, 600, I.getWidth() / 2., I.getHeight() / 2.);
    }

    //std::cout << "Cube size: " << lx << ", " << ly << ", " << lz << std::endl;
    std::cout << "Model file: " << model_file << std::endl;
    std::cout << "Camera parameters:\n" << cam << std::endl;
    std::cout << "Tracker: " << std::endl;
    std::cout << "  Use edges  : 1"<< std::endl;
    std::cout << "  Use texture: "
#ifdef VISP_HAVE_OPENCV
              << opt_use_texture << std::endl;
#else
              << " na" << std::endl;
#endif
    std::cout << "  Projection error: " << opt_projection_error_threshold << std::endl;

    // Construct display
    vpDisplay *d = NULL;
    if (!display_off) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
#elif defined(VISP_HAVE_OPENCV)
      d = new vpDisplayOpenCV(I);
#endif
    }

    // Initialize AprilTag detector
    std::shared_ptr<initializationStep::AprilTag> initStep (createAprilTag (
        opt_config_file,
        cam));
    std::shared_ptr<trackingStep::ModelBased> trackStep (createModelBased (
#ifdef VISP_HAVE_OPENCV
          (opt_use_texture ? vpMbGenericTracker::KLT_TRACKER : 0) |
#endif
          vpMbGenericTracker::EDGE_TRACKER,
          opt_config_file,
          model_file,
          cam,
          opt_projection_error_threshold));
    Tracker algo (initStep.get(),
        (apriltag_tracking ? (TrackingStep*)initStep.get() : trackStep.get())
        );

    //Tracker algo (&initStep, &initStep);

    // wait for a tag detection
    bool quit = false;
    while (!quit) {

#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      cap >> frame;
      vpImageConvert::convert(frame, I);
#endif

      vpDisplay::display(I);

      algo.process (I);

      algo.drawDebug (I);

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) { // exit
        quit = true;
      }

      vpDisplay::flush(I);
    }

    if (!display_off)
      delete d;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
}
