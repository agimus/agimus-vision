#include <agimus_vision/tracker_object/tracker.hpp>

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayOpenCV.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace to = agimus_vision::tracker_object;

vpCameraParameters makeTiagoCameraParameters()
{
  struct {
    int height = 480;
    int width = 640;
    std::string distortion_model = "plumb_bob";
    std::vector<double> K = { 536.8053440553133, 0.0, 310.4497775034432, 0.0, 538.1881626552579, 241.0458362306309, 0.0, 0.0, 1.0 };
    std::vector<double> P = { 536.246337890625, 0.0, 307.7145602612291, 0.0, 0.0, 539.6106567382812, 241.4628501786956, 0.0, 0.0, 0.0, 1.0, 0.0};
  } cam_info;


  vpCameraParameters cam;
  // Check that the camera is calibrated, as specified in the
  // sensor_msgs/CameraInfo message documentation.
  if (cam_info.K.size() != 3 * 3 || cam_info.K[0] == 0.)
    throw std::runtime_error ("uncalibrated camera");

  // Check matrix size.
  if (cam_info.P.size() != 3 * 4)
    throw std::runtime_error
      ("camera calibration P matrix has an incorrect size");

  if (cam_info.distortion_model.empty ())
    {
      const double& px = cam_info.K[0 * 3 + 0];
      const double& py = cam_info.K[1 * 3 + 1];
      const double& u0 = cam_info.K[0 * 3 + 2];
      const double& v0 = cam_info.K[1 * 3 + 2];
      cam.initPersProjWithoutDistortion(px, py, u0, v0);
      return cam;
    }

  if (cam_info.distortion_model == "plumb_bob")
    {
      const double& px = cam_info.P[0 * 4 + 0];
      const double& py = cam_info.P[1 * 4 + 1];
      const double& u0 = cam_info.P[0 * 4 + 2];
      const double& v0 = cam_info.P[1 * 4 + 2];
      cam.initPersProjWithoutDistortion(px, py, u0, v0);
      //cam.initPersProjWithDistortion(px, py, u0, v0, -cam_info.D[0], cam_info.D[0]);
      return cam;
    }

  throw std::runtime_error ("unsupported distortion model");

 // return vpCameraParameters(cam_info.P[0 * 4 + 0],cam_info.P[1 * 4 + 1],cam_info.P[0 * 4 + 2],cam_info.P[1 * 4 + 2],-cam_info.D[0],cam_info.D[0]);
}

to::initializationStep::AprilTag makeAprilTag()
{
  static std::shared_ptr<to::DetectorAprilTagWrapper> aprilTagDetector;
  if (!aprilTagDetector) {
    aprilTagDetector = std::make_shared<to::DetectorAprilTagWrapper>(vpDetectorAprilTag::TAG_36h11);

    aprilTagDetector->detector.setAprilTagPoseEstimationMethod(
        vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS );
    aprilTagDetector->detector.setAprilTagNbThreads(4);
    aprilTagDetector->detector.setAprilTagQuadDecimate(1.);
  }

  return to::initializationStep::AprilTag (aprilTagDetector);
}

std::array<std::array<double, 3>, 4> aprilTagPoints (double size)
{
  std::array< vpPoint, 4 > tPs = to::DetectorAprilTag::compute3DPoints(size);
  std::array<std::array<double, 3>, 4> pts;
  for (int i = 0; i < 4; ++i)
    pts[i] = { tPs[i].oP[0], tPs[i].oP[1], tPs[i].oP[2], };
  return pts;
}

PYBIND11_PLUGIN(py_agimus_vision) {
    py::module m("py_agimus_vision", "");

    py::class_<vpColVector>(m, "ColVector", py::buffer_protocol())
      .def(py::init<>())
      .def("__init__", [](vpColVector &m, py::array_t<double> b) {
            vpColVector _m((unsigned)b.size());
            for (unsigned i = 0; i < b.size(); ++i)
              _m[i] = *b.data(i);
            new (&m) vpColVector(std::move(_m));
          })
      .def_buffer([](vpColVector &m) -> py::buffer_info {
        return py::buffer_info(
            m.data,                                 /* Pointer to buffer */
            sizeof(double),                         /* Size of one scalar */
            py::format_descriptor<double>::format(),/* Python struct-style format descriptor */
            1,                                      /* Number of dimensions */
            { m.size() },                           /* Buffer dimensions */
            { sizeof(double) }                      /* Strides (in bytes) for each index */
        );
    });

    py::class_<vpPoint>(m, "Point")
      .def(py::init<>())
      //.def_readwrite("oP", &vpPoint::oP)
      //.def_readwrite("p", &vpPoint::p)
      //.def_readwrite("cP", &vpPoint::cP)
      .def_property("oP", [](vpPoint& pt) { return pt.oP.toStdVector(); }, [](vpPoint& pt, std::vector<double> oP) { return pt.oP = oP; })
      .def_property("cP", [](vpPoint& pt) { return pt.cP.toStdVector(); }, [](vpPoint& pt, std::vector<double> cP) { return pt.cP = cP; })
      .def_property("p" , [](vpPoint& pt) { return pt. p.toStdVector(); }, [](vpPoint& pt, std::vector<double>  p) { return pt. p =  p; })
      .def("project", (void (vpPoint::*)())&vpPoint::project)
      .def("project", (void (vpPoint::*)(const vpHomogeneousMatrix&))&vpPoint::project)
      .def("getWorldCoordinates", [](vpPoint& p) { return p.getWorldCoordinates().toStdVector(); })
      .def("setWorldCoordinates", (void (vpPoint::*)(double oX, double oY, double oZ))&vpPoint::setWorldCoordinates)
      ;

    py::class_<to::GrayImage_t>(m, "Image")
      .def(py::init<>())
      .def("read", [](to::GrayImage_t& I, const std::string& filename) { vpImageIo::read(I, filename); })
      .def("display", [](to::GrayImage_t& I) { vpDisplay::display(I); })
      .def("flush", [](to::GrayImage_t& I) { vpDisplay::flush(I); })
      .def("getClick", [](to::GrayImage_t& I) { vpDisplay::getClick(I); })
      .def("initDisplay", [](to::GrayImage_t& I) { static vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO); })
      .def("displayPoint", [](to::GrayImage_t& I, double u, double v, int thickness) { vpDisplay::displayPoint(I, (int)v, (int)u, vpColor::red, thickness); })
          /* TODO
             .def("displayFrame", [](to::GrayImage_t& I, vpHomogeneousMatrix M,) { vpDisplay::display(I); }
             vpDisplay::displayFrame(I, cMo, cam, 0.5, vpColor::none, 2 );
             */
      ;

    py::class_<vpCameraParameters>(m, "CameraParameters")
      .def("convertMeterPixel", [](const vpCameraParameters& cam, double x, double y) {
          double u, v;
          vpMeterPixelConversion::convertPointWithoutDistortion (cam, x, y, u, v);
          return py::make_tuple(u,v);
          })
    ;

    m.def ("makeTiagoCameraParameters", &makeTiagoCameraParameters);

    py::class_<vpHomogeneousMatrix>(m, "HomogeneousMatrix", py::buffer_protocol())
      .def(py::init<>())
      .def("__init__", [](vpHomogeneousMatrix &m, py::array_t<double> b) {
            vpHomogeneousMatrix _m;
            for (int i = 0; i < 4; ++i)
              for (int j = 0; j < 4; ++j)
                _m[i][j] = *b.data(i,j);
            new (&m) vpHomogeneousMatrix(_m);
          })
      .def_buffer([](vpHomogeneousMatrix &m) -> py::buffer_info {
        return py::buffer_info(
            m.data,                                 /* Pointer to buffer */
            sizeof(double),                         /* Size of one scalar */
            py::format_descriptor<double>::format(),/* Python struct-style format descriptor */
            2,                                      /* Number of dimensions */
            { 4, 4 },                               /* Buffer dimensions */
            { sizeof(double) * 4,                   /* Strides (in bytes) for each index */
              sizeof(double) }
        );
    });

    py::class_<to::DetectorAprilTagWrapper, std::shared_ptr<to::DetectorAprilTagWrapper> >(m, "DetectorAprilTagWrapper")
      .def("getTagsId", [](const to::DetectorAprilTagWrapper& atw) { return atw.detector.getTagsId(); })
      .def_readwrite("imageReady", &to::DetectorAprilTagWrapper::imageReady)
      ;

    py::class_<to::initializationStep::AprilTag>(m, "AprilTag")
      .def("detector", &to::initializationStep::AprilTag::detector)
      .def("cameraParameters", &to::initializationStep::AprilTag::cameraParameters)
      .def("addTag", &to::initializationStep::AprilTag::addTag)
      .def("detect", [](to::initializationStep::AprilTag& at, const to::GrayImage_t& I) { return at.detect(I) == to::state_tracking; })
      .def("getPose", [](const to::initializationStep::AprilTag& at) { vpHomogeneousMatrix M; at.getPose(M); return M;} )
      .def("getPoints", [](to::initializationStep::AprilTag& at, const vpCameraParameters& cam, int id) {
            std::size_t k = 0;
            std::vector<int> tagIds = at.detector()->detector.getTagsId();
            for (k = 0; k < tagIds.size(); ++k)
              if (tagIds[k] == id) break;
            if (k == tagIds.size())
              throw std::invalid_argument("Tag not detected");

            std::vector<vpImagePoint> imagePoints = at.detector()->detector.getPolygon(k);
            std::array<std::array<double,2>, 4 > pts;

            assert(imagePoints.size() == 4);
            for( unsigned int i = 0; i < 4; i++ ) {
              vpPixelMeterConversion::convertPointWithoutDistortion (cam, imagePoints[i],
                  pts[i][0], pts[i][1]);
            }
            return pts;
          })
      .def("drawDebug", &to::initializationStep::AprilTag::drawDebug)
      ;
    m.def ("makeAprilTag", &makeAprilTag);
    m.def ("aprilTagPoints", &aprilTagPoints);

    return m.ptr();
}
