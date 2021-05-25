#ifndef __TRACKER_OBJECT__FWD_HPP__ 
#define __TRACKER_OBJECT__FWD_HPP__

#include <memory>
#include <visp3/core/vpHomogeneousMatrix.h>

template <class T> class vpImage;
class vpRGBa;
class vpMbGenericTracker;
class vpDetectorAprilTag;

namespace agimus_vision {
namespace tracker_object {

enum State {
  state_detection,
  state_tracking,
};

typedef vpImage< unsigned char > GrayImage_t;
typedef vpImage< vpRGBa > RGBaImage_t;

// depth map image
typedef vpImage<u_int16_t> DepthMap_t;

struct DetectorAprilTagWrapper;
typedef std::shared_ptr<DetectorAprilTagWrapper> DetectorAprilTagWrapperPtr;

}
}

#endif // __TRACKER_OBJECT__FWD_HPP__ 
