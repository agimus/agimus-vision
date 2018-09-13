#include "agimus_vision/extrinsic_calibration/node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extrinsic_calibration");

    Node node{};

    node.spin();
    
    return 0;
}
