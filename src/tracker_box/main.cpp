#include "node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_box");

    Node node{};

    node.spin();
    
    return 0;
}
