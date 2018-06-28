#include "tracker_object/node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_object");

    Node node{};

    node.spin();
    
    return 0;
}