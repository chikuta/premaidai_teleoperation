#include "premaidai_teleoperation/interactive_marker.hh"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "interactive_marker");
    premaidai_teleoperation::InteractiveMarker marker(ros::this_node::getName());
    ros::spin();
    return 0;
}
