#include "premaidai_teleoperation/interactive_marker.hh"
#include <boost/bind.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace premaidai_teleoperation;

// anonymous namespace
namespace
{
    geometry_msgs::Pose
    TransformToPose(const geometry_msgs::TransformStamped& transform)
    {
        geometry_msgs::Pose ret;
        ret.position.x = transform.transform.translation.x;
        ret.position.y = transform.transform.translation.y;
        ret.position.z = transform.transform.translation.z;
        ret.orientation = transform.transform.rotation;
        return ret;
    }
}


InteractiveMarker::InteractiveMarker(const std::string& name)
    : frame_id_("/base_link")
    , base_link_("/base_link")
    , nh_("~")
    , name_(name)
    , reset_marker_pos_srv_()
    , broadcaster_()
    , tf_buffer_()
    , listener_(tf_buffer_)
{
    // get parameters
    if (!nh_.getParam("frame_id", frame_id_))
    {
        ROS_INFO("frame id set to default [/base_link]");
        frame_id_ = "/frame_id";
    }

    if (!nh_.getParam("initial_frame", initial_frame_))
    {
        ROS_INFO("initial frame set to default [/r_hand_ee]");
        initial_frame_ = "/r_hand_ee";
    }

    // get parameters
    if (!nh_.getParam("base_link", base_link_))
    {
        ROS_INFO("base_link set to default [/base_link]");
        base_link_ = "/base_link";
    }

    // initialize current pose
    current_pose_.orientation.x = 0.0;
    current_pose_.orientation.y = 0.0;
    current_pose_.orientation.z = 0.0;
    current_pose_.orientation.w = 1.0;

    // create service
    reset_marker_pos_srv_ = nh_.advertiseService("reset_marker_pose", &InteractiveMarker::resetMarkerPose, this);

    // register timer callback
    tf_timer_ = nh_.createTimer(ros::Duration(0.5), &InteractiveMarker::timerCallback, this);

    // get initial pose
    initial_trans_ = getInitialTransform(base_link_, initial_frame_);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = base_link_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = name_;
    int_marker.description = "EE pose";
    int_marker.scale = 0.1;
    int_marker.pose = TransformToPose(initial_trans_);

    // get marker information
    visualization_msgs::Marker marker;
    std::string mesh_resource;
    if (nh_.getParam("mesh_resource", mesh_resource))
    {
        ROS_INFO("Use mesh resource marker [%s]", mesh_resource.c_str());
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = mesh_resource;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.4;
    }
    else
    {
        ROS_INFO("Use cube marker");
        marker.type = visualization_msgs::Marker::CUBE;
        marker.mesh_resource = mesh_resource;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.4;
    }

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    visualization_msgs::InteractiveMarkerControl control;

    // x axis
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // y axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // z axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_ = InteractiveMarkerServerPtr(new interactive_markers::InteractiveMarkerServer(name_));
    server_->insert(int_marker, boost::bind(&InteractiveMarker::processFeedback, this, _1));
    server_->applyChanges();
}


// virtual
InteractiveMarker::~InteractiveMarker()
{}


void
InteractiveMarker::timerCallback(const ros::TimerEvent& event)
{
    geometry_msgs::Pose pose;
    tf2::doTransform(current_pose_, pose, initial_trans_);
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = base_link_;
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    broadcaster_.sendTransform(transform);
}


void
InteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    current_pose_ = feedback->pose;
}


bool
InteractiveMarker::resetMarkerPose(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Reset marker pose [%s]", initial_frame_.c_str());
    initial_trans_ = getInitialTransform(base_link_, initial_frame_);

    geometry_msgs::Pose pose = TransformToPose(getInitialTransform(base_link_, initial_frame_));
    server_->setPose(name_, pose);
    server_->applyChanges();
    return true;
}


geometry_msgs::TransformStamped
InteractiveMarker::getInitialTransform(const std::string& base_link, const std::string& target_frame)
{
    geometry_msgs::TransformStamped transform;

    try
    {
        if (tf_buffer_.canTransform(base_link, target_frame, ros::Time(0), ros::Duration(10)))
        {
            transform = tf_buffer_.lookupTransform(base_link, target_frame, ros::Time(0));
        }
        else
        {
            ROS_ERROR("Fail to convert tf frame [%s] -> [%s]", base_link.c_str(), target_frame.c_str());
        }
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    return transform;
}
