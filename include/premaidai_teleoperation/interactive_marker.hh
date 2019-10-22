#ifndef _PREMAIDAI_INTERACTIVE_MARKER_HH_
#define _PREMAIDAI_INTERACTIVE_MARKER_HH_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <interactive_markers/interactive_marker_server.h>

namespace premaidai_teleoperation
{

class InteractiveMarker
{
public:
    explicit InteractiveMarker(const std::string& name);
    virtual ~InteractiveMarker();

protected:
    void timerCallback(const ros::TimerEvent& event);
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    bool resetMarkerPose(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    geometry_msgs::TransformStamped getInitialTransform(const std::string& base_link, const std::string& target_frame);

private:
    // typedef
    typedef std::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

    InteractiveMarkerServerPtr server_;         /** interactive marker server */
    std::string name_;                          /** interactive marker server name */
    std::string frame_id_;                      /** frame_id (default: /base_link) */
    std::string base_link_;                     /** base_link name (default: /base_link) */
    std::string initial_frame_;                 /** base_link name (default: /base_link) */
    ros::NodeHandle nh_;                        /** ros private node handler */
    ros::ServiceServer reset_marker_pos_srv_;   /** reset marker position service */
    ros::Timer tf_timer_;                       /** timer callback for tf (base_link_ -> frame_id_) */
    tf2_ros::Buffer tf_buffer_;                 /** tf buffer */
    tf2_ros::TransformListener listener_;       /** tf listener */
    tf2_ros::TransformBroadcaster broadcaster_; /** tf broadcaster */
    geometry_msgs::TransformStamped initial_trans_;
    geometry_msgs::Pose current_pose_;
};

};

#endif //_PREMAIDAI_INTERACTIVE_MARKER_HH_
