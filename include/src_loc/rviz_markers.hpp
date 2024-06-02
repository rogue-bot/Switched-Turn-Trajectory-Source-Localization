#pragma once
// #ifndef INCLUDED_RVIZ_MARKERS
// #define INCLUDED_RVIZ_MARKERS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

namespace Rviz_Markers{
class rviz_markers{
    protected: 
        ros::Publisher line_marker_pub;
        ros::Publisher sine_marker_pub;
        ros::Publisher switch_marker_pub;
        ros::Publisher grad_Arrow_pub;
        ros::Publisher sine_Arrow_pub;
        ros::Publisher sine_path_Arrow_pub;
        ros::Publisher min_vec_Arrow_pub;
        ros::Publisher pos_Arrow_pub;
        ros::Subscriber mav_pose_sub; 
        ros::NodeHandle* marker_nh;
        geometry_msgs::PoseStamped mav_pose;
        float xp, yp, zp, psi;
        Eigen::Vector2f startPos, currentPos;
        bool isPosAvailable;
        geometry_msgs::Point currentPoint, switch_point;
        visualization_msgs::Marker line_strip;
        visualization_msgs::Marker sine_strip;
        visualization_msgs::Marker Switch_points;
        visualization_msgs::Marker grad_Arrow;
        visualization_msgs::Marker sine_Arrow;
        visualization_msgs::Marker sine_path_Arrow;
        visualization_msgs::Marker pos_Arrow;
        visualization_msgs::Marker min_vec_Arrow;
        int gradArrowID;

    public:
        rviz_markers();
        void update__marker_nh(ros::NodeHandle* _nh);

        void marker_mav_pose_cb(const geometry_msgs::PoseStamped& msg);
        void init_markers();
        void add_point_linestrip(geometry_msgs::Point Point);
        void add_point_sinestrip(geometry_msgs::Point Point);
        void add_point_switch(geometry_msgs::Point Point);
        void update_pos_Arrow();
        void update_grad_Arrow(double theta);
        void update_sine_Arrow(double theta, geometry_msgs::Point);
        void update_sine_path_Arrow(double theta, geometry_msgs::Point);
        void update_min_vec_Arrow(geometry_msgs::Point, geometry_msgs::Point);
        
};
}

// #endif