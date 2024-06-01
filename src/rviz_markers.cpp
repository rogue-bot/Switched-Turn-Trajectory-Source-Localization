#include "src_loc/rviz_markers.hpp"
#include <tf/transform_datatypes.h>

namespace Rviz_Markers{

rviz_markers::rviz_markers()
    :isPosAvailable(false), gradArrowID(100){}

void rviz_markers::update__marker_nh(ros::NodeHandle* _nh){
    marker_nh = _nh;
    isPosAvailable = false;
    gradArrowID = 100;
}

void rviz_markers::marker_mav_pose_cb(const geometry_msgs::PoseStamped& msg){
    mav_pose = msg;
    // get pose of the mav convert it into roll pitch and yaw.
    float linearposx=msg.pose.position.x;
    float linearposy=msg.pose.position.y;
    float linearposz=msg.pose.position.z;
    xp = linearposx;
    yp = linearposy;
    zp = linearposz;
    currentPos[0] = xp;
    currentPos[1] = yp;
    currentPoint.x = xp;
    currentPoint.y = yp;
    currentPoint.z = zp;
    // ROS_INFO("x %f, y %f", currentPos[0], currentPos[1]);
    // std::cout << "Pos cb : " << xp << " " << yp << "\n";
    double quatx= msg.pose.orientation.x;
    double quaty= msg.pose.orientation.y;
    double quatz= msg.pose.orientation.z;
    double quatw= msg.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // update current yaw and Position of the mav
    psi = yaw; // std::fmod((yaw + 2*M_PI), (2*M_PI));
    if(!isPosAvailable) startPos = currentPos;
    isPosAvailable = true;
}

void rviz_markers::init_markers(){
    sine_marker_pub = marker_nh->advertise<visualization_msgs::Marker>("sine_marker", 100);
    line_marker_pub = marker_nh->advertise<visualization_msgs::Marker>("trej_marker", 10);
    switch_marker_pub = marker_nh->advertise<visualization_msgs::Marker>("Switch_marker", 10);
    grad_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("GradArrow",0);
    sine_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("SineArrow",1 );
    sine_path_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("SinePatHArrow",1 );
    pos_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("PosArrow", 10);
    min_vec_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("min_vec_Arrow", 1);
    mav_pose_sub = marker_nh->subscribe("/mavros/local_position/pose", 1, &rviz_markers::marker_mav_pose_cb, this);
}

void rviz_markers::add_point_sinestrip(geometry_msgs::Point Point){
    sine_strip.header.frame_id = "my_frame";
    sine_strip.ns = "marker_ns";
    sine_strip.action = visualization_msgs::Marker::ADD;
    sine_strip.pose.orientation.w = 0.0; 
    sine_strip.id = 2023;
    sine_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sine_strip.scale.x = 0.5;
    sine_strip.scale.x = 0.5;
    sine_strip.scale.x = 0.5;
    sine_strip.color.r = rand();
    sine_strip.color.g = 0.0;
    sine_strip.color.b = 1.0;
    sine_strip.color.a = 1.0;            
    // std::cout << "point : " << Point.x << " " << Point.y << "\n";
    sine_strip.header.stamp = ros::Time::now(); 
    sine_strip.points.push_back(Point);
    sine_marker_pub.publish(sine_strip);
}

void rviz_markers::add_point_linestrip(geometry_msgs::Point Point){
    line_strip.header.frame_id = "my_frame";
    line_strip.ns = "marker_ns";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0; 
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.5;
    line_strip.scale.x = 0.5;
    line_strip.scale.x = 0.5;
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;            
    // std::cout << "point line : " << Point.x << " " << Point.y << "\n";
    line_strip.header.stamp = ros::Time::now(); 
    line_strip.points.push_back(Point);
    line_marker_pub.publish(line_strip);
}

void rviz_markers::add_point_switch(geometry_msgs::Point Point){
    Switch_points.header.frame_id = "my_frame";
    Switch_points.ns = "marker_ns";
    Switch_points.action = visualization_msgs::Marker::ADD;
    Switch_points.pose.orientation.w = 1.0; 
    Switch_points.id = 2;
    Switch_points.type = visualization_msgs::Marker::POINTS;
    Switch_points.scale.x = 0.5;
    Switch_points.scale.y = 0.5;
    Switch_points.color.r = 1.0;
    Switch_points.color.g = 1.0;
    Switch_points.color.b = 0.0;
    Switch_points.color.a = 1.0;            
    // std::cout << "point : " << Point.x << " " << Point.y << "\n";
    Switch_points.header.stamp = ros::Time::now(); 
    Switch_points.points.push_back(Point);
    switch_marker_pub.publish(Switch_points);
}
void rviz_markers::update_pos_Arrow(){
    pos_Arrow.header.frame_id = "my_frame";
    pos_Arrow.ns = "marker_ns";
    pos_Arrow.action = visualization_msgs::Marker::ADD;
    pos_Arrow.id = 5;
    pos_Arrow.type = visualization_msgs::Marker::ARROW;
    pos_Arrow.scale.x = 1;
    pos_Arrow.scale.y = 1;
    pos_Arrow.scale.z = 1;
    
    pos_Arrow.points[0] = mav_pose.pose.position;
    pos_Arrow.points[1] = switch_point;
    pos_Arrow.color.r = 1.0;
    pos_Arrow.color.g = 0.0;
    pos_Arrow.color.b = 0.0;
    pos_Arrow.color.a = 1.0;            
    pos_Arrow.header.stamp = ros::Time::now(); 
    pos_Arrow_pub.publish(pos_Arrow);
}

void rviz_markers::update_grad_Arrow(double theta){
    grad_Arrow.header.frame_id = "my_frame";
    grad_Arrow.ns = "marker_ns";
    grad_Arrow.action = visualization_msgs::Marker::ADD;
    grad_Arrow.id = gradArrowID++;
    grad_Arrow.type = visualization_msgs::Marker::ARROW;
    grad_Arrow.scale.x = 5;
    grad_Arrow.scale.y = 0.4;
    grad_Arrow.scale.z = 0.2;
    
    // Vector2f gradVector_norm = gradVector/gradVector.norm();
    grad_Arrow.pose.position = mav_pose.pose.position;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    grad_Arrow.pose.orientation.x = q[0];
    grad_Arrow.pose.orientation.y = q[1];
    grad_Arrow.pose.orientation.z = q[2];
    grad_Arrow.pose.orientation.w = q[3];
    grad_Arrow.color.r = 0.5;
    grad_Arrow.color.g = 0.0;
    grad_Arrow.color.b = 1.0;
    grad_Arrow.color.a = 1.0;            
    grad_Arrow.header.stamp = ros::Time::now(); 
    grad_Arrow_pub.publish(grad_Arrow);
}

void rviz_markers::update_sine_Arrow(double theta, geometry_msgs::Point sine_point){
    sine_Arrow.header.frame_id = "my_frame";
    sine_Arrow.ns = "marker_ns";
    sine_Arrow.action = visualization_msgs::Marker::DELETEALL;
    sine_Arrow_pub.publish(sine_Arrow);

    sine_Arrow.action = visualization_msgs::Marker::MODIFY;
    sine_Arrow.id =  200 + gradArrowID++;
    sine_Arrow.type = visualization_msgs::Marker::ARROW;
    sine_Arrow.scale.x = 5;
    sine_Arrow.scale.y = 1;
    sine_Arrow.scale.z = 1;
    
    // Vector2f gradVector_norm = gradVector/gradVector.norm();
    sine_Arrow.pose.position = sine_point;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    sine_Arrow.pose.orientation.x = q[0];
    sine_Arrow.pose.orientation.y = q[1];
    sine_Arrow.pose.orientation.z = q[2];
    sine_Arrow.pose.orientation.w = q[3];
    sine_Arrow.color.r = 0.5;
    sine_Arrow.color.g = 0.0;
    sine_Arrow.color.b = 1.0;
    sine_Arrow.color.a = 1.0;            
    sine_Arrow.header.stamp = ros::Time::now(); 
    sine_Arrow_pub.publish(sine_Arrow);
}
void rviz_markers::update_sine_path_Arrow(double theta, geometry_msgs::Point sine_point){
    sine_path_Arrow.header.frame_id = "my_frame";
    sine_path_Arrow.ns = "marker_ns";

    sine_path_Arrow.action = visualization_msgs::Marker::MODIFY;
    sine_path_Arrow.id =  800 + gradArrowID++;
    sine_path_Arrow.type = visualization_msgs::Marker::ARROW;
    sine_path_Arrow.scale.x = 5;
    sine_path_Arrow.scale.y = 1;
    sine_path_Arrow.scale.z = 1;
    
    // Vector2f gradVector_norm = gradVector/gradVector.norm();
    sine_path_Arrow.pose.position = sine_point;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    sine_path_Arrow.pose.orientation.x = q[0];
    sine_path_Arrow.pose.orientation.y = q[1];
    sine_path_Arrow.pose.orientation.z = q[2];
    sine_path_Arrow.pose.orientation.w = q[3];
    sine_path_Arrow.color.r = rand();
    sine_path_Arrow.color.g = rand();
    sine_path_Arrow.color.b = rand();
    sine_path_Arrow.color.a = 1.0;            
    sine_path_Arrow.header.stamp = ros::Time::now(); 
    sine_path_Arrow_pub.publish(sine_path_Arrow);
}

void rviz_markers::update_min_vec_Arrow(geometry_msgs::Point waypoint, geometry_msgs::Point uav_pos){
    

    min_vec_Arrow.header.frame_id = "my_frame";
    min_vec_Arrow.ns = "marker_ns";
    min_vec_Arrow.action = visualization_msgs::Marker::DELETEALL;
    min_vec_Arrow_pub.publish(min_vec_Arrow);

    min_vec_Arrow.action = visualization_msgs::Marker::MODIFY;
    min_vec_Arrow.id =  20000 + gradArrowID++;
    min_vec_Arrow.type = visualization_msgs::Marker::ARROW;
    min_vec_Arrow.scale.x = 0.4;
    min_vec_Arrow.scale.y = 1;
    min_vec_Arrow.scale.z = 3;
    
    // Vector2f gradVector_norm = gradVector/gradVector.norm();
    // std::cout << "before assigning points" << "\n";
    min_vec_Arrow.points.clear();
    min_vec_Arrow.points.push_back(waypoint);
    min_vec_Arrow.points.push_back( uav_pos);
    min_vec_Arrow.color.r = 0.2;
    min_vec_Arrow.color.g = 0.5;
    min_vec_Arrow.color.b = 1.0;
    min_vec_Arrow.color.a = 1.0;            
    min_vec_Arrow.header.stamp = ros::Time::now(); 
    min_vec_Arrow_pub.publish(min_vec_Arrow);
}
}