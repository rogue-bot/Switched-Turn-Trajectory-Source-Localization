#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float32.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <src_loc/DoMTSTurnSwitch.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Dense>

#define PI 3.1415
using namespace Eigen;

class TRM{
    private:
        mavros_msgs::State current_state;

        ros::Subscriber state_sub ;
        ros::Publisher local_pos_pub ;
        ros::Publisher cir_pos_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;

        ros::ServiceServer switch_turn;


        mavros_msgs::PositionTarget ptarget;
        mavros_msgs::PositionTarget sinTraj;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        
        
        ros::ServiceClient drone_pos_client; 

        gazebo_msgs::GetModelState droneState;

        geometry_msgs::Point actual_arva_pos;
        geometry_msgs::PoseStamped pose;

        ros::Publisher d_actual;
        std_msgs::Float32 _d_actual; 
        ros::Publisher h_approx;
        std_msgs::Float32 _h_approx;
        arva_sim::arva arva_state;
        arva_sim::arva prev_arva_state;
        ros::Subscriber arva_sub;
        
    public:
        TRM(ros::NodeHandle* nh){
            ros::Duration(10.0).sleep();
            state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &TRM::state_cb, this);
            local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            cir_pos_pub = nh->advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming", this);
            set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode", this);

            switch_turn = nh->advertiseService("MTSTurnSwitch", &TRM::switch_turn_rate_cb, this);

            d_actual = nh->advertise<std_msgs::Float32>("d_actual",1);
            drone_pos_client = nh->serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
            
            h_approx = nh->advertise<std_msgs::Float32>("h_approx",1);
            arva_sub = nh->subscribe<arva_sim::arva>("/arva_receiver/signal", 10, &TRM::arva_state_cb, this);
 
            droneState.request.model_name = "iris_arva";
            droneState.request.relative_entity_name = "world";

            offb_set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;

            ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            ptarget.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                //mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::FORCE; 
        
            ptarget.position.z = 2;
            ptarget.velocity.x = 20*PI/90.0;
            ptarget.velocity.y = 0;
            ptarget.velocity.z = 0;

            ptarget.yaw_rate = 20*PI/180.0 ;

            ros::Rate rate(50.0);
            
            ros::Time last_request = ros::Time::now();
            ros::Time take_offtimer = ros::Time::now();
            while(ros::ok && ros::Time::now() - take_offtimer < ros::Duration(20.0)){
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent){
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();

            }

            ros::ServiceServer take_off_trig = nh->advertiseService("take_off_trig", &TRM::take_of_trig_cb, this);
            ros::Time last_switch = ros::Time::now();

            int Ns = 1000;
            Vector2f x_start, x_stop, x_param[Ns], dir;
            Matrix2f A;
            float theta, L, beta, w, a;
            x_start << 0, 0;
            x_stop << 10, 10;
            
            dir = x_stop - x_start;
            L = dir.norm();
            theta = atan2(dir[1], dir[0]);
            w = 2*M_1_PI/L;
            a = L/4;
            A << cos(theta), sin(theta),
                sin(theta), -cos(theta) ;
            for(int i=0; i<Ns; i++){
                x_param[i] << A* Vector2f (beta, a*sin(w*beta));
            }
            sinTraj.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

            sinTraj.type_mask =
                //mavros_msgs::PositionTarget::IGNORE_PX |
                //mavros_msgs::PositionTarget::IGNORE_PY |
                //mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::FORCE; 
        
            sinTraj.position.x = 2;
            sinTraj.position.y = 2;
            sinTraj.position.z = 2;
            sinTraj.velocity.x = 20*PI/90.0;
            sinTraj.velocity.y = 0;
            sinTraj.velocity.z = 0;

            int i = 0;

            while(ros::ok()){
                
                if (i<Ns){
                    
                    sinTraj.position.x = x_param[i][0];
                    sinTraj.position.y = x_param[i][1];
                }                
                cir_pos_pub.publish(sinTraj);

                if (drone_pos_client.call(droneState)){
                    actual_arva_pos = droneState.response.pose.position;
                    // ROS_INFO("x %f, y %f", actual_arva_pos.x, actual_arva_pos.y);
                } 
                _d_actual.data  = 100 * powf( powf(actual_arva_pos.x -50,2) + powf(actual_arva_pos.y -50,2),0.5 );
                // ROS_INFO(arva_state.arva_signals[0].distance);
                // ROS_INFO("Running Till arva state is called ");  
                // _h_approx.data = arva_state.arva_signals[0].distance; // powf(arva_state.arva_signals[0].distance,-0.333); h_approx.publish(_h_approx);
                d_actual.publish(_d_actual);
                ros::spinOnce();
                rate.sleep();
            }

        };

        bool take_of_trig_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            return true;
        }

        void arva_state_cb(const arva_sim::arva::ConstPtr& arva){
            // prev_arva_state = arva_state;
            arva_state = *arva;
        }

        void state_cb(const mavros_msgs::State::ConstPtr& msg){
            current_state = *msg;
        }

        bool switch_turn_rate_cb(src_loc::DoMTSTurnSwitch::Request& request, src_loc::DoMTSTurnSwitch::Response& response){
           ptarget.yaw_rate = -ptarget.yaw_rate; 
           return true;
        }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "MTS_trej");
    ros::NodeHandle nh;
    TRM algo = TRM(&nh);
    ros::spin();
    return 0;
}
