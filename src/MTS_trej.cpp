#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <src_loc/DoMTSTurnSwitch.h>
#include <src_loc/GetSensorMeasurement.h>
#include <src_loc/TerminateAlgo.h>
#include "src_loc/rviz_markers.hpp"

#define PI 3.1415
using namespace Eigen;
using namespace Rviz_Markers;

constexpr float r2d = 180/M_PI;
constexpr float d2r = M_PI/180;
constexpr int sin_traj_points = 1000;
constexpr int trm_samples = 500;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float change_ang_range(float angle){
    if ( angle <= -2*M_PI){
        change_ang_range(angle + 2*M_PI);
    }else return fmod(angle + 2*M_PI, 2*M_PI);
}

class MTS_Switch{
    protected:
        float turn_rate = 10*PI/180; 
        float psi_dot_max = 2 * turn_rate;
        float vel = 40*PI/90; 
        float altitude = 10.0;
        float turn_radius;
        float Time_period;
        float yaw_ini;
        float max_loc_time;
        ros::Time algo_start_time, algo_end_time, traj_start_time;
        bool traj_started = false;
        float v_sl;



        mavros_msgs::State current_state; // variable to for mav state callback

        ros::Subscriber state_sub; // mav state subscriber
        ros::Publisher local_pos_pub; // mavros local setpoint publisher for takeoff
        ros::Publisher cir_pos_pub; // mavros raw local setpoint publisher for circular trajectory
        ros::ServiceClient arming_client; // mavros service client for arming
        ros::ServiceClient set_mode_client; // mavros service client for setting off-board mode

        ros::ServiceServer switch_turn; // service server for turn switching
        mavros_msgs::PositionTarget ptarget; // circular motion position target message

        mavros_msgs::SetMode offb_set_mode; // message for off-board mode
        mavros_msgs::CommandBool arm_cmd; // message for arm mode
        mavros_msgs::CommandBool disarm_cmd; // message for arm mode
        
        ros::ServiceClient drone_pos_client; // service client for getting mav position

        gazebo_msgs::GetModelState droneState; // msg to get gazebo mav state

        // geometry_msgs::Point actual_arva_pos; // message for arva position
        geometry_msgs::PoseStamped pose; // pose msg to publish to mavros, used in takeoff
        geometry_msgs::PoseStamped pose_ground; // pose msg to publish to mavros, used in takeoff
                                              
        ros::Publisher d_actual; // distance from mav to source publisher
        std_msgs::Float32 _d_actual; //  message for distance publisher         
        ros::Publisher h_approx; // arva intensity measurement publisher 
        std_msgs::Float32 _h_approx; // message for arva intensity publisher
        // arva_sim::arva arva_state; // arva message 0for sensor callback
        // arva_sim::arva prev_arva_state; // arva message for storing previous arva measurement
        // ros::Subscriber arva_sub; // arva sensor subscriber
        ros::NodeHandle* nh;
        ros::ServiceServer take_off_trig;
        ros::Subscriber terminate_sub;
        std_msgs::Bool isReached;
        bool term_flag = false;

        // changing model state
        gazebo_msgs::ModelState state_msg;
        gazebo_msgs::SetModelState set_state_msg;
        ros::ServiceClient SetModelStateClinet;

        // landing 
        mavros_msgs::SetMode land_mode;
        mavros_msgs::CommandTOL land_cmd;
        // ros::ServiceClient set_mode_clieuuu
        ros::ServiceClient land_client;


        
    public:
        void setupUavParams(){
            // get turn rate
            if(ros::param::get("/turn_rate", turn_rate)){
                turn_rate = turn_rate * d2r;  // convert deg/sec to rad/sec
                ROS_INFO("Turn rate parameter retrived, %f ",turn_rate);
            }else{
                ROS_WARN("Turn rate parameter not found! ");
            }
            psi_dot_max = 1.5 * turn_rate;
            // get velocity
            if(ros::param::get("/vel", vel)){
                ROS_INFO("Velocity parameter retrived, %f ",vel);
            }else{
                ROS_WARN("Velocity parameter not found! ");
            }
            // get altitude
            if(ros::param::get("/altitude", altitude)){
                ROS_INFO("altitude parameter retrived, %f ",altitude);
            }else{
                ROS_WARN("Altitude parameter not found! ");
            }

            // get initial theta
            if(ros::param::get("/yaw_ini", yaw_ini)){
                yaw_ini *= d2r;
                ROS_INFO("yaw_ini parameter retrived, %f ",yaw_ini);
            }else{
                ROS_WARN("yaw_ini parameter not found! ");
            }
            turn_radius =  vel/ turn_rate;
            ros::param::set("/turn_radius", turn_radius);
            ROS_INFO("Turn Radius parameter is set to %f ", turn_radius);



            Time_period = 2*M_PI / turn_rate;
            ros::param::set("/Time_period", Time_period);
            ROS_INFO("Time period of circular trejectory %f ", Time_period);

            // get maximum localization time 
            if(ros::param::get("/max_time", max_loc_time)){
                ROS_INFO("max_loc_time parameter retrived, %f ",max_loc_time);
            }else{
                ROS_WARN("max_loc_time parameter not found! ");
            }

            // get Sliding mode tuning parameter 
            if(ros::param::get("/v", v_sl)){
                ROS_INFO("v parameter retrived, %f ",v_sl);
            }else{
                ROS_WARN("max_loc_time parameter not found! ");
            }

        }
        void init(){

            setupUavParams();
            // Initialize publishers, subscribers and services

            // mavros state subscriber
            state_sub = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &MTS_Switch::state_cb, this);
            // local setpoint publisher
            local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            // local setpoint_raw publisher for circular trajectory
            cir_pos_pub = nh->advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            // arming service client
            arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming", this);
            // set mode service client
            set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode", this);
            // mav to source distance publisher
            d_actual = nh->advertise<std_msgs::Float32>("d_actual",1);
            // get mav position client
            drone_pos_client = nh->serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
            // arva intensity publisher 
            // h_approx = nh->advertise<std_msgs::Float32>("h_approx",1);
            // arva signal subscriber
            // arva_sub = nh->subscribe<arva_sim::arva>("/arva_receiver/signal", 10, &MTS_Switch::arva_state_cb, this);
            // setup termination
            terminate_sub = nh->subscribe("TerminateAlg", 1, &MTS_Switch::terminate_cb, this);
            
            // info for drone_pos_client
            droneState.request.model_name = "iris";
            // droneState.request.model_name = "typhoon_h480";
            droneState.request.relative_entity_name = "world";

            
            // set offboard request and arm command
            offb_set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;
            disarm_cmd.request.value = false;
            
            // set takeoff position
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = altitude;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = sin(yaw_ini/2); // 0.7071068;
            pose.pose.orientation.w = cos(yaw_ini/2); // 0.7071068;

            pose_ground = pose;
            pose_ground.pose.position.z = 0;
            // set circular trejactory for ptarget
            ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            ptarget.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                // mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::FORCE; 
            ptarget.position.z = altitude;
            ptarget.velocity.x = vel;
            ptarget.velocity.y = 0;
            ptarget.velocity.z = 0;
            // turn_rate = 10*PI/180; // 20 deg/sec turn rate.
            ptarget.yaw_rate = turn_rate;


            // info for setmodelstate.
            // SetModelStateClinet = nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

            // state_msg.model_name = "iris_arva";
            // state_msg.reference_frame = "world";
            // state_msg.pose = pose.pose;
            // state_msg.pose.position.z = 0.2;

            // set_state_msg.request.model_state = state_msg;

            // landing 
            // set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
            // land_client = nh->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
            // land_mode.request.custom_mode = "AUTO.LAND";
            // land_cmd.request.altitude = 0; // Land at current altitude
            // land_cmd.request.min_pitch = 0; // Use default minimum pitch
            // land_cmd.request.yaw = 0; // Use current yaw heading 

            ros::spinOnce();
        };

        void terminate_cb(const std_msgs::Bool::ConstPtr& msg){
            isReached = *msg;
            term_flag = isReached.data;
            if(!term_flag){
                ros::Duration loc_time;
                algo_end_time = ros::Time::now();
                loc_time = algo_end_time - algo_start_time;
                ros::param::set("/loc_time", loc_time.toSec());
            }
            if (traj_started ){
                if( (ros::Time::now() - algo_start_time).toSec() > max_loc_time){
                    ROS_INFO("Time limit exceeded!");
                    term_flag = true;
                }
            }
        }

        void takeoff(){

            ros::Rate rate(50.0);
            
            // ros::spinOnce();
            // if ( current_state.armed ){
            //     land();
            // }

            // ros::Time model_set_timer = ros::Time::now();

            // while(ros::ok() && (ros::Time::now() - model_set_timer).toSec() < 5 ){
            //     if(SetModelStateClinet.call(set_state_msg)){
            //          ROS_INFO("set model state client is called");
            //     }
            //     rate.sleep();
            // }

            
            // setup timers for takeoff and arming
            ros::Time last_request = ros::Time::now();

            // setup offboard mode and arm the drone and takeoff
            while(ros::ok() && (!(droneState.response.pose.position.z > altitude-0.5) 
                            || !(abs(droneState.response.pose.position.x) < 1)
                            || !(abs(droneState.response.pose.position.y) < 1) )
                 )
            {
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
                            // break;
                        }
                        last_request = ros::Time::now();
                    }
                }
                drone_pos_client.call(droneState);
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
                // std::cout << "altitude " << droneState.response.pose.position.z << " set alt " << altitude<< "\n" ;
                // std::cout << "x, y " << droneState.response.pose.position.x << ", " << droneState.response.pose.position.y<< " set " << pose.pose.position.x << ", " << pose.pose.position.y << "\n" ;
                ROS_INFO_THROTTLE(2,"Inside arming loop");
            }
            ROS_WARN("arming ended");
            // wait for few seconds for uav to settle in the origin with required psi_0
            ros::Time take_offtimer = ros::Time::now();
            while(ros::ok() && (ros::Time::now() - take_offtimer).toSec() < 5 ){
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            take_off_trig = nh->advertiseService("take_off_trig", &MTS_Switch::take_of_trig_cb, this);
            ros::spinOnce();
        }
        void land(){
            while (!set_mode_client.call(land_mode) || !land_mode.response.mode_sent){
                ros::spinOnce();
                ros::Duration(2).sleep();

                ROS_INFO_THROTTLE(2,"Inside land mode set loop");
            }   
            ROS_INFO("Landing mode has been set.");
            while (!land_client.call(land_cmd) || !land_cmd.response.success){
                ros::spinOnce();
                ros::Duration(2).sleep();
                ROS_INFO_THROTTLE(2,"Inside land command set loop");
            }
            ROS_INFO("Landing command sent.");
            ros::Duration(15).sleep();
            if ( arming_client.call(disarm_cmd) && arm_cmd.response.success){
                ROS_INFO("disarmed");
            }else{
                ROS_INFO("disarm failed");
            }
        }

        MTS_Switch(){
        }
        MTS_Switch(ros::NodeHandle* _nh){
            nh = _nh;
            traj_start_time = ros::Time::now();
            init();

            // Switch turn service
            switch_turn = nh->advertiseService("MTSTurnSwitch", &MTS_Switch::switch_turn_rate_cb, this);
            ros::Duration(2.0).sleep(); // wait for 10s for gazebo to load
            // fix ros rate
            ros::Rate rate(50.0);

            takeoff();
            
            ROS_INFO("Done take off");
            // Start the take of trigger server once takeoff completed
            ros::Time last_switch = ros::Time::now();

            algo_start_time = ros::Time::now();
            // start circular trajetory and publish distance to source
            while(ros::ok() && !term_flag ){

                // if(isReached.data) break;
                if(!traj_started)
                    traj_started = true;

                ptarget.position.z = altitude;
                cir_pos_pub.publish(ptarget);
                ros::spinOnce();
                rate.sleep();
                ROS_INFO_THROTTLE(1,"inside algo loop");
            }

            // land();
        };

        // service callback for takeoff trigger
        bool take_of_trig_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            return true;
        }

        // callback for arva subscriber
        // void arva_state_cb(const arva_sim::arva::ConstPtr& arva){
        //     // prev_arva_state = arva_state;
        //     arva_state = *arva;
        // }

        // callback for mavros state 
        void state_cb(const mavros_msgs::State::ConstPtr& msg){
            current_state = *msg;
        }

        // Service callback for Turn Switching
        bool switch_turn_rate_cb(src_loc::DoMTSTurnSwitch::Request& request, src_loc::DoMTSTurnSwitch::Response& response){
           ptarget.yaw_rate = -ptarget.yaw_rate; 
           // psi_dot_n = -psi_dot_n;
           return true;
        }

};

class GAMTS_Traj: public MTS_Switch {
    protected:
    // define additional variables needed for GAMTS
    float psi{}, psi_dot_n, b = 0.5, theta_l, theta_l_ini, psi_dot; 
    ros::Subscriber mav_pose_sub; // subscriber for mav pose information
    geometry_msgs::PoseStamped mav_pose; // mav pose message
    ros::ServiceServer switch_turn; // service server for turn switching
    bool isFirstLoop = true;
    Vector2f startPos, endPos, currentPos; // mav positions for each nomimal loop
    
    public:
    void setupGAMTSParams(){
        if(ros::param::get("/b", b)){
            ROS_INFO("b parameter is retireved, b = %f ", b);
        }else{
            ROS_WARN("b parameter has not been found! ");
        };
        if(ros::param::get("/theta_l_ini", theta_l_ini)){
            theta_l_ini *= d2r ; // convert from deg to radians
            ROS_INFO("theta_l_ini parameter is retireved, theta_l_ini = %f ", theta_l_ini);
        }else{
            ROS_WARN("theta_l_ini parameter has not been found! ");
        };
    }
    // callback to get position and yaw of the mav
    void mav_pose_cb(const geometry_msgs::PoseStamped& msg){
        mav_pose = msg;
        // get pose of the mav convert it into roll pitch and yaw.
        float linearposx=msg.pose.position.x;
        float linearposy=msg.pose.position.y;
        double quatx= msg.pose.orientation.x;
        double quaty= msg.pose.orientation.y;
        double quatz= msg.pose.orientation.z;
        double quatw= msg.pose.orientation.w;

        tf::Quaternion q(quatx, quaty, quatz, quatw);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // update current yaw and Position of the mav
        psi =  std::fmod((yaw + 2*M_PI), (2*M_PI));
        currentPos << linearposx,linearposy;
    };
   
    // Redefine the switch turn callback to include the gradient augmentation and theta_l computation.
    bool switch_turn_rate_cb(src_loc::DoMTSTurnSwitch::Request& request, src_loc::DoMTSTurnSwitch::Response& response){
       // ptarget.yaw_rate = -ptarget.yaw_rate; 
       psi_dot_n = -psi_dot_n;
       // std::cout << "cb: " << psi_dot_n << "\n";

       // variable to identify first loop
       isFirstLoop = false;
        theta_l = request.theta_l;
    //    endPos = currentPos;
    //    Vector2f diffPos = startPos - endPos;
    //    theta_l = atan2(diffPos[1],diffPos[0]);

       return true;
    }

    GAMTS_Traj(){

    };

    // Redefine the constructor with nodehandle parameter. 
    GAMTS_Traj(ros::NodeHandle* _nh) {
        nh = _nh;
        init();

        setupGAMTSParams();

        psi_dot_n = turn_rate;
        // psi_dot_max = 2*turn_rate;
        theta_l  = theta_l_ini;

        // Initial the switch turn service with new callback
        switch_turn = nh->advertiseService("MTSTurnSwitch", &GAMTS_Traj::switch_turn_rate_cb, this);
        // Subscriber for obtaining mav pose information
        mav_pose_sub = nh->subscribe("/mavros/local_position/pose", 1, &GAMTS_Traj::mav_pose_cb, this);       
        
        ros::Duration(10.0).sleep(); // wait for 10s for gazebo to load
        // fix ros rate
        ros::Rate rate(50.0);
        takeoff();

        // initiate the start position for the theta_l computation
        startPos = currentPos; 

        // Start the take of trigger server once takeoff completed
        ros::Time last_switch = ros::Time::now();
        algo_start_time = ros::Time::now();
        // start circular trajetory and publish distance to source
        while(ros::ok() && !term_flag ){
            // if(isReached.data) break;
            if(!traj_started)
                traj_started = true;

            // the turn rate depending the gamts algorithm
            if( !isFirstLoop){
                psi_dot = psi_dot_n + b *  sgn(psi_dot_n*(theta_l -psi )) * (theta_l - psi);
                // std::cout << "psi_dot_n " << psi_dot_n 
                            // << "augmented" <<  b *  sgn(psi_dot_n*(theta_l -psi )) * (theta_l - psi)
                            // << "psi_dot_max" << psi_dot_max<< "\n"; 
                if ( std::abs(psi_dot ) > psi_dot_max ){
                    ptarget.yaw_rate = sgn(psi_dot) * psi_dot_max;
                    // ROS_WARN_THROTTLE(1,"turn-rate limit reached");
                }else{
                    ptarget.yaw_rate =  psi_dot;
                }
            }else{
                ptarget.yaw_rate = psi_dot_n;
            }

            cir_pos_pub.publish(ptarget);
            ros::spinOnce();
            rate.sleep();
        }
    }
};

class GDTS_Traj: public MTS_Switch{
   private:
    ros::Subscriber mav_pose_sub;
    tf::Quaternion current_pose_quat;
    float psi{}, psi_dot_n, b = 0.5, theta_l, theta_l_ini, psi_dot; 
    geometry_msgs::PoseStamped mav_pose; // mav pose message
    ros::ServiceServer switch_turn; // service server for turn switching
    bool isFirstLoop = true;
    Vector2f startPos, endPos, currentPos; // mav positions for each nomimal loop

   public: 

    void terminate_cb(const std_msgs::Bool::ConstPtr& msg){
        isReached = *msg;
        term_flag = isReached.data;
        if(!term_flag){
            ros::Duration loc_time;
            algo_end_time = ros::Time::now();
            loc_time = algo_end_time - algo_start_time;
            ros::param::set("/loc_time", loc_time.toSec());
        }
        if (traj_started ){
            if( (ros::Time::now() - algo_start_time).toSec() > max_loc_time){
                ROS_INFO("Time limit exceeded!");
                term_flag = true;
            }
        }
    }

    void mav_pose_cb(const geometry_msgs::PoseStamped& msg){
        mav_pose = msg;
        // get pose of the mav convert it into roll pitch and yaw.
        float linearposx=msg.pose.position.x;
        float linearposy=msg.pose.position.y;
        double quatx= msg.pose.orientation.x;
        double quaty= msg.pose.orientation.y;
        double quatz= msg.pose.orientation.z;
        double quatw= msg.pose.orientation.w;

        tf::Quaternion q(quatx, quaty, quatz, quatw);
        current_pose_quat = q; 
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // update current yaw and Position of the mav
        psi = yaw; // std::fmod((yaw + 2*M_PI), (2*M_PI));
        currentPos << linearposx,linearposy;
    }; 
    

    // Redefine the switch turn callback to include the gradient augmentation and theta_l computation.
    bool switch_turn_rate_cb(src_loc::DoMTSTurnSwitch::Request& request, src_loc::DoMTSTurnSwitch::Response& response){
        // ROS_INFO("switch call back triggered "); 
        theta_l = request.theta_l;
        psi = fmod((psi + 2*M_PI), 2*M_PI);
        theta_l = fmod((theta_l + 2*M_PI), 2*M_PI);
        
        Vector2f psi_vec(cos(psi), sin(psi));
        Vector2f theta_vec(cos(theta_l), sin(theta_l));
        float dot = (theta_vec[0] * -psi_vec[1]) +  (theta_vec[1] * psi_vec[0]);
        // std::cout << "psi_vec : " << psi_vec << "\n";
        // std::cout << "theta_vec : " << theta_vec << "\n";
        // std::cout << "inside switch  theta_l" << theta_l*r2d << " psi: " << psi*r2d << " psi_dot " << psi_dot_n << "\n";
        if (dot * psi_dot_n < 0)
            psi_dot_n = -psi_dot_n;
        ptarget.yaw_rate = psi_dot_n;
       return true;
    };

    GDTS_Traj(){

    };

    GDTS_Traj(ros::NodeHandle* _nh){
        nh = _nh;
        init();
        psi_dot_n = turn_rate;
        theta_l = -M_PI/2;

        switch_turn = nh->advertiseService("MTSTurnSwitch", &GDTS_Traj::switch_turn_rate_cb, this);
        // Subscriber for obtaining mav pose information
        mav_pose_sub = nh->subscribe("/mavros/local_position/pose", 1 , &GDTS_Traj::mav_pose_cb, this);       
        terminate_sub = nh->subscribe("TerminateAlg", 1, &GDTS_Traj::terminate_cb, this);

        ros::Duration(2.0).sleep(); // wait for 10s for gazebo to load
        // fix ros rate
        ros::Rate rate(50.0);
        
        takeoff();
        ros::spinOnce();
        ptarget.yaw_rate = psi_dot_n;
        cir_pos_pub.publish(ptarget);
        algo_start_time = ros::Time::now();
        while (ros::ok() && !term_flag )
        {
            if(!traj_started)
                traj_started = true;
            // ptarget.yaw_rate = psi_dot_n;
            cir_pos_pub.publish(ptarget);
            ros::spinOnce();
            rate.sleep();
        }
    }
};

// Yet to be implemented
class TRM_trej: public rviz_markers, public MTS_Switch {
    private:
    Vector2f sinStart, sinEnd;
    uint no_of_waypoints = 1000;
    std::vector<Vector2f> waypoints;
    std::vector<float> waypoints_heading;
    std::vector<float> waypoints_heading_rate;
    float pos_x_inertial, pos_y_inertial;    
    float root2 = sqrt(2);
    float root2_inv = 1/sqrt(2);
    float Delta = 10;
    float Delta_max = 40;
    float Delta_min = 10;
    float rho;
    Vector2f xk, xk_1;
    float sk, sk_1;
    Matrix2f Rot_phi;
    float amplitude;
    float omega;
    float phi;
    float dx = 0;
    float L = 0;
    float kp1 = 0.1, kp2 = 0.1 , kp3, kp4;
    int look_ahead_idx = 40;

    // int measurement_sample_idx = 7;
    // int mearurement_interval = no_of_waypoints / measurement_sample_idx;

    Eigen::Matrix<float, trm_samples, 5> A{}; 
    Eigen::Matrix<float, 1, 5> f{};
    Eigen::Matrix<float, trm_samples, 1> b{};
    Eigen::Matrix<float, trm_samples, 2> p{};
    Eigen::Matrix<float, trm_samples, 2> X{};
    Eigen::Matrix<float, trm_samples, 1> S{};
    Eigen::Vector2f g{};
    Eigen::Matrix2f G{};
    uint data_idx = 0;

    tf::Quaternion current_pose_quat;

    ros::Subscriber mav_pose_sub; // subscriber for mav pose information
    geometry_msgs::PoseStamped mav_pose; // mav pose message

    ros::ServiceClient sensor_info;
    ros::Subscriber terminate_algo_sub;
    src_loc::GetSensorMeasurement sensor_measurement;
    src_loc::TerminateAlgo terminate_message;

    ros::ServiceClient set_param_client; 
    ros::ServiceClient get_param_client; 
    mavros_msgs::ParamSet param_set;
    mavros_msgs::ParamSet param_get;

    public:

    void get_TRM_params(){

        if(ros::param::get("/kp1", kp1)){
            ROS_INFO("kp1 parameter is retireved, kp1 = %f ", kp1);
        }else{
            ROS_WARN("kp1 parameter has not been found! ");
        };

        if(ros::param::get("/kp2", kp2)){
            ROS_INFO("kp2 parameter is retireved, kp2 = %f ", kp2);
        }else{
            ROS_WARN("kp2 parameter has not been found! ");
        };

        if(ros::param::get("/kp3", kp3)){
            ROS_INFO("kp3 parameter is retireved, kp3 = %f ", kp3);
        }else{
            ROS_WARN("kp3 parameter has not been found! ");
        };

        if(ros::param::get("/kp4", kp4)){
            ROS_INFO("kp4 parameter is retireved, kp4 = %f ", kp4);
        }else{
            ROS_WARN("kp4 parameter has not been found! ");
        };

        if(ros::param::get("/look_ahead_idx", look_ahead_idx)){
            ROS_INFO("look_ahead_idx parameter is retireved, look_ahead_idx = %f ", look_ahead_idx);
        }else{
            ROS_WARN("look_ahead_idx parameter has not been found! ");
        };
    }

    void compute_sin_traj(Vector2f start_point, Vector2f end_point){
        Matrix2f A;
        float theta,  beta, w, a;
        Vector2f dir, traj_point;//, param_points[sin_traj_points];

        dir  = end_point - start_point;
        
        L = dir.norm();
        beta = L/no_of_waypoints;
        theta =  atan2(dir[1], dir[0]) ;
        w = 2*M_PI/L;
        a = L/4;
        amplitude = a;
        omega = w;
        
        phi = theta;
        
        A << cos(theta), -sin(theta),
             sin(theta), cos(theta) ;

        Rot_phi << cos(theta),  sin(theta),
                    -sin(theta), cos(theta) ;
     
        std::cout <<"start point " <<  start_point << "\t" << end_point << "\n";
        std::cout << "L: " << L << " beta: " << beta << ", w: " << w << " a: " << a << "\n";
        waypoints.clear(); 
        waypoints_heading.clear();
        waypoints_heading_rate.clear();
        for(int i=0; i< no_of_waypoints; i++){
            traj_point = start_point + A* Vector2f (beta*i, a*sin(w*beta*i));
            // std::cout << "\n" << "traj point " << "\n"<<  traj_point <<  "\n";
            waypoints.push_back(traj_point);
            // float wp_heading = ( atan2( sin(phi) - cos(phi)*a*w*cos(w*beta*i), 
            //                                             cos(phi) + sin(phi)*a*w*cos(w*beta*i) ) );
            // waypoints_heading.push_back( wp_heading );
        }
        for(int i = 0; i<no_of_waypoints; i++){
            float wp_heading;
            if(i < no_of_waypoints - 1){
               wp_heading =  change_ang_range( 
                                atan2(  waypoints[i+1][1] - waypoints[i][1],
                                        waypoints[i+1][0] - waypoints[i][0] ) ) ; 
            }
            waypoints_heading.push_back( wp_heading );
            float wp_heading_rate = pow(cos(wp_heading),2) * (-a)* pow(w,2) * sin(w*beta*i) ;
            waypoints_heading_rate.push_back( wp_heading_rate);
            // geometry_msgs::Point sine_point;
            // sine_point.x = traj_point[0];
            // sine_point.y = traj_point[1];
            // sine_point.z = altitude;
            // Rviz_Markers::rviz_markers::add_point_sinestrip(sine_point);
            // if(i%100 == 0) update_sine_path_Arrow(wp_heading, sine_point);
            // add_point_sinestrip(sine_point);
            // add_point_linestrip(sine_point);
        }
    }

    float compute_turn_rate(){
        Vector2f min_vec(10e5,10e5);
        Vector2f look_ahead_min_vec;
        float min_vec_norm = min_vec.norm();
        float wp_heading = waypoints_heading[0];
        float look_ahead_min_vec_norm;
        float look_ahead_wp_heading;
        int min_wp_idx = 0;
        for (int i=0; i < no_of_waypoints-look_ahead_idx;i++){
            if( (currentPos - waypoints[i]).norm() <= min_vec_norm ){
                min_vec = waypoints[i] - currentPos;
                min_vec_norm = min_vec.norm();
                wp_heading = waypoints_heading[i];
                min_wp_idx = i;
            }
        }
            int look_ahead_wp_idx = min_wp_idx + look_ahead_idx; 
                look_ahead_min_vec = currentPos - waypoints[look_ahead_wp_idx];
                look_ahead_min_vec_norm = look_ahead_min_vec.norm();
                look_ahead_wp_heading = waypoints_heading[look_ahead_wp_idx];
        
        geometry_msgs::Point look_ahead_waypointPoint;
        look_ahead_waypointPoint.x = waypoints[min_wp_idx][0];
        look_ahead_waypointPoint.y = waypoints[min_wp_idx][1]; 
        look_ahead_waypointPoint.z = altitude;
        update_min_vec_Arrow(look_ahead_waypointPoint, currentPoint);
        Vector2f look_ahead_heading_vec(cos(look_ahead_wp_heading), sin(look_ahead_wp_heading));
        // std::cout << "heading now " << wp_heading*r2d << "\n";
        update_sine_Arrow(look_ahead_wp_heading, currentPoint);


        float chi = change_ang_range( atan2(min_vec[1], min_vec[0]) );
        psi = change_ang_range(psi);

        Vector3f look_ahead_min_vec_3d(look_ahead_min_vec[0] , look_ahead_min_vec[1], 0);  
        Vector3f min_vec_3d(min_vec[0] , min_vec[1], 0);  
        Vector3f look_ahead_heading_3d(cos(look_ahead_wp_heading), sin(look_ahead_wp_heading), 0);
        Vector3f wp_heading_3d(cos(wp_heading), sin(wp_heading), 0);
        Vector3f psi_3d(cos(psi), sin(psi), 0);

        float heading_error = psi - look_ahead_wp_heading;
        // std::cout << "heading error : " << heading_error*r2d ;
        // std::cout << "heading correction : " << kp1 * sin(fmod(abs(heading_error), M_PI)) * sgn(look_ahead_heading_3d.cross(psi_3d)[2]) 
                //   << " dist correction " << kp2 * tanh(look_ahead_min_vec_norm) *sgn( (look_ahead_heading_3d.cross(min_vec_3d))[2] ) << "\n";
        float theta_dot;

        if ( min_vec_norm < 0.4){
            float rate_error  = waypoints_heading_rate[look_ahead_wp_idx] - theta_dot;
            theta_dot = kp1 * tanh(fmod(abs(heading_error), M_PI)) * sgn(psi_3d.cross(look_ahead_heading_3d)[2])
                        + kp4 * rate_error; 
            // std::cout << "only heading following " << "\n";
            // std::cout << "turn_rate:" << theta_dot << "\n";
        }
        else {
            float heading_correction_2 = kp3 * tanh(fmod(abs(heading_error), M_PI)) * sgn(psi_3d.cross(look_ahead_heading_3d)[2]) ; 
            float distance_correction  =  kp2   * tanh(look_ahead_min_vec_norm) * sgn(look_ahead_min_vec_3d.cross(look_ahead_heading_3d)[2]) 
                                                * sgn(wp_heading_3d.dot(psi_3d));
            theta_dot =  heading_correction_2 + distance_correction;

            // std::cout << " distance correctoin " << "\n";
            // std:: cout << "angle corerection: " << heading_correction_2 << "\n" <<
            //               "distance correction: " << distance_correction << "\n" <<  
            //               "sign : " << sgn(look_ahead_heading_3d.cross(look_ahead_min_vec_3d)[2]) <<
            //               "dot sign : " << sgn(look_ahead_heading_vec.dot(look_ahead_min_vec)) <<
            //               "\n" ; 
        }
        // std::cout << "turn rate :" << theta_dot*r2d <<"\n";
        return theta_dot; 

    }
    // void setup_straight_line_motion(float x_pos, float y_pos, float yaw){
        
    //     // set straight line trejactory for ptarget
    //     ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED ;
    //     ptarget.type_mask =
    //         // mavros_msgs::PositionTarget::IGNORE_VX |
    //         // mavros_msgs::PositionTarget::IGNORE_VY |
    //         // mavros_msgs::PositionTarget::IGNORE_VZ |
    //         mavros_msgs::PositionTarget::IGNORE_AFX |
    //         mavros_msgs::PositionTarget::IGNORE_AFY |
    //         mavros_msgs::PositionTarget::IGNORE_AFZ |
    //         // mavros_msgs::PositionTarget::IGNORE_PZ |
    //         mavros_msgs::PositionTarget::IGNORE_YAW   | 
    //         mavros_msgs::PositionTarget::FORCE     |
    //         mavros_msgs::PositionTarget::IGNORE_YAW_RATE 
    //         ;
    //     ptarget.position.x = x_pos;
    //     ptarget.position.y = y_pos;
    //     ptarget.position.z = altitude;
    //     // ptarget.yaw = yaw;
    //     // ptarget.velocity.x = vel;
    //     // ptarget.velocity.y = vel;
    //     // ptarget.velocity.z = 0;
    //     // turn_rate = 10*PI/180; // 20 deg/sec turn rate.
    //     // ptarget.yaw_rate = 0;
    // } 
    
    void setup_sine_motion(){
        
        // set straight line trejactory for ptarget
        ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED ;
        ptarget.type_mask =
            // mavros_msgs::PositionTarget::IGNORE_VX |
            // mavros_msgs::PositionTarget::IGNORE_VY |
            // mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY | 
            // mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW   | 
            mavros_msgs::PositionTarget::FORCE     
            // mavros_msgs::PositionTarget::IGNORE_YAW_RATE 
            ;

        // float yaw = atan(amplitude * omega * cos(omega*dx)) + phi ; 
        // // float yaw = psi;
        // float yaw_rate = (float)(amplitude*powf(omega,2)*cos(omega*dx) * vel * pow(cos(yaw),3));
        // dx += vel *cos(yaw-phi)* 1/50;//abs(xk[0] - (Rot_phi*currentPos)[0]);
        // // std::cout << "dx: " << dx << "yaw: " << yaw*r2d  << "yaw_rate: " << yaw_rate*r2d << "\n";
        // ptarget.yaw = -yaw;

        ptarget.velocity.x = vel;
        ptarget.velocity.y = 0;
        ptarget.velocity.z = 0;
        ptarget.position.z = 0;
        // turn_rate = 10*PI/180; // 20 deg/sec turn rate.
        ptarget.yaw_rate = -1*compute_turn_rate();
    } 

    bool check_waypoint_reached(Eigen::Vector2f current_pos, Eigen::Vector2f target){
        return ((current_pos - target).norm() < 1.0); 
    }

    // void mav_pose_cb(const geometry_msgs::PoseStamped& msg){
    //     mav_pose = msg;
    //     // get pose of the mav convert it into roll pitch and yaw.
    //     pos_x_inertial = msg.pose.position.x;
    //     pos_y_inertial = msg.pose.position.y;
    //     double quatx= msg.pose.orientation.x;
    //     double quaty= msg.pose.orientation.y;
    //     double quatz= msg.pose.orientation.z;
    //     double quatw= msg.pose.orientation.w;

    //     tf::Quaternion q(quatx, quaty, quatz, quatw);
    //     current_pose_quat = q; 
    //     tf::Matrix3x3 m(q);
    //     double roll, pitch, yaw;
    //     m.getRPY(roll, pitch, yaw);

    //     // update current yaw and Position of the mav
    //     psi = yaw; // std::fmod((yaw + 2*M_PI), (2*M_PI));
    //     currentPos << pos_x_inertial,pos_y_inertial;
    // }; 

    void update_measurements(){
        sensor_info.call(sensor_measurement);
        S[data_idx] = -sensor_measurement.response.Measurement;
        X(data_idx,0) = currentPos[0];
        X(data_idx,1) = currentPos[1];
        // std::cout << "S " << S << "\n";
        // std::cout << "X " << X << "\n";
        data_idx++;
    }
    
    float current_measurent(){
        sensor_info.call(sensor_measurement);
        return -sensor_measurement.response.Measurement;
    }

    void estimate_model(){
        // std::cout << "inside estimate \n";
        int data_end = data_idx - 1;
        std::cout << "data_idx " <<  data_idx;
        p.col(0) = X.col(0) - VectorXf::Ones(data_idx) * X(data_end,0);
        p.col(1) = X.col(1) - VectorXf::Ones(data_end) * X(data_end,1);
        // std::cout << "p: " << p << "\n ";
        A.col(0) = p.col(0);
        A.col(1) = p.col(1);
        A.col(2) = p.col(0).cwiseProduct(p.col(1));
        A.col(3) = p.col(0).cwiseProduct(p.col(0))*root2_inv;
        A.col(4) = p.col(1).cwiseProduct(p.col(1))*root2_inv;
        // std::cout << "A: " << A << "\n";
        b = S - VectorXf::Ones(data_idx)* S(data_end,0);
        // std::cout << "b: " << b << "\n";
        MatrixXf _A = A.block(0, 0, data_idx, 5);
        // std::cout << "_A: " << _A << "\n";
        VectorXf _b = b.block(0, 0, data_idx, 1);
        // std::cout << "_b: " << _b << "\n";
        // VectorXf e = (_A.transpose() * _A).inverse()*_A.transpose()*_b;
        // VectorXf e = _A.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(_b);  
        VectorXf e = _A.fullPivHouseholderQr().solve(_b);
        g << e(0), e(1);
        G <<    root2*e(3), e(2),
                e(2),       root2*e(4);                     
    }

    Vector2f trm_subproblem(Vector2f _g, Matrix2f _G){
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(_G);
        Vector2f eigen_values = eigensolver.eigenvalues();
        std::cout << "grad: \n" << _g << " \n Hessian:\n" << _G << "\n"; 
        std::cout <<  "eigen values : \n"<< eigen_values << "\n"; 
        Matrix2f B;
        if ( eigen_values[0] > 0 && eigen_values[1] > 0){
            std::cout << "Positive Def G \n";
            // Vector2f Ginvg = _G.inverse()*_g;
            // float Ginvg_norm = Ginvg.norm();
            // if ( Ginvg_norm <= Delta){
            //     return -Ginvg;
            // }else{
            //     return -(Delta/Ginvg_norm) * Ginvg; 
            // }
            B = _G;
        }else{
            std::cout << "indeterminant G \n";
            float lambda = -eigen_values.minCoeff() + 0.01;
            for(int i=0;i<3;i++){
                std::cout << "iter : " << i << ", lambda: " << lambda << "\n";
                LLT<Matrix2f> chol_dec( _G + lambda* Matrix2f::Identity());
                Matrix2f R(chol_dec.matrixU());
                Vector2f p = -(R*R.transpose()).llt().solve(_g);
                Vector2f q = (R.transpose()).llt().solve(p);
                lambda = lambda + pow(p.norm()/q.norm(),2)* ((p.norm() - Delta)/Delta);
            }
            B = (_G + lambda * Matrix2f::Identity());
        }
        Vector2f p = -B.inverse()*_g;
        if((p).norm() > Delta){
            return (Delta / (p).norm()) * p;
        }else{
            return p;
        }
    }

    float model_prediction(Vector2f _xk, float f0){
        Vector2f pk =  _xk - xk_1;
        return f0 + g.transpose()*pk + 0.5 * pk.transpose()*G*pk;
    }
    
    void limit_all_velocity(float xy_vel){
        param_set.request.param_id = "MPC_XY_VEL_ALL"; 
        param_get.request.param_id = "MPC_XY_VEL_ALL"; 
        param_set.request.value.real = xy_vel;
        set_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        get_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/get");

        if (set_param_client.call(param_set) && param_set.response.success)
            ROS_WARN("Set MPC_XY_VEL_ALL to %f", param_set.response.value.real);
        else
            ROS_WARN("Failed to set MPC_XY_VEL_ALL to %f", param_set.response.value.real);

        // if (get_param_client.call(param_get) && param_get.response.success)
        //     ROS_INFO("get MPC_XY_VEL_MAX to %f", param_get.response.value.real);
        // else
        //     ROS_WARN("Failed to get MPC_XY_VEL_MAX ");
    }

    void limit_max_velocity(float xy_vel){
        param_set.request.param_id = "MPC_XY_VEL_MAX"; 
        param_get.request.param_id = "MPC_XY_VEL_MAX"; 
        param_set.request.value.real = xy_vel;
        set_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        get_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/get");

        if (set_param_client.call(param_set) && param_set.response.success)
            ROS_WARN("Set MPC_XY_VEL_MAX to %f", param_set.response.value.real);
        else
            ROS_WARN("Failed to set MPC_XY_VEL_MAX to %f", param_set.response.value.real);

        // if (get_param_client.call(param_get) && param_get.response.success)
        //     ROS_INFO("get MPC_XY_VEL_MAX to %f", param_get.response.value.real);
        // else
        //     ROS_WARN("Failed to get MPC_XY_VEL_MAX ");
    }

    
    TRM_trej(){
    }

    TRM_trej(ros::NodeHandle* _nh){
        nh = _nh;
        init();

        update__marker_nh(_nh);
        init_markers();
        get_TRM_params();
        sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
        terminate_algo_sub = nh->subscribe("TerminateAlg", 1 ,&MTS_Switch::terminate_cb, dynamic_cast<MTS_Switch*>(this));

        sinStart << 0.0, 0.0;
        sinEnd <<  Delta*cos(yaw_ini), Delta*sin(yaw_ini);
        xk = sinStart;
        sk = current_measurent();

        compute_sin_traj(sinStart, sinEnd);

        ros::Rate rate(80);
        // ros::Duration(10.0).sleep(); // wait for 10s for gazebo to load
        // yaw_ini = fmod(atan(amplitude*omega) + phi,2*M_PI );
        // pose.pose.orientation.z = sin(yaw_ini/2); // 0.7071068;
        // pose.pose.orientation.w = cos(yaw_ini/2); // 0.7071068;
        
        takeoff();

        // limit_max_velocity(5);    
        // limit_max_velocity(12.0);
        // limit_all_velocity(10.0);

        ros::spinOnce();
        // cir_pos_pub.publish(ptarget);
        algo_start_time = ros::Time::now();

        
        // init();
        // takeoff();
        // cir_pos_pub.publish(ptarget);
        std::cout << "next Point" << sinEnd << "\n";
        ros::spinOnce();

        std::cout << "before for loop for sine points viz" << "\n";
        setup_sine_motion();
        cir_pos_pub.publish(ptarget);
        ROS_WARN("Before for loop");
        uint wp_i = 0;
        int switch_count = 0;

        Vector2f next_point, pk;
            for (int i= 0; i< no_of_waypoints; i++){
                // std::cout << "adding points to sine line and arrows" << "\n";
                geometry_msgs::Point sinePoint;
                sinePoint.x = waypoints[i][0];
                sinePoint.y  = waypoints[i][1];
                sinePoint.z = altitude;
                add_point_sinestrip(sinePoint);
                // if(i%100 == 0){ update_sine_path_Arrow(waypoints_heading[i], sinePoint); }
            }
        ros::Time sensor_timer = ros::Time::now();

        while(ros::ok() && !term_flag){
            
            if( ros::Time::now().toSec() - sensor_timer.toSec() > 0.5){
                update_measurements();
                // std::cout << "data_idx: " << data_idx << "\n";
                sensor_timer = ros::Time::now();
                ros::spinOnce();
            }

            if (check_waypoint_reached(waypoints[no_of_waypoints-look_ahead_idx-10], currentPos) ){

                if(switch_count == 0){
                    xk_1 = xk;
                    xk =  currentPos;// X.row( trm_samples - 1 );
                    sk_1 = current_measurent(); // -sensor_measurement.response.Measurement;

                    std::cout << "Switch count : " << switch_count << "\n";
                    estimate_model();
                    std::cout << "xk: \n" << xk << "xk_1: \n" << xk_1 << "\n";
                    pk = trm_subproblem(g, G);
                    next_point = currentPos + pk;
                    switch_count++;
                }else{
                    // estimate_model();
                    xk_1 = xk;
                    xk =  currentPos;// X.row( trm_samples - 1 );
                    // sensor_info.call(sensor_measurement);//S(trm_samples - 1);
                    sk_1 = current_measurent(); //-sensor_measurement.response.Measurement;

                    std::cout << "Switch count : " << switch_count << "\n";
                    std::cout << "xk: \n" << xk << "\n xk_1: \n" << xk_1 << "\n";
                    rho = -(sk- sk_1)/ (model_prediction(xk,sk_1) - model_prediction(xk_1, sk_1));
                    std::cout << "rho: " << rho << "\n";
                    if(rho < 0.25){
                        Delta = std::max(Delta/2, Delta_min);
                        next_point = xk_1;
                        // xk_1 = xk;
                        // xk = currentPos;
                    }else if (rho > 0.8) {
                        Delta = std::min((float)(Delta*1.5), Delta_max);
                        estimate_model();
                        pk = trm_subproblem(g, G);
                        next_point = currentPos + pk;
                    }
                    switch_count++;
                }
                std::cout << "Pk: " << pk << "\n";
                std::cout << "next point: " << next_point << "\n";
                X.setZero();
                S.setZero();
                data_idx = 0;
                std::cout << "grad est " << "\n" << g <<"\n";
                std::cout << "Hessian est " << "\n" << G <<"\n";

                // compute_sin_traj(currentPos, next_point);
                // next_point = currentPos + Vector2f(Delta*sin(M_PI/4), Delta*cos(M_PI/4)); 
                compute_sin_traj(currentPos, next_point);
                for (int i= 0; i< no_of_waypoints; i++){
                    // std::cout << "adding points to sine line and arrows" << "\n";
                    geometry_msgs::Point sinePoint;
                    sinePoint.x = waypoints[i][0];
                    sinePoint.y  = waypoints[i][1];
                    sinePoint.z = altitude;
                    add_point_sinestrip(sinePoint);
                    // update_sine_path_Arrow(waypoints_heading[i],sinePoint); 
                }
            }
            add_point_linestrip(currentPoint);
            
            // test 
            // setup_straight_line_motion(waypoints[0][0], waypoints[0][1], 0);
            // std::cout << "dx : " << dx << " L: " << L << "\n" ;
            // std::cout << "condition for measurement: " << fmod(dx, L/trm_samples) << "\n";
            ptarget.yaw_rate = compute_turn_rate();
            cir_pos_pub.publish(ptarget);
            ros::spinOnce();
            rate.sleep();
        }

        // limit_max_velocity(12.0);
        // limit_all_velocity(12.0);
    } 
    
};

class SL_traj: public rviz_markers, public MTS_Switch{
    private:
        float d_dot, v = 0.1;
        int field_param;
        // long d_vec[2] = {0.0}; 
        double d_prev = 0;
        double d_current = 0;
        uint d_idx = 0;
        ros::ServiceClient sensor_info;
        ros::Subscriber terminate_algo_sub;

        src_loc::GetSensorMeasurement sensor_measurement;
        src_loc::TerminateAlgo terminate_message;

    public:

    void update_d_dot(){
        ros::Time t_start = ros::Time::now();
        ros::spinOnce();
        if( field_param != 3) (ros::Rate(1)).sleep();
        sensor_info.call(sensor_measurement);
        ros::Time t_stop = ros::Time::now();
        ros::spinOnce();
        double dt = t_stop.toSec() - t_start.toSec();
        // std::cout << "start = " << t_start.toSec()  << " stop = " << t_stop.toSec() << "\n";
        // std::cout << dt << "\n" ;
        double meas = sensor_measurement.response.Measurement;
        d_current = meas;
        d_dot = (d_current - d_prev ) / dt; 
        d_prev = d_current;
        // d_vec[d_idx] = meas;
        // d_dot = (meas - d_vec[(d_idx-1+2)%2]) / dt  ;
        // d_idx = (d_idx+1)%2;
        // std::cout << " dt" << dt << "d_dot :" << d_dot << " vec" << d_prev << " " << d_current << "\n";

    }
    
    // void terminate_cb(const std_msgs::Bool::ConstPtr &msg){
    //     isReached = *msg;
    //     term_flag = isReached.data;
    // } 

    SL_traj(){};
    SL_traj(ros::NodeHandle* _nh){
        nh = _nh;

        if(ros::param::get("/field_param", field_param)){
            ROS_INFO("field_param = %d", field_param );
        }else{
            ROS_ERROR("field_param parameter doesn't exists!!");
        }

        init();
        update__marker_nh(_nh);
        init_markers();
        v = v_sl;
        sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
        terminate_algo_sub = nh->subscribe("TerminateAlg", 1 ,&MTS_Switch::terminate_cb, dynamic_cast<MTS_Switch*>(this));
        ros::Rate rate(50);

        takeoff();
        ros::spinOnce();
        cir_pos_pub.publish(ptarget);
        algo_start_time = ros::Time::now();

        sensor_info.call(sensor_measurement);
        d_prev = sensor_measurement.response.Measurement ;  

        while (ros::ok() && !term_flag )
        {
            if(!traj_started)
                traj_started = true;
            update_d_dot();
            ptarget.yaw_rate = psi_dot_max * sgn(d_dot - v);
            cir_pos_pub.publish(ptarget);
            add_point_linestrip(currentPoint);
            ros::spinOnce();
            rate.sleep();
        }

    }

};

class zig_zag: public rviz_markers, public MTS_Switch{

    private:
    // variables to keep track of signal strength and rate of signal strength
    double N_i, N_i_1, N_dot_i, N_dot_i_1;
    // tumble angles and tumble distance
    float tumble_dist = 2;
    float tumble_angle_1 = 120*d2r;
    float tumble_angle_2 = 90*d2r;
    float tumble_angle_3 = 60*d2r;
    float tumble_angle_4 = 30*d2r;
    float tumble_angle_base = 0;
    
    int k1 = 0, k2 = 0, k3 = 0, k4 = 0;
    int K1 = 3, K2 = 4, K3 = 3;
    float p = 0.1;
    int sgn_flag = -1;
    float r_i, r_i_1, dr;
    float pos_x_inertial, pos_y_inertial;    
    bool isWayPointReached;
    int step_count = 0;
    ros::ServiceClient set_param_client; 
    ros::ServiceClient get_param_client; 
    mavros_msgs::ParamSet param_set;
    mavros_msgs::ParamSet param_get;

    tf::Quaternion current_pose_quat;

    ros::ServiceClient sensor_info;
    ros::Subscriber terminate_algo_sub;
    ros::Subscriber range_sub;

    src_loc::GetSensorMeasurement sensor_measurement;
    src_loc::TerminateAlgo terminate_message;
    std_msgs::Float32 range_msg;


    ros::Subscriber mav_pose_sub; // subscriber for mav pose information
    geometry_msgs::PoseStamped mav_pose; // mav pose message

    public:

    void update_N(){
        N_i_1 = N_i;
        sensor_info.call(sensor_measurement);
        N_i = sensor_measurement.response.Measurement;
        r_i_1  = r_i; 
        r_i = range_msg.data;
        dr = r_i - r_i_1;
    }

    void update_N_dot(){
        N_dot_i_1 = N_dot_i;
        N_dot_i = (N_i - N_i_1) / dr; 
    }

    bool check_waypoint_reached(Eigen::Vector2f current_pos, Eigen::Vector2f target){
        return ((current_pos - target).norm() < 0.2); 
    }

    // void terminate_cb(const std_msgs::Bool::ConstPtr &msg){
    //     isReached = *msg;
    //     term_flag = isReached.data;
    // } 

    void range_sub_cb(const std_msgs::Float32::ConstPtr &msg){
        range_msg = *msg;
    }

    void setup_straight_line_motion(float x_pos, float y_pos, float yaw){
        
        // set straight line trejactory for ptarget
        ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED ;
        ptarget.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            // mavros_msgs::PositionTarget::IGNORE_PZ |
            // mavros_msgs::PositionTarget::IGNORE_YAW |
            mavros_msgs::PositionTarget::FORCE      
            // mavros_msgs::PositionTarget::IGNORE_YAW_RATE 
            ;
        ptarget.position.x = x_pos;
        ptarget.position.y = y_pos;
        ptarget.position.z = altitude;
        ptarget.yaw = yaw;
        // ptarget.velocity.x = vel;
        // ptarget.velocity.y = 0;
        // ptarget.velocity.z = 0;
        // turn_rate = 10*PI/180; // 20 deg/sec turn rate.
        ptarget.yaw_rate = 0;
    } 

    void mav_pose_cb(const geometry_msgs::PoseStamped& msg){
        mav_pose = msg;
        // get pose of the mav convert it into roll pitch and yaw.
        pos_x_inertial = msg.pose.position.x;
        pos_y_inertial =msg.pose.position.y;
        double quatx= msg.pose.orientation.x;
        double quaty= msg.pose.orientation.y;
        double quatz= msg.pose.orientation.z;
        double quatw= msg.pose.orientation.w;

        tf::Quaternion q(quatx, quaty, quatz, quatw);
        current_pose_quat = q; 
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // update current yaw and Position of the mav
        psi = yaw; // std::fmod((yaw + 2*M_PI), (2*M_PI));
        currentPos << pos_x_inertial,pos_y_inertial;
    }; 

    float compute_theta(float theta_){
        update_N();
        update_N_dot();
        float del_theta;
        
        if (step_count >=2) {
            if (N_dot_i - N_dot_i_1 > 0 ){
                k1++;
                k2++;
            }
            else{
                k1 = 0;
                k2 = 0;
            }
            // if (N_dot_i > p ) k3++;
            // else k3 = 0;
        }

        if( k1 == K1) theta_ = tumble_angle_2;
        else if( k2 == K2) theta_= tumble_angle_3;
        // else if( k3 == K3) theta_ = tumble_angle_4;
        
        tumble_angle_base = theta_;
        sgn_flag *= -1;
        if (N_i > N_i_1) {
            std::cout << "Increase in concentration"<< ", N_i: " << N_i << ", N_i_i : "<< N_i_1 << "\n" ;
            int small_angle_sgn = 1;
            if (rand() % 2) {
              small_angle_sgn  = -1;
            } 
            del_theta =  small_angle_sgn * (float(rand() % 300 + 0) /100)*d2r;
        }
        else {
            std::cout << "Decrease in concentration"<< ", N_i: " << N_i << ", N_i_i : "<< N_i_1 << "\n" ;
            del_theta = (float(rand() % 1000 + 1000)/100)*d2r;
            // return theta_ * sgn_flag + del_theta; 
        }
        std::cout << "sgn_flag " << sgn_flag << "\n" << "K : " << k1 << " " << k2 << " " << k3 << " " << "\n";
        std::cout << "del_theta " << (del_theta)*r2d << "\n";
        std::cout << "theta " << (theta_ * sgn_flag + del_theta)*r2d << "\n";
        std::cout << "theta_base " << tumble_angle_base*r2d << "\n";
        return (theta_ + del_theta)*sgn_flag;
    }
    void setup_zig_zag_parameters(){
        if(ros::param::get("/K1", K1)){
            ROS_INFO("K1 parameter is retireved, K1 = %f ", K1);
        }else{
            ROS_WARN("K1 parameter has not been found! ");
        };

        if(ros::param::get("/K2", K2)){
            ROS_INFO("K2 parameter is retireved, K2 = %f ", K2);
        }else{
            ROS_WARN("K2 parameter has not been found! ");
        };

        if(ros::param::get("/K3", K3)){
            ROS_INFO("K3 parameter is retireved, K3 = %f ", K3);
        }else{
            ROS_WARN("K3 parameter has not been found! ");
        };

        if(ros::param::get("/tumble_dist", tumble_dist)){
            ROS_INFO("tumble_dist parameter is retireved, tumble_dist = %f ", tumble_dist);
        }else{
            ROS_WARN("tumble_dist parameter has not been found! ");
        };
    }
    void limit_all_velocity(float xy_vel){
        param_set.request.param_id = "MPC_XY_VEL_ALL"; 
        param_get.request.param_id = "MPC_XY_VEL_ALL"; 
        param_set.request.value.real = xy_vel;
        set_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        get_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/get");

        if (set_param_client.call(param_set) && param_set.response.success)
            ROS_WARN("Set MPC_XY_VEL_ALL to %f", param_set.response.value.real);
        else
            ROS_WARN("Failed to set MPC_XY_VEL_ALL to %f", param_set.response.value.real);

        // if (get_param_client.call(param_get) && param_get.response.success)
        //     ROS_INFO("get MPC_XY_VEL_MAX to %f", param_get.response.value.real);
        // else
        //     ROS_WARN("Failed to get MPC_XY_VEL_MAX ");
    }

    void limit_max_velocity(float xy_vel){
        param_set.request.param_id = "MPC_XY_VEL_MAX"; 
        param_get.request.param_id = "MPC_XY_VEL_MAX"; 
        param_set.request.value.real = xy_vel;
        set_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        get_param_client = nh->serviceClient<mavros_msgs::ParamSet>("/mavros/param/get");

        if (set_param_client.call(param_set) && param_set.response.success)
            ROS_WARN("Set MPC_XY_VEL_MAX to %f", param_set.response.value.real);
        else
            ROS_WARN("Failed to set MPC_XY_VEL_MAX to %f", param_set.response.value.real);

        // if (get_param_client.call(param_get) && param_get.response.success)
        //     ROS_INFO("get MPC_XY_VEL_MAX to %f", param_get.response.value.real);
        // else
        //     ROS_WARN("Failed to get MPC_XY_VEL_MAX ");
    }
    zig_zag(){};
    zig_zag(ros::NodeHandle* _nh){
        nh = _nh;

        setup_zig_zag_parameters();
        init();   

        update__marker_nh(_nh);
        init_markers();

        sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
        terminate_algo_sub = nh->subscribe("TerminateAlg", 1 ,&MTS_Switch::terminate_cb, dynamic_cast<MTS_Switch*>(this));
        range_sub = nh->subscribe("range", 1 ,&zig_zag::range_sub_cb, this);
        mav_pose_sub = nh->subscribe("/mavros/local_position/pose", 1 , &zig_zag::mav_pose_cb, this);       
        ros::Rate rate(50);

        takeoff();
        ros::spinOnce();
        r_i_1 = range_msg.data;
        cir_pos_pub.publish(ptarget);
        algo_start_time = ros::Time::now();

        sensor_info.call(sensor_measurement);
        N_i_1 = sensor_measurement.response.Measurement ;  
        N_i = N_i_1;
        N_dot_i = 0;
        // update_N();

        Eigen::Vector2f pos_ref(currentPos); 
        
        float theta = tumble_angle_1;
        tumble_angle_base = theta;
        float theta_prev = 0 * d2r;


        float pos_x_local_target =  tumble_dist*cos(yaw_ini); 
        float pos_y_local_target =  tumble_dist*sin(yaw_ini); 
        Eigen::Vector2f pos_local_target(pos_x_local_target , pos_y_local_target);

        Eigen::Vector2f pos_inertial_target(pos_ref + pos_local_target); 

        setup_straight_line_motion( pos_inertial_target[0],
                                    pos_inertial_target[1],
                                    yaw_ini
                                    );

        limit_max_velocity(vel);    
        limit_all_velocity(vel);    
        while (ros::ok() && !term_flag )
        {
            if(!traj_started)
                traj_started = true;

            if( check_waypoint_reached(currentPos, pos_inertial_target) ){
                if (step_count <= 2) step_count++ ;

                // ROS_WARN("check point reached");

                theta = compute_theta( tumble_angle_base );

                theta_prev = atan2((currentPos - pos_ref)[1], (currentPos - pos_ref)[0]);  
                theta_prev = std::fmod(theta_prev+2*M_PI, 2*M_PI);
                pos_local_target <<  tumble_dist*cos(theta + theta_prev), tumble_dist*sin(theta + theta_prev); 
                pos_inertial_target =  currentPos + pos_local_target; 
                pos_ref = currentPos;
                // theta = std::fmod((psi+theta+2*M_PI),2*M_PI);
                setup_straight_line_motion( pos_inertial_target[0],
                                            pos_inertial_target[1],
                                            std::fmod(theta + theta_prev+ 2*M_PI, 2*M_PI)
                                            );

                // std::cout << "theta_prev : " <<theta_prev*r2d << "\n" << "theta " << (theta)*r2d  << " " << "\n";
                // std::cout << "final angle : "<< std::fmod(theta + theta_prev+ 2*M_PI, 2*M_PI)*r2d << "\n";
            }

            update_N();
            cir_pos_pub.publish(ptarget);
            add_point_linestrip(currentPoint);
            ros::spinOnce();
            rate.sleep();
        }
    
        limit_max_velocity(-10.0);
        limit_all_velocity(-10.0);
    
    }
 
};


int main(int argc, char** argv){
    ros::init(argc, argv, "MTS_trej");
    ros::NodeHandle nh;

    int algo_num;
    if(ros::param::get("/algo_num", algo_num)){
        ROS_INFO("algo_num = %d", algo_num);
    }else{
        ROS_ERROR("algo_num parameter doesn't exists!!");
    }

    if (algo_num == 1){
        MTS_Switch algo = MTS_Switch(&nh);
    }
    else if (algo_num == 2){
        GAMTS_Traj algo = GAMTS_Traj(&nh);
    }
    else if (algo_num == 3){
        GDTS_Traj algo = GDTS_Traj(&nh);
    }
    else if (algo_num == 4){
        SL_traj algo = SL_traj(&nh);
    }
    else if (algo_num == 5){
        zig_zag algo = zig_zag(&nh);
    }
    else if (algo_num == 6){
        TRM_trej algo = TRM_trej(&nh);
    }

    // TRM algo = TRM(&nh);
    // ros::spin();
    return 0;
}
