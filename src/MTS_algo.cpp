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
#include <gazebo_msgs/GetModelState.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <array>
#include <random>
#include "src_loc/rviz_markers.hpp"
#include <src_loc/GetSensorMeasurement.h>
#include <src_loc/DoMTSTurnSwitch.h>
#include <src_loc/TerminateAlgo.h>
#include <src_loc/GetTRMNextPos.h>



using namespace Rviz_Markers;

using std::array;
using namespace Eigen;
constexpr int sample_N = 3;
constexpr int dataPts = 500;
constexpr int samples = 10;
constexpr float r2d = 180/M_PI;
constexpr float d2r = M_PI/180;
float del_thresh = 5 * d2r;

class algo_params_setup{
    protected: 
    float Time_period,field_param, theta_l_ini ;


    public:
    void setup_algo_params(){
        ros::Rate setup_rate(5);
        
        while(!ros::param::has("Time_period")){
            setup_rate.sleep();
            ROS_INFO("Algorithm node is waiting for parameter setup!");
        }

        if(ros::param::get("/Time_period",Time_period)){
            ROS_INFO("Time period of circular loop %f ",Time_period);
        }else{
            ROS_WARN("Time Period parameter not found! ");
        }

        if(ros::param::get("/field_param",field_param)){
            ROS_INFO("Time period of circular loop %f ",field_param);
        }else{
            ROS_WARN("Time Period parameter not found! ");
        }

        if(ros::param::get("/theta_l_ini",theta_l_ini)){
            theta_l_ini *= d2r;
            ROS_INFO("Time period of circular loop %f ",theta_l_ini);
        }else{
            ROS_WARN("theta_l_ini parameter not found! ");
        }
    }
};


class MTS_algo : public rviz_markers, public algo_params_setup {
    private:
        float m[3];
        size_t index{}, index_flag, front, back, mid;
        array<float, sample_N> buf{}; 
        std_msgs::Bool isReached;

        ros::NodeHandle* nh;
        std_srvs::Empty take_off_msg;
        src_loc::GetSensorMeasurement sensor_measurement;
        src_loc::DoMTSTurnSwitch MTS_Switch_srv;
        std_msgs::Float32 sensor_info_sub_msg;
        src_loc::TerminateAlgo terminate_message;

        ros::ServiceClient take_off_srvClient;
        ros::ServiceClient sensor_info;
        ros::ServiceClient mts_switch_client;
        ros::Subscriber sensor_info_sub;
        ros::Subscriber terminate_algo_sub;
        bool term_flag = false;

    
    public:
        void sensor_sub_cb(const std_msgs::Float32 &msg){
            sensor_info_sub_msg = msg;
        }
        void terminate_cb(const std_msgs::Bool::ConstPtr &msg){
            isReached = *msg;
            term_flag = isReached.data;
        } 
       void update_measurement(float new_m){
        buf[index] = new_m;
        index = (index + 1) % sample_N;
        index_flag++;
        // m[0] = m[1];
        // m[1] = m[2];
        // m[2] = new_m; 
       } 
       bool Check_MTS(){
        if(index_flag > sample_N){
            back = index;
            front = (back + sample_N - 1) % sample_N;
            mid = (back + sample_N / 2) % sample_N;
            // std::cout << back << " " << mid << " " << front << "\n" ; 
            // std::cout << buf[back] << " " << buf[mid] << " " << buf[front] << "\n" ; 
            if (buf[mid] > buf[back] && buf[mid] > buf[front]){ 
                // std::cout << back << " " << mid << " " << front << "\n" ; 
                // std::cout << buf[back] << " " << buf[mid] << " " << buf[front] << "\n" ; 
                buf.fill(999999); 
                index = 0;
                index_flag = 0;
                return true;
                }
            else return false; 
        }else return false;
        // if(m[0] < m[1] && m[1] > m[2]) return true;
        // else return false;
       }
       MTS_algo(){

       }
       void MTS_Algo_run(ros::NodeHandle* _nh){
            nh = _nh;
            setup_algo_params();
            update__marker_nh(_nh);
            init_markers();
            
            take_off_srvClient = nh->serviceClient<std_srvs::Empty>("take_off_trig");
            sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
            mts_switch_client = nh->serviceClient<src_loc::DoMTSTurnSwitch>("MTSTurnSwitch");
            sensor_info_sub = nh->subscribe("sensor_avg_info", 1 ,&MTS_algo::sensor_sub_cb, this);
            terminate_algo_sub = nh->subscribe("TerminateAlg", 1 ,&MTS_algo::terminate_cb, this);

            if(take_off_srvClient.waitForExistence()){
                ros::Rate rate(10);
                for(int i = 0; i<sample_N*5 ;i++){
                    sensor_info.call(sensor_measurement);
                    update_measurement(sensor_measurement.response.Measurement);
                    rate.sleep();
                }
                // ros::Duration 
                ros::Time last_switch = ros::Time::now();
                while(ros::ok() && !term_flag){

                    // if(isReached.data) break;

                    if( (Check_MTS() &&  ros::Time::now()-last_switch > ros::Duration(Time_period/10))){
                        // ROS_INFO("mts check passed");
                        mts_switch_client.call(MTS_Switch_srv);
                        add_point_switch(currentPoint);
                        //if(mts_switch_client.call(MTS_Switch_srv)){
                        //    ROS_INFO("Turn Switch Commanded. \n");
                        //} 
                        last_switch = ros::Time::now();
                    }
                     
                    add_point_linestrip(currentPoint);
                    sensor_info.call(sensor_measurement);
                    update_measurement(sensor_measurement.response.Measurement);
                    ros::spinOnce();
                    rate.sleep();
                }
            }
       }
    
};

class GDTS_algo : public rviz_markers, public algo_params_setup {

    protected:
    ros::NodeHandle* nh;
    std_srvs::Empty take_off_msg;
    src_loc::GetSensorMeasurement sensor_measurement;
    src_loc::DoMTSTurnSwitch MTS_Switch_srv;
    std_msgs::Bool isReached;

    ros::Subscriber TerminateAlgoSub;
    Vector3f LSM_res;
    Vector2f grad, startPos, currentPos, p, endPos;
    ros::Subscriber mav_pose_sub; 
    geometry_msgs::PoseStamped mav_pose;
    float xp, yp, zp, psi;
    ros::ServiceClient sensor_info, mts_switch_client, take_off_srvClient;
    size_t idx{};
    Matrix<float, dataPts, 3> A{};
    Matrix<float, dataPts, 1> B{};
    bool isPosAvailable = false, isFirstLoop = true;
    ros::Time loop_start;
    ros::Duration loop_duration;
    bool term_flag = false;
    int grad_est = 1;

    public:

    void terminate_cb(const std_msgs::Bool::ConstPtr &msg){
        isReached = *msg;
        term_flag = isReached.data;
    }

    void mav_pose_cb(const geometry_msgs::PoseStamped& msg){
        mav_pose = msg;
        // get pose of the mav convert it into roll pitch and yaw.
        float linearposx=msg.pose.position.x;
        float linearposy=msg.pose.position.y;

        xp = linearposx;
        yp = linearposy;

        currentPos[0] = xp;
        currentPos[1] = yp;
        // ROS_INFO("x %f, y %f", currentPos[0], currentPos[1]);
        // std::cout << "Pos cb : " << xp << " " << yp << "\n";
        if(!isPosAvailable) startPos = currentPos;
        isPosAvailable = true;
    }

    void update_data(){
       A.row(idx) << xp, yp, 1; 
       sensor_info.call(sensor_measurement);
       B.row(idx) << sensor_measurement.response.Measurement;
       idx++;
       idx = idx % dataPts;
    }

    bool isAligned(){
        p =  currentPos - startPos;
        float theta = acos( p.dot(grad) / (p.norm() * grad.norm() )); 
        // std::cout << "p (" << p[0] << ","  << p[1] << ") "<< " grad ("<< grad[0] << ","<< grad[1] << ")\n";
        // std::cout << "theta : " << theta * 180/M_PI << "\n"; //<< " p-norm : " << p.norm() << " grad.norm : " << grad.norm() << "\n";
        return theta < del_thresh;
    }

    float LSM(){
        MatrixXf _A = A.block(0,0,idx-1,3);
        MatrixXf _B = B.block(0,0,idx-1,1);
        VectorXf xe = VectorXf::Ones(idx) * endPos[0];
        _A.block(0,0,idx-1,1).col(0) -= xe;
        VectorXf ye = VectorXf::Ones(idx) * endPos[1];
        _A.block(0,1,idx-1,1).col(0) -= ye;

        // std::cout << "grad " << grad << "\n";
        // std::cout << "A  : " << _A << "\t" <<  " B : " << _B << "\n";

        LSM_res = ((_A.transpose()*_A).inverse())*_A.transpose()*_B;
        grad[0] = LSM_res[0]; 
        grad[1] = LSM_res[1];

        double theta_l = atan2(grad[1], grad[0]);
        return theta_l;
    }

    float FDM(){
        MatrixXf _A = A.block(0,0,idx-1,3);
        MatrixXf _B = B.block(0,0,idx-1,1);
        size_t min_idx, max_idx;
        float x_min, y_min, x_max, y_max, min_sig = 1e6, max_sig = -1e6;
        for (size_t i=0; i<idx-1;i++){
            // std::cout << "signal " << _B(i) << "\n";
            if ( _B(i) > max_sig){
                max_idx = i;
                max_sig = _B(i);
            }
            if ( _B(i) < min_sig){ 
                min_idx = i;
                min_sig = _B(i);    
            } 
        }
        x_min = _A.row(min_idx)[0];
        y_min = _A.row(min_idx)[1];
        x_max = _A.row(max_idx)[0];
        y_max = _A.row(max_idx)[1];
        
        // std::cout << "min_idx: " << min_idx << " max_idx: " << max_idx << "\n";
        // std::cout << "x_min: " << x_min<< " y_min: " << y_min << "\n";
        // std::cout << "x_max: " << x_max<< " y_max: " << y_max << "\n";
        grad[0] = x_max - x_min;
        grad[1] = y_max - y_min;

        double theta_l = atan2(grad[1], grad[0]);
        std::cout << "theta_l : " << theta_l << "\n";
        return theta_l;
    }

    GDTS_algo(){}

    GDTS_algo(ros::NodeHandle* _nh){
        nh = _nh;
        update__marker_nh(_nh);
        sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
        mts_switch_client = nh->serviceClient<src_loc::DoMTSTurnSwitch>("MTSTurnSwitch");
        take_off_srvClient = nh->serviceClient<std_srvs::Empty>("take_off_trig");
        TerminateAlgoSub = nh->subscribe("TerminateAlg", 1, &GDTS_algo::terminate_cb, this);
        
        if(ros::param::get("/grad_est", grad_est)){
            ROS_INFO("grad_est = %d", grad_est);
        }else{
            ROS_ERROR("grad_est parameter doesn't exists!!");
        }
        ros::spinOnce;
     };

    
    void switch_procedure(){
        std::cout << "Aligned : " << "\n";

        ros::spinOnce();
        endPos = currentPos;

        double theta_l;

        if (grad_est == 1){
           theta_l = LSM();
        }
        else if(grad_est == 2){
            theta_l = FDM();
        } 

        MTS_Switch_srv.request.theta_l = theta_l; 
                    
        idx = 0;
        A.setZero();
        B.setZero();
        startPos = currentPos;
        switch_point = currentPoint;

        mts_switch_client.call(MTS_Switch_srv);
        add_point_switch(currentPoint);
        update_grad_Arrow(theta_l);
        loop_start = ros::Time::now();
        ros::spinOnce;
    }

    void run_GDTS_algo(){
        setup_algo_params();
        ros::Rate rate(20);
        grad[0] = -1;
        grad[1] = -1; 
        if(take_off_srvClient.waitForExistence()){

            mav_pose_sub = nh->subscribe("/mavros/local_position/pose", 1, &GDTS_algo::mav_pose_cb, this);
            init_markers();
    
            // for(int i = 0; i<10 ;i++){
            //      update_data();
            // }

            std::cout << "current Pos : " << currentPos[0] << " "  << currentPos[1] << "\n";
            startPos = currentPos;
            switch_point = currentPoint;
            loop_start = ros::Time::now();
            loop_duration = ros::Duration(0.8*Time_period);
            ros::Duration first_loop_duration(0.2*Time_period);

            // idx = 0;
            // A.setZero();
            // B.setZero();

            // update_data();
            // ros::spinOnce;
            // startPos = currentPos;
            // switch_point = currentPoint;
            // for(int i = 0; i<6 ;i++){
            //     ros::spinOnce();
            //      update_data();
            // }
            idx = 0;
            A.setZero();
            B.setZero();

            update_data();
            ros::spinOnce;
            startPos = currentPos;
            switch_point = currentPoint;
            for(int i = 0; i<2 ;i++){
                ros::spinOnce();
                 update_data();
                add_point_linestrip(currentPoint);
            }
            ros::Time last_switch = ros::Time::now();
            ros::Duration half_period(Time_period*0.6);
            while (ros::ok() && !term_flag )
            {
                // if(isAligned()) std::cout << "its aligned " << isPosAvailable << " idx " << idx << std::endl;
                // ros::Time::now() - loop_start > first_loop_duration
                // std::cout << idx << "\n";
                if(isFirstLoop && isPosAvailable && idx > 4){
                    switch_procedure();      
                    isFirstLoop = false;
                    std::cout << "first loop done \n"; 
                }

                if (field_param == 3){
                    if(  (isAligned() && isPosAvailable && idx > 3) 
                            || (ros::Time::now() - last_switch   > half_period)){
                        // std::cout << "aligned" << "\n";
                        last_switch = ros::Time::now();
                        switch_procedure();      
                    }
                }else{ 
                    if(  (isAligned() && isPosAvailable && idx > 30) 
                            || (ros::Time::now() - last_switch   > half_period)){
                        // std::cout << "aligned" << "\n";
                        last_switch = ros::Time::now();
                        switch_procedure();      
                    }
                }
                // ROS_INFO("%f half_period , %f last_switch \n", half_period.toSec(), (last_switch - ros::Time::now()).toSec());
                // update_pos_Arrow();                
                // sensor_info.call(sensor_measurement);
                add_point_linestrip(currentPoint); 
                update_data();
                ros::spinOnce();
                rate.sleep();
            }
        }
    };  
};

class gamts_grad_aug: public GDTS_algo {
    protected:
    // ros::NodeHandle* nh;
    size_t index{}, index_flag, front, back, mid;
    array<float, sample_N> buf{}; 
    float theta_l = theta_l_ini;
    public:

        void update_data(){
            A.row(idx) << xp, yp, 1; 
            sensor_info.call(sensor_measurement);
            float new_m = sensor_measurement.response.Measurement;
            B.row(idx) << new_m;
            idx++;
            idx = idx % dataPts;

            // mts buffer update
            buf[index] = new_m;
            index = (index + 1) % sample_N;
            index_flag++;
        }

        bool Check_MTS(){
         if(index_flag > sample_N){
             back = index;
             front = (back + sample_N - 1) % sample_N;
             mid = (back + sample_N / 2) % sample_N;
             // std::cout << back << " " << mid << " " << front << "\n" ; 
             // std::cout << buf[back] << " " << buf[mid] << " " << buf[front] << "\n" ; 
             if (buf[mid] > buf[back] && buf[mid] > buf[front]){ 
                 // std::cout << back << " " << mid << " " << front << "\n" ; 
                 // std::cout << buf[back] << " " << buf[mid] << " " << buf[front] << "\n" ; 
                 buf.fill(999999); 
                 index = 0;
                 index_flag = 0;
                 return true;
                 }
             else return false; 
         }else return false;
        }


    void switch_procedure(){
        std::cout << "Switching : " << "\n";

        ros::spinOnce();

        // double theta_l;

        if (grad_est == 1){
           theta_l = LSM();
        }
        else if(grad_est == 2){
            endPos = currentPos;
            Vector2f diffPos = endPos - startPos;
            theta_l = atan2(diffPos[1],diffPos[0]);
            theta_l =  std::fmod((theta_l + 2*M_PI), (2*M_PI));
        } 

        endPos = currentPos;
        MTS_Switch_srv.request.theta_l = theta_l; 
          
        idx = 0;
        A.setZero();
        B.setZero();
        startPos = currentPos;
        switch_point = currentPoint;

        mts_switch_client.call(MTS_Switch_srv);
        add_point_switch(currentPoint);
        update_grad_Arrow(theta_l);
        loop_start = ros::Time::now();
        ros::spinOnce;
    }

        gamts_grad_aug(ros::NodeHandle* _nh){
            nh = _nh;
            update__marker_nh(_nh);
            sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
            mts_switch_client = nh->serviceClient<src_loc::DoMTSTurnSwitch>("MTSTurnSwitch");
            take_off_srvClient = nh->serviceClient<std_srvs::Empty>("take_off_trig");
            TerminateAlgoSub = nh->subscribe("TerminateAlg", 1, &GDTS_algo::terminate_cb, dynamic_cast<GDTS_algo*>(this) );

            if(ros::param::get("/grad_est", grad_est)){
                ROS_INFO("grad_est = %d", grad_est);
            }else{
                ROS_ERROR("grad_est parameter doesn't exists!!");
            }
            ros::spinOnce;
        }

        void run_gamts_grad_aug(){
            setup_algo_params();
            ros::Rate rate(20);
            if(take_off_srvClient.waitForExistence()){

                mav_pose_sub = nh->subscribe("/mavros/local_position/pose", 1, &GDTS_algo::mav_pose_cb, dynamic_cast<GDTS_algo*>(this));
                init_markers();
    
                // std::cout << "current Pos : " << currentPos[0] << " "  << currentPos[1] << "\n";
                startPos = currentPos;
                switch_point = currentPoint;
                loop_start = ros::Time::now();
                ros::Duration loop_duration(0.8*Time_period);
                ros::Duration first_loop_duration(0.2*Time_period);

                idx = 0;
                A.setZero();
                B.setZero();

                update_data();
                ros::spinOnce;
                startPos = currentPos;
                switch_point = currentPoint;
                for(int i = 0; i<5 ;i++){
                    ros::spinOnce();
                    update_data();
                    add_point_linestrip(currentPoint);
                }
                ros::Time last_switch = ros::Time::now();
                ros::Duration half_period(Time_period/2);
                while (ros::ok() && !term_flag )
                {
                    // if(isAligned()) std::cout << "its aligned " << isPosAvailable << " idx " << idx << std::endl;
                    // ros::Time::now() - loop_start > first_loop_duration
                    // std::cout << idx << "\n";
                    // if(isFirstLoop && isPosAvailable && idx > 4 ){
                    //     switch_procedure();      
                    //     isFirstLoop = false;
                    //     std::cout << "first loop done \n"; 
                    // }

                    if (field_param == 3){
                        if( Check_MTS() && isPosAvailable && idx > 4) {
                            // std::cout << "aligned" << "\n";
                            last_switch = ros::Time::now();
                            switch_procedure();      
                        }
                    }else{ 
                        if( Check_MTS() && isPosAvailable && idx > 30){
                            // std::cout << "aligned" << "\n";
                            last_switch = ros::Time::now();
                            switch_procedure();      
                        }
                    }
                    // ROS_INFO("%f half_period , %f last_switch \n", half_period.toSec(), (last_switch - ros::Time::now()).toSec());
                    // update_pos_Arrow();                
                    // sensor_info.call(sensor_measurement);
                    add_point_linestrip(currentPoint); 
                    update_data();
                    ros::spinOnce();
                    rate.sleep();
                }
            }
        }

};

class TRM_algo: public algo_params_setup {
    private:
    ros::NodeHandle* nh;

    std_srvs::Empty take_off_msg;
    src_loc::GetSensorMeasurement sensor_measurement;
    std_msgs::Float32 sensor_info_sub_msg;
    src_loc::TerminateAlgo terminate_message;

    ros::ServiceServer GetNextPositionSrv;


    ros::ServiceClient take_off_srvClient;
    ros::ServiceClient sensor_info;
    ros::Subscriber sensor_info_sub;
    ros::Subscriber terminate_algo_sub;

    ros::Subscriber mav_pose_sub; 
    geometry_msgs::PoseStamped mav_pose;

    Vector2f grad, currentPos, startpos;
    float xp, yp;
    std_msgs::Bool isReached;
    float del_t;
    bool isFirstLoop = true;
    // const int samples = 10;
    int sample_idx = 0;

    bool term_flag = false;

    float root2 = sqrt(2);
    float root2_inv = 1/sqrt(2);

    Eigen::Matrix<float, samples, 5> A{}; 
    Eigen::Matrix<float, 1, 5> f{};
    Eigen::Matrix<float, samples, 1> b{};
    Eigen::Matrix<float, samples, 2> p{};

    public:
        void sensor_sub_cb(const std_msgs::Float32 &msg){
            sensor_info_sub_msg = msg;
        }

        void terminate_cb(const std_msgs::Bool::ConstPtr &msg){
            isReached = *msg;
            term_flag = isReached.data;
        } 

        void update_measurement(){
            sensor_info.call(sensor_measurement); 
            float measurement = sensor_measurement.response.Measurement;
            
        } 

        Eigen::Vector2f compute_next_point(){

        }
        
        bool compute_next_point_srv_cb(src_loc::GetTRMNextPos::Request& req, src_loc::GetTRMNextPos::Response& res){
            isFirstLoop = false;
            Eigen::Vector2f next_point = compute_next_point();
            res.pos_x = next_point[0]; 
            res.pos_y = next_point[1]; 
            return true; 
        }

        void mav_pose_cb(const geometry_msgs::PoseStamped& msg){
            mav_pose = msg;
            // get pose of the mav convert it into roll pitch and yaw.
            float linearposx = msg.pose.position.x;
            float linearposy = msg.pose.position.y;

            xp = linearposx;
            yp = linearposy;

            currentPos[0] = xp;
            currentPos[1] = yp;
            // ROS_INFO("x %f, y %f", currentPos[0], currentPos[1]);
            // std::cout << "Pos cb : " << xp << " " << yp << "\n";
            // if(!isPosAvailable) startPos = currentPos;
            // isPosAvailable = true;
    }

    TRM_algo(){}

    TRM_algo(ros::NodeHandle* _nh){
        nh = _nh;
        sensor_info = nh->serviceClient<src_loc::GetSensorMeasurement>("GetSensorMeasurement");
        GetNextPositionSrv = nh->advertiseService("GetTRMNextPositionSrv", &TRM_algo::compute_next_point_srv_cb, this);
        take_off_srvClient = nh->serviceClient<std_srvs::Empty>("take_off_trig");
        terminate_algo_sub = nh->subscribe("TerminateAlg", 1, &TRM_algo::terminate_cb, this);

        setup_algo_params();
        ros::spinOnce();
    }
       void run_TRM_algo(){
        ros::Rate rate(20);
        if(take_off_srvClient.waitForExistence()){

            mav_pose_sub = nh->subscribe("/mavros/local_position/pose", 1, &TRM_algo::mav_pose_cb, this);

            while(ros::ok() && !term_flag){
                if(!isFirstLoop){
                    
                }
                ros::spinOnce();
                rate.sleep();
            }
        }

       }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "MTS_Algo");
    // ros::param::get("/sample", sample_N);// ::get("/sample_N", sample_N);

    ros::NodeHandle nh;
    int algo_num;
    if(ros::param::get("/algo_num", algo_num)){
        ROS_INFO("algo_num = %d", algo_num);
    }else{
        ROS_ERROR("algo_num parameter doesn't exists!!");
    }
   
    if (algo_num == 1){
        MTS_algo mtsAlgorithm;
        mtsAlgorithm.MTS_Algo_run(&nh);
    }
    else if ( algo_num == 2){
        gamts_grad_aug gamtsAlgorithm(&nh);
        gamtsAlgorithm.run_gamts_grad_aug();
    }
    else if (algo_num == 3){
        GDTS_algo gdtsAlgorithm = GDTS_algo(&nh);
        gdtsAlgorithm.run_GDTS_algo();
    }
    else if (algo_num == 4){
        return 0;
    }
    return 0;
}
