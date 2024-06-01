
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
#include <src_loc/GetSensorMeasurement.h>
#include <src_loc/DoMTSTurnSwitch.h>
#include <src_loc/TerminateAlgo.h>

using std::array;
using namespace Eigen;
constexpr int sample_N = 3;
constexpr int dataPts = 500;
constexpr float r2d = 180/M_PI;
constexpr float d2r = M_PI/180;
float del_thresh = 5 * d2r;

class algo_params_setup{
    protected: 
    float Time_period;

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
    }
};

class rviz_markers{
    protected: 
        ros::Publisher line_marker_pub;
        ros::Publisher switch_marker_pub;
        ros::Publisher grad_Arrow_pub;
        ros::Publisher pos_Arrow_pub;
        ros::Subscriber mav_pose_sub; 
        ros::NodeHandle* marker_nh;
        geometry_msgs::PoseStamped mav_pose;
        float xp, yp, zp, psi;
        Vector2f startPos, currentPos;
        bool isPosAvailable = false;
        geometry_msgs::Point currentPoint, switch_point;
        visualization_msgs::Marker line_strip;
        visualization_msgs::Marker Switch_points;
        visualization_msgs::Marker grad_Arrow;
        visualization_msgs::Marker pos_Arrow;
        int gradArrowID = 100;

    public:
        void update__marker_nh(ros::NodeHandle* _nh){
            marker_nh = _nh;
        }
        void marker_mav_pose_cb(const geometry_msgs::PoseStamped& msg){
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
            if(!isPosAvailable) startPos = currentPos;
            isPosAvailable = true;
        }
        void init_markers(){
            line_marker_pub = marker_nh->advertise<visualization_msgs::Marker>("trej_marker", 10);
            switch_marker_pub = marker_nh->advertise<visualization_msgs::Marker>("Switch_marker", 10);
            grad_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("GradArrow",0);
            pos_Arrow_pub = marker_nh->advertise<visualization_msgs::Marker>("PosArrow", 10);

            mav_pose_sub = marker_nh->subscribe("/mavros/local_position/pose", 1, &rviz_markers::marker_mav_pose_cb, this);
        }
        void add_point_linestrip(geometry_msgs::Point Point){

            line_strip.header.frame_id = "my_frame";
            line_strip.ns = "marker_ns";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0; 

            line_strip.id = 0;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.2;

            line_strip.color.r = 0.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;            
            // std::cout << "point : " << Point.x << " " << Point.y << "\n";
            line_strip.header.stamp = ros::Time::now(); 
            line_strip.points.push_back(Point);
            line_marker_pub.publish(line_strip);
        }

        void add_point_switch(geometry_msgs::Point Point){

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
        void update_pos_Arrow(){
            pos_Arrow.header.frame_id = "my_frame";
            pos_Arrow.ns = "marker_ns";
            pos_Arrow.action = visualization_msgs::Marker::ADD;

            pos_Arrow.id = 5;
            pos_Arrow.type = visualization_msgs::Marker::ARROW;
            pos_Arrow.scale.x = 0.5;
            pos_Arrow.scale.y = 0.5;
            pos_Arrow.scale.z = 0.5;
            
            pos_Arrow.points[0] = mav_pose.pose.position;
            pos_Arrow.points[1] = switch_point;
            pos_Arrow.color.r = 1.0;
            pos_Arrow.color.g = 0.0;
            pos_Arrow.color.b = 0.0;
            pos_Arrow.color.a = 1.0;            

            pos_Arrow.header.stamp = ros::Time::now(); 
            pos_Arrow_pub.publish(pos_Arrow);
        }

    void update_grad_Arrow(double theta){
            grad_Arrow.header.frame_id = "my_frame";
            grad_Arrow.ns = "marker_ns";
            grad_Arrow.action = visualization_msgs::Marker::ADD;

            grad_Arrow.id = gradArrowID++;
            grad_Arrow.type = visualization_msgs::Marker::ARROW;
            grad_Arrow.scale.x = 5;
            grad_Arrow.scale.y = 0.2;
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
        src_loc::TerminateAlgo terminate_message;

        ros::ServiceClient take_off_srvClient;
        ros::ServiceClient sensor_info;
        ros::ServiceClient mts_switch_client;
        ros::Subscriber terminate_algo_sub;
        bool term_flag = false;

    
    public:
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

                    if( (Check_MTS() &&  ros::Time::now()-last_switch > ros::Duration(Time_period/10))  || (ros::Time::now()-last_switch > ros::Duration(Time_period/2))){
                        ROS_INFO("mts check passed");
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

            idx = 0;
            A.setZero();
            B.setZero();

            update_data();
            ros::spinOnce;
            startPos = currentPos;
            switch_point = currentPoint;
            for(int i = 0; i<6 ;i++){
                ros::spinOnce();
                 update_data();
            }
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
            while (ros::ok() && !term_flag )
            {
                // if(isAligned()) std::cout << "its aligned " << isPosAvailable << " idx " << idx << std::endl;
                // ros::Time::now() - loop_start > first_loop_duration
                // std::cout << idx << "\n";
                if(isFirstLoop && isPosAvailable && idx > 30 ){
                    switch_procedure();      
                    isFirstLoop = false;
                    std::cout << "first loop done \n"; 
                    std::cout << "checking cmake  \n"; 
                }
                ros::Duration half_period(Time_period/2);
                 
                if(  (isAligned() && isPosAvailable && idx > 30) 
                        || (ros::Time::now() - last_switch   > half_period)){
                    // std::cout << "aligned" << "\n";
                    last_switch = ros::Time::now();
                    switch_procedure();      
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

class gamts_grad_aug: public GDTS_algo, public MTS_algo{
    protected:
    ros::NodeHandle* nh;

    public:
    gamts_grad_aug(ros::NodeHandle* _nh){
        nh = _nh;
    }
    void run_gamts_grad_aug(){

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
    const int vars_no = 5;
    const int src_loc_no = 10;

    double vars[vars_no][2] = {{75.0, 100.0}, {100.0, 100.0}, {125.0, 100.0}, {100.0, 75.0}, {100.0, 125.0}};
    double source_locs[vars_no][src_loc_no][2];

    std::default_random_engine generator1;
    std::default_random_engine generator2;

    generator1.seed(1);
    generator2.seed(2);

    for(int i=0; i<5; i++){
        double sig_x = vars[i][0];
        double sig_y = vars[i][1];
        std::cout << "sig_x : " << sig_x << " sig_y" << sig_y << "\n";
        std::uniform_real_distribution<double> distribution_sig_x(1.5 * sig_x, 3*sig_x);
        std::uniform_real_distribution<double> distribution_sig_y(1.5 * sig_y, 3*sig_y);
        for(int j=0; j<src_loc_no; j++){
            float src_x = distribution_sig_x(generator1);
            float src_y = distribution_sig_y(generator2);
            
            source_locs[i][j][0] = src_x;
            source_locs[i][j][1] = src_y;

            std::cout << src_x << " " << src_y << "\n";
        }
        
    }

    for(float psi_0 = 0.0; psi_0 < 360; psi_0 += 12 ){
        ros::param::set("/yaw_ini", psi_0);
        for( int i=0; i<vars_no;i++){
            ros::param::set("/sx", vars[i][0]);
            ros::param::set("/sy", vars[i][1]);
            for(int j=0; j<src_loc_no;j++){
                ros::param::set("/pos_x_s", source_locs[i][j][0]);
                ros::param::set("/pos_y_s", source_locs[i][j][1]);
                MTS_algo mtsAlgorithm;
                mtsAlgorithm.MTS_Algo_run(&nh);
            }
        }
    }
   
    // if (algo_num == 1 || algo_num == 2){
    //     MTS_algo mtsAlgorithm;
    //     mtsAlgorithm.MTS_Algo_run(&nh);
    // }
    // else if (algo_num == 3){
    //     GDTS_algo gdtsAlgorithm = GDTS_algo(&nh);
    //     gdtsAlgorithm.run_GDTS_algo();
    // }
    return 0;
}
