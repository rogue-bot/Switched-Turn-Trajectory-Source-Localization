#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/Marker.h>
#include <src_loc/ziggurat.hpp>
#include <src_loc/GetSensorMeasurement.h>
// grip map doesn't work normal eigen package need to use eigen conversion.
// remember to add eigen_conversions in find_package cmake file.
// also add build and exec dependencies for eigen_conversions. 



using std::array;
using namespace Eigen;
using namespace grid_map;

constexpr int filter_N = 5;

class filter{
    public:
    float window_update(float in){
        sum += in - buf[index];
        buf[index] = in;
        index = (index + 1) % filter_N;
        return sum / filter_N;
    };

    private:
    float sum{};
    size_t index{}, fill_count{};
    array<float, filter_N> buf{};
};

class SourceMarker {
    private:
    ros::NodeHandle nh;
    visualization_msgs::Marker source_marker;
    visualization_msgs::Marker text_marker;
    ros::Publisher vis_pub; 
    ros::Publisher source_text_pub; 
    public:
    void update_nh(ros::NodeHandle* _nh){
        nh = *_nh;
    }
    void init(geometry_msgs::Point SourcePoint){
        vis_pub = nh.advertise<visualization_msgs::Marker>( "source_marker", 0);
        source_text_pub = nh.advertise<visualization_msgs::Marker>( "text_marker", 0);
        float height = 2;
        source_marker.header.frame_id = "my_frame";
        source_marker.header.stamp = ros::Time();
        source_marker.ns = "marker_ns";
        source_marker.id = 1;
        source_marker.type = visualization_msgs::Marker::CYLINDER;
        source_marker.action = visualization_msgs::Marker::ADD;
        source_marker.pose.position.x = SourcePoint.x;
        source_marker.pose.position.y = SourcePoint.y;
        source_marker.pose.position.z = SourcePoint.z + height/2;
        source_marker.pose.orientation.x = 0.0;
        source_marker.pose.orientation.y = 0.0;
        source_marker.pose.orientation.z = 0.0;
        source_marker.pose.orientation.w = 1.0;
        source_marker.scale.x = 3;
        source_marker.scale.y = 3;
        source_marker.scale.z = height;
        source_marker.color.a = 1.0; // Don't forget to set the alpha!
        source_marker.color.r = 0.0;
        source_marker.color.g = 1.0;
        source_marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        vis_pub.publish( source_marker );


        text_marker.header.frame_id = "my_frame";
        text_marker.header.stamp = ros::Time();
        text_marker.ns = "marker_ns";
        text_marker.id = 3;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = SourcePoint.x + 3;
        text_marker.pose.position.y = SourcePoint.y + 3;
        text_marker.pose.position.z = SourcePoint.z + height*3/2;
        float norm;
        norm = sqrt(pow(SourcePoint.x,2)+pow(SourcePoint.y,2)+ pow(SourcePoint.z,2));
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = -SourcePoint.x/norm;
        text_marker.pose.orientation.z = -SourcePoint.y/norm;
        text_marker.pose.orientation.w = -SourcePoint.y/norm;
        text_marker.scale.z = 16;
        text_marker.scale.y = 16;
        text_marker.scale.x = 16;
        text_marker.color.a = 1.0; // Don't forget to set the alpha!
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.text = "Source";
        source_text_pub.publish( text_marker );
    }
    void update(){
        vis_pub.publish( source_marker );
        source_text_pub.publish( text_marker );
    }
};

class field{
    private:
    float xp, yp, zp, xs, ys, zs;
    float field_strength;
    float sx, sy, sz, qx, qy, qz;
    float Qmax;
    int field_type, noise_flag;
    ros::NodeHandle* nh;
    nav_msgs::Odometry  odom;
    // std::default_random_engine generator;
    std::mt19937 generator;
    filter F;
    bool window_flag;
    int sensor_rate = 8;
    float turn_radius, Time_period, altitude;
    std_msgs::Bool termination_msg;
    std::vector<float> m_;
    ros::Publisher sensor_pub;
    std_msgs::Float32 sensor_data;
    ros::Publisher sensor_avg_pub;
    std_msgs::Float32 sensor_avg_data;

    public:
    void updateSensorParams(){
        // get Qmax
        if( ros::param::get("/Qmax", Qmax)){
            ROS_INFO("Qmax is retrived, Qmax = %f", Qmax);
        }else{
            ROS_WARN("Qmax parameter not found!");
        }
        // get source x pos
        if( ros::param::get("/pos_x_s", xs)){
            ROS_INFO("xs is retrived, xs = %f", xs);
        }else{
            ROS_WARN("xs parameter not found!");
        }
        // get source y pos
        if( ros::param::get("/pos_y_s", ys)){
            ROS_INFO("ys is retrived, ys = %f", ys);
        }else{
            ROS_WARN("ys parameter not found!");
        }
        // get source z pos
        if( ros::param::get("/pos_z_s", zs)){
            ROS_INFO("zs is retrived, zs = %f", zs);
        }else{
            ROS_WARN("zs parameter not found!");
        }
        // get field_param
        if( ros::param::get("/field_param", field_type)){
            ROS_INFO("field_type is retrived, field_type = %d", field_type);
        }else{
            ROS_WARN("field_type parameter not found!");
        }
        // get noise flag
        if( ros::param::get("/noise_flag", noise_flag)){
            ROS_INFO("noise_flag is retrived, noise_flag = %d", noise_flag);
        }else{
            ROS_WARN("noise_flag parameter not found!");
        }
        // get window flag 
        if( ros::param::get("/window_flag", window_flag)){
            ROS_INFO("window_flag is retrived, window_flag = %d", window_flag);
        }else{
            ROS_WARN("window_flag parameter not found!");
        }
        // get sigma_x 
        if( ros::param::get("/sx", sx)){
            ROS_INFO("sx is retrived, sx = %f", sx);
        }else{
            ROS_WARN("sx parameter not found!");
        }
        // get sigma_y 
        if( ros::param::get("/sy", sy)){
            ROS_INFO("sy is retrived, sy = %f", sy);
        }else{
            ROS_WARN("sy parameter not found!");
        }
        // get sigma_z
        if( ros::param::get("/sz", sz)){
            ROS_INFO("sz is retrived, sz = %f", sz);
        }else{
            ROS_WARN("sz parameter not found!");
        }
        // get sensor_rate
        if( ros::param::get("/sensor_rate", sensor_rate)){
            ROS_INFO("sensor_rate is retrived, sensor_rate = %d", sensor_rate);
        }else{
            ROS_WARN("sensor_rate parameter not found!");
        }
        // get turn radius
        if( ros::param::get("/turn_radius", turn_radius)){
            ROS_INFO("turn_radius is retrived, turn_radius = %f", turn_radius);
        }else{
            ROS_WARN("turn_radius parameter not found!");
        }
        // get Time period 
        if( ros::param::get("/Time_period", Time_period)){
            ROS_INFO("turn_radius is retrived, Time_period = %f", Time_period);
        }else{
            ROS_WARN("turn_radius parameter not found!");
        }
        // get altitude 
        if( ros::param::get("/altitude", altitude)){
            ROS_INFO("turn_radius is retrived, altitude = %f", altitude);
        }else{
            ROS_WARN("turn_radius parameter not found!");
        }

        // get m, magnetic moment vector 
        if( ros::param::get("/m", m_)){
            ROS_INFO("turn_radius is retrived, m= %f , %f, %f", m_[0], m_[1], m_[2]);
        }else{
            ROS_WARN("turn_radius parameter not found!");
        }
    }
    bool terminateAlgoCheck(){
        if (range() < turn_radius) return true;
        else return  false;
    }
    
    field(ros::NodeHandle* _nh){
        nh = _nh;

        updateSensorParams();
        SourceMarker sm;

        sm.update_nh(nh);
        ros::Publisher publisher = nh->advertise<grid_map_msgs::GridMap>("grid_map",10000, true);
        GridMap map({"elevation"});
        map.setFrameId("map");
        if (field_type == 3){
            map.setGeometry(Length(60, 60), 0.5, Position(xs,ys));
        }else map.setGeometry(Length(300, 300), 1, Position(xs,ys));

        ROS_INFO("Map has been created! \n");
        geometry_msgs::Point SourcePoint;
        SourcePoint.x = xs;
        SourcePoint.y = ys;
        SourcePoint.z = zs;
        sm.init(SourcePoint);

        ros::Subscriber mav_odom = nh->subscribe<nav_msgs::Odometry>("mavros/local_position/odom",2 , &field::pos_p_cb, this);
        ros::ServiceServer sensor_service =  nh->advertiseService("GetSensorMeasurement", &field::sensor_service_cb, this);

        sensor_pub = nh->advertise<std_msgs::Float32>("sensor_info",1);
        sensor_avg_pub = nh->advertise<std_msgs::Float32>("sensor_avg_info",1);

        ros::Publisher range_pub = nh->advertise<std_msgs::Float32>("range",1);
        std_msgs::Float32 range_msg;
        ros::Rate rate(sensor_rate);
        ros::Publisher terminateAlgoPub = nh->advertise<std_msgs::Bool>("TerminateAlg", 1);

        filter F;
    
        // update the map with scalar field values.
        for(GridMapIterator it(map); !it.isPastEnd(); ++it){
            Position position;
            map.getPosition(*it, position);
            map.at("elevation", *it) = scalar_field_dist(position.x(), position.y()); 
            // std::cout << "position : " << position.x() << " " << position.y() << ", value " << Scalar_Field.scalar_field_dist(position.x(), position.y()) << "\n";
        }
        while(ros::ok()){
            ros::Time time = ros::Time::now();
            // ROS_INFO("%f\n",Scalar_Field.guass_plume());

            range_msg.data = range();
            range_pub.publish(range_msg);

            // publish the scalar field map
            map.setTimestamp(time.toNSec());
            grid_map_msgs::GridMap message;
            GridMapRosConverter::toMessage(map, message);
            publisher.publish(message);
            sm.update();

            termination_msg.data = terminateAlgoCheck();
            terminateAlgoPub.publish(termination_msg); 

            ros::spinOnce();
            rate.sleep();
        }
    }
    
    field(ros::NodeHandle* _nh, Vector3f pos_s, int _field_param, int _noise_flag, bool _window_flag){
        nh = _nh;
        xs = pos_s[0];
        ys = pos_s[1];
        zs = pos_s[2];
        field_type = _field_param;
        noise_flag = _noise_flag;
        window_flag = _window_flag;
    }

    void pos_p_cb(const nav_msgs::Odometry::ConstPtr& _odom){
        odom = *_odom;
        xp = odom.pose.pose.position.x;
        yp = odom.pose.pose.position.y;
        zp = odom.pose.pose.position.z;
    }

    float guass_plume(){
        float _meas =  Qmax * exp( - powf( xs - xp , 2.0)/(2*powf(sx, 2.0))  
                                   - powf( ys - yp , 2.0)/(2*powf(sy, 2.0))  
                                   - 0*powf( zs - zp , 2.0)/(2*powf(sz, 2.0)) ); 
        if(noise_flag){
            cxx::ziggurat_normal_distribution<float> N{0, (Qmax - _meas) / Qmax};
            float noise = (Qmax - _meas) / Qmax * N(generator);
            return _meas + noise;
        }else{
            return _meas;
        }
    }

    float guass_plume(float _xp, float _yp){
        float _meas =  Qmax * exp( - powf( xs - _xp , 2.0)/(2*powf(sx, 2.0))  
                                   - powf( ys - _yp , 2.0)/(2*powf(sy, 2.0))  
                                   - 0*powf( zs - zp , 2.0)/(2*powf(sz, 2.0)) ); 
        if(noise_flag){
            cxx::ziggurat_normal_distribution<float> N{0, (Qmax - _meas) / Qmax};
            float noise = (Qmax - _meas) / Qmax * N(generator);
            return _meas + noise;
        }else{
            return _meas;
        }
    }

    float quad(){
        float _meas = Qmax - qx * powf(xs-xp, 2.0)- qy * powf(ys-yp, 2.0)- qz * powf(zs-zp, 2.0);
        
        if(noise_flag){
            cxx::ziggurat_normal_distribution<float> N{0, (Qmax - _meas) / Qmax};
            
            float noise = (Qmax - _meas) / Qmax * N(generator);
            return _meas + noise;
        }else{
            return _meas;
        }
    }

    float quad(float _xp, float _yp){
        float _meas = Qmax - qx * powf(xs- _xp, 2.0)- qy * powf(ys- _yp, 2.0)- qz * powf(zs-zp, 2.0);
        
        if(noise_flag){
            cxx::ziggurat_normal_distribution<float> N{0, (Qmax - _meas) / Qmax};
            float noise = (Qmax - _meas) / Qmax * N(generator);
            return _meas + noise;
        }else{
            return _meas;
        }
    }

    float arva_field(){
        Vector3d r_vec(xs-xp,ys-yp,zs-zp);
        Vector3d m;
        m[0] = m_[0];
        m[1] = m_[1];
        m[2] = m_[2];
        Matrix3d A;
        A << 2*pow(r_vec[0],2) - pow(r_vec[1],2) - pow(r_vec[2],2), 3*r_vec[0]*r_vec[1], 3*r_vec[0]*r_vec[2],
             3*r_vec[0]*r_vec[1], 2*pow(r_vec[1],2) - pow(r_vec[0],2) - pow(r_vec[2],2), 3*r_vec[1]*r_vec[2],
             3*r_vec[0]*r_vec[2], 3*r_vec[1]*r_vec[2], 2*pow(r_vec[2],2) - pow(r_vec[0],2) - pow(r_vec[1],2);

        
        float _meas =  (1/(4*M_PI*pow(r_vec.norm(),5)) * A * m).norm(); 
        
        if(noise_flag){
            cxx::ziggurat_normal_distribution<float> N{0, powf(Qmax,2.0) / powf(100.0,2.0)};
            float noise = N(generator);
            return _meas + noise;
        }else{
            return _meas;
        }
    }

    float arva_field(float _xp, float _yp){

        Vector3d r_vec(xs-_xp,ys-_yp,altitude);
        Vector3d m;
        m[0] = m_[0];
        m[1] = m_[1];
        m[2] = m_[2];
        Matrix3d A;
        A << 2*pow(r_vec[0],2) - pow(r_vec[1],2) - pow(r_vec[2],2),     3*r_vec[0]*r_vec[1],                                    3*r_vec[0]*r_vec[2],
             3*r_vec[0]*r_vec[1],                                       2*pow(r_vec[1],2) - pow(r_vec[0],2) - pow(r_vec[2],2),  3*r_vec[1]*r_vec[2],
             3*r_vec[0]*r_vec[2],                                       3*r_vec[1]*r_vec[2],                                    2*pow(r_vec[2],2) - pow(r_vec[0],2) - pow(r_vec[1],2);

        
        float _meas = 10000 *  (1/(4*M_PI*pow(r_vec.norm(),5)) * A * m).norm(); 
        
        if(noise_flag){
            cxx::ziggurat_normal_distribution<float> N{0, powf(Qmax,2.0) / powf(100.0,2.0)};
            float noise = N(generator);
            return _meas + noise;
        }else{
            return _meas;
        }
    }
    void update_gauss_field(float _sx, float _sy, float _sz, float _Qmax){
        sx = _sx;
        sy = _sy;
        sz = _sz;
        Qmax = _Qmax;
    }

    void update_quad_field(float _qx, float _qy, float _qz, float _Qmax){
        sx = _qx;
        sy = _qy;
        sz = _qz;
        Qmax = _Qmax;
    }

    float range(){
        return powf(powf(xp-xs,2) + powf(yp-ys,2), 0.5);
    }

    bool sensor_service_cb(src_loc::GetSensorMeasurement::Request& req, src_loc::GetSensorMeasurement::Response& res){
        // ROS_INFO("sensor service called");
        float  measurement;
        float window_measurement;
        if(field_type == 1) {
            measurement =  guass_plume();
        }
        else if (field_type == 2){
            measurement =  quad();
        }
        else if (field_type == 3){
            measurement =  arva_field();
        }
        
        window_measurement = F.window_update(measurement);

        if (window_flag) {
            res.Measurement = window_measurement;
        }else{
            res.Measurement = measurement;
        }

        sensor_data.data =  measurement;
        sensor_pub.publish(sensor_data);

        sensor_avg_data.data = window_measurement;
        sensor_avg_pub.publish(sensor_avg_data);
        return true;
    }

    float scalar_field_dist(float x, float y){
        if(field_type == 1) {
            return guass_plume(x, y);
        }
        else if (field_type == 2){
            return quad(x, y);
        }
        else{
            return arva_field(x, y);
        }
    }
};



int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_node");
    ros::NodeHandle nh;

    //// initialize scalar field parameters.
    // Vector3f pos_s;

    // pos_s[0] = -80.0;
    // pos_s[1] = -80.0;
    // pos_s[2] = 0;    
    // geometry_msgs::Point SourcePoint;
    // SourcePoint.x = pos_s[0];
    // SourcePoint.y = pos_s[1];
    // SourcePoint.z = pos_s[2];
    // float sx = 100;
    // float sy = 60;
    // float sz = 10;
    // float Qmax = 10e3;
    // int field_param = 1;
    // int noise_flag = 1;
    // bool window_flag = true;

    // field Scalar_Field = field(&nh, pos_s, field_param, noise_flag, window_flag);
    field Scalar_Field = field(&nh);
    // Scalar_Field.update_gauss_field(sx, sy, sz, Qmax);
    return 0;
}
