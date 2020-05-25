#include "tilted_control.h"
#include "Eigen/Dense"
#include "Eigen/QR"
#include "tf/tf.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

using namespace std;

//---Get params from ros parameter server-----
void load_param( string & p, string def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}

void load_param( double & p, double def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  cout << name << ": " << "\t" << p << endl;
}
//--------------------


class TiltedControl{
    public:
        TiltedControl();

        void run();
        void TiltMotors(double alpha);
        void TiltCheck(sensor_msgs::JointState tilt);
        void EvaluateJ();

    private:
        ros::NodeHandle _nh;
        ros::Publisher _motor_pub;
        ros::Subscriber _pose_sub;
        ros::Subscriber _vel_sub;
        ros::Subscriber _des_pose_sub;
        ros::Subscriber _des_vel_sub;
        ros::Subscriber _des_acc_sub;
        ros::Publisher _tilt_motor_0_pub;
        ros::Publisher _tilt_motor_1_pub;
        ros::Publisher _tilt_motor_2_pub;
        ros::Publisher _tilt_motor_3_pub;
        ros::Publisher _tilt_motor_4_pub;
        ros::Publisher _tilt_motor_5_pub;
        ros::Subscriber _tilt_motors_sub;

        Eigen::Vector3d _des_pos;
        Eigen::Matrix3d _des_att;
        Eigen::Vector3d _des_vel;
        Eigen::Vector3d _des_d_att;
        Eigen::Vector3d _des_acc;
        Eigen::Vector3d _des_dd_att;

        Eigen::Vector3d _mis_pos;
        Eigen::Matrix3d _mis_att;
        Eigen::Vector3d _mis_vel;
        Eigen::Vector3d _mis_d_att;

        std_msgs::Float64 _tilt[6];
        Eigen::Matrix<double, 6, 6> _J_inv, _J;

        double _Kpp, _Kdp, _Kip, _Kpr, _Kdr, _Kir;
        double _Kpr_z, _Kdr_z, _Kir_z;
        double _Kpp_z, _Kdp_z, _Kip_z;
        double _alpha;
        double _Km;
        double _Kf;
        double _L;
        double _Z;
        double _rate;
        double _f;
        
        void pose_cb( geometry_msgs::PoseStamped pose );
        void vel_cb( geometry_msgs::TwistStamped vel );
        void des_pose_cb( geometry_msgs::Pose pose);
        void des_vel_cb( geometry_msgs::Twist vel);
        void des_acc_cb( geometry_msgs::Twist acc);
        Eigen::Vector3d skew2vec(Eigen::Matrix3d mis_att, Eigen::Matrix3d des_att);

        bool _first_pose;
        bool _first_vel;
        bool _motors_tilted;


};

TiltedControl::TiltedControl(){

    string _pose_topic_name;
    string _vel_topic_name;
    string _motor_topic_name;
    string _des_pose_topic_name;
    string _des_vel_topic_name;
    string _des_acc_topic_name;
    string _tilt_motor_0_topic_name;
    string _tilt_motor_1_topic_name;
    string _tilt_motor_2_topic_name;
    string _tilt_motor_3_topic_name;
    string _tilt_motor_4_topic_name;
    string _tilt_motor_5_topic_name;
    string _tilt_motors_topic_name;
    load_param( _pose_topic_name, "/hexa_tilt/local_pose", "pose_topic_name");
    load_param( _vel_topic_name, "/hexa_tilt/local_vel", "vel_topic_name");
    load_param( _motor_topic_name, "/hexa_tilt/cmd/motor_vel", "motor_topic_name");
    load_param( _des_pose_topic_name, "/trajectory/pose", "des_pose_topic_name");
    load_param( _des_vel_topic_name, "/trajectory/vel", "des_vel_topic_name");
    load_param( _des_acc_topic_name, "/trajectory/acc", "des_acc_topic_name");
    load_param( _tilt_motor_0_topic_name, "/tilt/tilt_rotor_0_joint_controller/command", "tilt_motor_0_topic_name");
    load_param( _tilt_motor_1_topic_name, "/tilt/tilt_rotor_1_joint_controller/command", "tilt_motor_1_topic_name");
    load_param( _tilt_motor_2_topic_name, "/tilt/tilt_rotor_2_joint_controller/command", "tilt_motor_2_topic_name");
    load_param( _tilt_motor_3_topic_name, "/tilt/tilt_rotor_3_joint_controller/command", "tilt_motor_3_topic_name");
    load_param( _tilt_motor_4_topic_name, "/tilt/tilt_rotor_4_joint_controller/command", "tilt_motor_4_topic_name");
    load_param( _tilt_motor_5_topic_name, "/tilt/tilt_rotor_5_joint_controller/command", "tilt_motor_5_topic_name");
    load_param( _tilt_motors_topic_name, "/tilt/joint_states", "tilt_motors_topic_name");

    load_param( _Kpp, 1.0, "Kpp");
    load_param( _Kdp, 1.0, "Kdp");
    load_param( _Kip, 1.0, "Kip");
    load_param( _Kpp_z, 1.0, "Kpp_z");
    load_param( _Kdp_z, 1.0, "Kdp_z");
    load_param( _Kip_z, 1.0, "Kip_z");
    load_param( _Kpr, 1.0, "Kpr");
    load_param( _Kdr, 1.0, "Kdr");
    load_param( _Kir, 1.0, "Kir");
    load_param( _Kpr_z, 1.0, "Kpr_z");
    load_param( _Kdr_z, 1.0, "Kdr_z");
    load_param( _Kir_z, 1.0, "Kir_z");
    load_param( _alpha, 0.18, "alpha");
    load_param( _Km, 0.06*8.54858e-06, "Km");
    load_param( _Kf, 8.54858e-06, "Kf");
    load_param( _L, 0.21, "L_arm");
    load_param( _Z, 0.08, "Z_arm");
    load_param( _rate, 250, "rate");
    load_param( _f, -9.81, "gravity");

    _pose_sub = _nh.subscribe( _pose_topic_name.c_str(), 0, &TiltedControl::pose_cb, this);
    _vel_sub = _nh.subscribe( _vel_topic_name.c_str(), 0, &TiltedControl::vel_cb, this);
    _des_pose_sub = _nh.subscribe( _des_pose_topic_name.c_str(), 0, &TiltedControl::des_pose_cb, this);
    _des_vel_sub = _nh.subscribe( _des_vel_topic_name.c_str(), 0, &TiltedControl::des_vel_cb, this);
    _des_acc_sub = _nh.subscribe( _des_acc_topic_name.c_str(), 0, &TiltedControl::des_acc_cb, this);
    _motor_pub = _nh.advertise< std_msgs::Float32MultiArray > ( _motor_topic_name.c_str(), 0);
    _tilt_motors_sub = _nh.subscribe( _tilt_motors_topic_name.c_str(), 0, &TiltedControl::TiltCheck, this);

    //TO DO: fare un solo topic per i tilt
    _tilt_motor_0_pub = _nh.advertise< std_msgs::Float64 > (_tilt_motor_0_topic_name.c_str(), 0);
    _tilt_motor_1_pub = _nh.advertise< std_msgs::Float64 > (_tilt_motor_1_topic_name.c_str(), 0);
    _tilt_motor_2_pub = _nh.advertise< std_msgs::Float64 > (_tilt_motor_2_topic_name.c_str(), 0);
    _tilt_motor_3_pub = _nh.advertise< std_msgs::Float64 > (_tilt_motor_3_topic_name.c_str(), 0);
    _tilt_motor_4_pub = _nh.advertise< std_msgs::Float64 > (_tilt_motor_4_topic_name.c_str(), 0);
    _tilt_motor_5_pub = _nh.advertise< std_msgs::Float64 > (_tilt_motor_5_topic_name.c_str(), 0);


    _first_pose = _first_vel = false;
    _motors_tilted = false;

}

void TiltedControl::pose_cb( geometry_msgs::PoseStamped pose ){

    _mis_pos << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
    cout << _mis_pos.transpose() << endl;
    Eigen::Quaterniond q_eig( pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,  pose.pose.orientation.z);
    q_eig.normalize();
	_mis_att = q_eig.toRotationMatrix();
    _first_pose = true;
    cout << "mis_pos: " << _mis_pos.transpose() << endl;
    /*
    double roll, pitch, yaw;
    tf::Quaternion q( pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,  pose.pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    _mis_att << roll, pitch, yaw;
    */

}

void TiltedControl::vel_cb( geometry_msgs::TwistStamped vel ){

    _mis_vel << vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z; 
    _mis_d_att << vel.twist.angular.x, vel.twist.angular.y, vel.twist.angular.z;
    
    _first_vel = true;
}

void TiltedControl::des_pose_cb( geometry_msgs::Pose pose){

    _des_pos  << pose.position.x, pose.position.y, pose.position.z;
    Eigen::Quaterniond q_eig( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	_des_att = q_eig.toRotationMatrix();
    cout << "des pos: " << _des_pos.transpose() << endl;

}

void TiltedControl::des_vel_cb( geometry_msgs::Twist vel){
    _des_vel << vel.linear.x, vel.linear.y, vel.linear.z;
    _des_d_att << vel.angular.x, vel.angular.y, vel.angular.z;
}

void TiltedControl::des_acc_cb( geometry_msgs::Twist acc){
    _des_acc << acc.linear.x, acc.linear.y, acc.linear.z;
    _des_dd_att << acc.angular.x, acc.angular.y, acc.angular.z;
}

void TiltedControl::TiltMotors(double alpha){

    _tilt[0].data = alpha;
    _tilt[1].data = -1.0*alpha;
    _tilt[2].data = -1.0*alpha;
    _tilt[3].data = alpha;
    _tilt[4].data = -1.0*alpha;
    _tilt[5].data = alpha;

    _tilt_motor_0_pub.publish(_tilt[0].data);
    _tilt_motor_1_pub.publish(_tilt[1].data);
    _tilt_motor_2_pub.publish(_tilt[2].data);
    _tilt_motor_3_pub.publish(_tilt[3].data);
    _tilt_motor_4_pub.publish(_tilt[4].data);
    _tilt_motor_5_pub.publish(_tilt[5].data);
}

void TiltedControl::TiltCheck(sensor_msgs::JointState tilt){

    if( ( tilt.position[0] >= (_tilt[0].data - 0.005) || tilt.position[0] <= (_tilt[0].data + 0.005))  &&
        ( tilt.position[1] >= (_tilt[1].data - 0.005) || tilt.position[1] <= (_tilt[1].data + 0.005))  &&
        ( tilt.position[2] >= (_tilt[2].data - 0.005) || tilt.position[2] <= (_tilt[2].data + 0.005))  &&
        ( tilt.position[3] >= (_tilt[3].data - 0.005) || tilt.position[3] <= (_tilt[3].data + 0.005))  &&
        ( tilt.position[4] >= (_tilt[4].data - 0.005) || tilt.position[4] <= (_tilt[4].data + 0.005))  && 
        ( tilt.position[5] >= (_tilt[5].data - 0.005) || tilt.position[5] <= (_tilt[5].data + 0.005)) ){

            _motors_tilted = true;
        }
        else
        {
            _motors_tilted = false;
            cout << "Motors not tilted" << endl;
        }

}

Eigen::Vector3d TiltedControl::skew2vec(Eigen::Matrix3d mis_att, Eigen::Matrix3d des_att){

    Eigen::Vector3d eR;
    Eigen::Matrix3d eM;

    eM = des_att.transpose()*mis_att - mis_att.transpose()*des_att;
    eR << eM(2, 1), eM(0,2), eM(1, 0);
    
    cout << "a mis: \n" << mis_att << endl;
    cout << "a des: \n" << des_att << endl;
    //cout << "skew: \n" << eM << endl;
    //cout << "eR: \n" << eR.transpose() << endl;
    
    return eR;
}

void TiltedControl::EvaluateJ(){
    double a1, a2, a3, a4, a5, a6;
    double kf, km;
    double Rbw1_1, Rbw1_2, Rbw1_3;
    double Rbw2_1, Rbw2_2, Rbw2_3;
    double Rbw3_1, Rbw3_2, Rbw3_3;
    double alpha = _alpha;
    a1 = alpha;
    a2 = -1.0*alpha;
    a3 = -1.0*alpha;
    a4 = alpha;
    a5 = -1.0*alpha;
    a6 = alpha;

    kf = _Kf;
    km = _Km;
    Rbw1_1 = _mis_att(0,0); Rbw1_2 = _mis_att(0,1); Rbw1_3 = _mis_att(0,2);
    Rbw2_1 = _mis_att(1,0); Rbw2_2 = _mis_att(1,1); Rbw2_3 = _mis_att(1,2);
    Rbw3_1 = _mis_att(2,0); Rbw3_2 = _mis_att(2,1); Rbw3_3 = _mis_att(2,2);

    /*
    // matrice con ccw= 1 e cw=-1
    _J << 0.667*Rbw1_3*kf*cos(a1) - 0.333*Rbw1_1*kf*sin(a1) + 0.577*Rbw1_2*kf*sin(a1), 0.667*Rbw1_3*kf*cos(a2) + 0.333*Rbw1_1*kf*sin(a2) - 0.577*Rbw1_2*kf*sin(a2),
          0.667*Rbw1_3*kf*cos(a3) + 0.333*Rbw1_1*kf*sin(a3) + 0.577*Rbw1_2*kf*sin(a3), 0.667*Rbw1_3*kf*cos(a4) - 0.333*Rbw1_1*kf*sin(a4) - 0.577*Rbw1_2*kf*sin(a4), 0.667*Rbw1_3*kf*cos(a5) - 0.667*Rbw1_1*kf*sin(a5), 0.667*Rbw1_3*kf*cos(a6) + 0.667*Rbw1_1*kf*sin(a6),
          0.667*Rbw2_3*kf*cos(a1) - 0.333*Rbw2_1*kf*sin(a1) + 0.577*Rbw2_2*kf*sin(a1), 0.667*Rbw2_3*kf*cos(a2) + 0.333*Rbw2_1*kf*sin(a2) - 0.577*Rbw2_2*kf*sin(a2), 0.667*Rbw2_3*kf*cos(a3) + 0.333*Rbw2_1*kf*sin(a3) + 0.577*Rbw2_2*kf*sin(a3), 0.667*Rbw2_3*kf*cos(a4) - 0.333*Rbw2_1*kf*sin(a4) - 0.577*Rbw2_2*kf*sin(a4), 0.667*Rbw2_3*kf*cos(a5) - 0.667*Rbw2_1*kf*sin(a5), 0.667*Rbw2_3*kf*cos(a6) + 0.667*Rbw2_1*kf*sin(a6),
          0.667*Rbw3_3*kf*cos(a1) - 0.333*Rbw3_1*kf*sin(a1) + 0.577*Rbw3_2*kf*sin(a1), 0.667*Rbw3_3*kf*cos(a2) + 0.333*Rbw3_1*kf*sin(a2) - 0.577*Rbw3_2*kf*sin(a2), 0.667*Rbw3_3*kf*cos(a3) + 0.333*Rbw3_1*kf*sin(a3) + 0.577*Rbw3_2*kf*sin(a3), 0.667*Rbw3_3*kf*cos(a4) - 0.333*Rbw3_1*kf*sin(a4) - 0.577*Rbw3_2*kf*sin(a4), 0.667*Rbw3_3*kf*cos(a5) - 0.667*Rbw3_1*kf*sin(a5), 0.667*Rbw3_3*kf*cos(a6) + 0.667*Rbw3_1*kf*sin(a6),
          - 3.61*kf*cos(a1) - 2.38*kf*sin(a1) - 17.2*km*sin(a1), 3.61*kf*cos(a2) + 2.38*kf*sin(a2) - 17.2*km*sin(a2),
          3.61*kf*cos(a3) - 2.38*kf*sin(a3) - 17.2*km*sin(a3), 2.38*kf*sin(a4) - 3.61*kf*cos(a4) - 17.2*km*sin(a4),
          34.4*km*sin(a5) - 7.22*kf*cos(a5), 7.22*kf*cos(a6) + 34.4*km*sin(a6),
          6.25*kf*cos(a1) - 1.37*kf*sin(a1) + 29.8*km*sin(a1), 1.37*kf*sin(a2) - 6.25*kf*cos(a2) + 29.8*km*sin(a2),
          6.25*kf*cos(a3) + 1.37*kf*sin(a3) - 29.8*km*sin(a3), - 6.25*kf*cos(a4) - 1.37*kf*sin(a4) - 29.8*km*sin(a4),
          -2.75*kf*sin(a5), 2.75*kf*sin(a6),
          18.1*km*cos(a1) - 3.8*kf*sin(a1), - 18.1*km*cos(a2) - 3.8*kf*sin(a2),
          - 18.1*km*cos(a3) - 3.8*kf*sin(a3), 18.1*km*cos(a4) - 3.8*kf*sin(a4),
          - 18.1*km*cos(a5) - 3.8*kf*sin(a5), 18.1*km*cos(a6) - 3.8*kf*sin(a6);

    */
    //matrice con ccw=-1 e cw=1
    _J << 0.667*Rbw1_3*kf*cos(a1) - 0.333*Rbw1_1*kf*sin(a1) + 0.577*Rbw1_2*kf*sin(a1), 0.667*Rbw1_3*kf*cos(a2) + 0.333*Rbw1_1*kf*sin(a2) - 0.577*Rbw1_2*kf*sin(a2),
          0.667*Rbw1_3*kf*cos(a3) + 0.333*Rbw1_1*kf*sin(a3) + 0.577*Rbw1_2*kf*sin(a3), 0.667*Rbw1_3*kf*cos(a4) - 0.333*Rbw1_1*kf*sin(a4) - 0.577*Rbw1_2*kf*sin(a4), 0.667*Rbw1_3*kf*cos(a5) - 0.667*Rbw1_1*kf*sin(a5), 0.667*Rbw1_3*kf*cos(a6) + 0.667*Rbw1_1*kf*sin(a6),
          0.667*Rbw2_3*kf*cos(a1) - 0.333*Rbw2_1*kf*sin(a1) + 0.577*Rbw2_2*kf*sin(a1), 0.667*Rbw2_3*kf*cos(a2) + 0.333*Rbw2_1*kf*sin(a2) - 0.577*Rbw2_2*kf*sin(a2), 0.667*Rbw2_3*kf*cos(a3) + 0.333*Rbw2_1*kf*sin(a3) + 0.577*Rbw2_2*kf*sin(a3), 0.667*Rbw2_3*kf*cos(a4) - 0.333*Rbw2_1*kf*sin(a4) - 0.577*Rbw2_2*kf*sin(a4), 0.667*Rbw2_3*kf*cos(a5) - 0.667*Rbw2_1*kf*sin(a5), 0.667*Rbw2_3*kf*cos(a6) + 0.667*Rbw2_1*kf*sin(a6),
          0.667*Rbw3_3*kf*cos(a1) - 0.333*Rbw3_1*kf*sin(a1) + 0.577*Rbw3_2*kf*sin(a1), 0.667*Rbw3_3*kf*cos(a2) + 0.333*Rbw3_1*kf*sin(a2) - 0.577*Rbw3_2*kf*sin(a2), 0.667*Rbw3_3*kf*cos(a3) + 0.333*Rbw3_1*kf*sin(a3) + 0.577*Rbw3_2*kf*sin(a3), 0.667*Rbw3_3*kf*cos(a4) - 0.333*Rbw3_1*kf*sin(a4) - 0.577*Rbw3_2*kf*sin(a4), 0.667*Rbw3_3*kf*cos(a5) - 0.667*Rbw3_1*kf*sin(a5), 0.667*Rbw3_3*kf*cos(a6) + 0.667*Rbw3_1*kf*sin(a6),
          17.2*km*sin(a1) - 2.38*kf*sin(a1) - 3.61*kf*cos(a1), 3.61*kf*cos(a2) + 2.38*kf*sin(a2) + 17.2*km*sin(a2), 
          3.61*kf*cos(a3) - 2.38*kf*sin(a3) + 17.2*km*sin(a3), 2.38*kf*sin(a4) - 3.61*kf*cos(a4) + 17.2*km*sin(a4),
          - 7.22*kf*cos(a5) - 34.4*km*sin(a5), 7.22*kf*cos(a6) - 34.4*km*sin(a6),
          6.25*kf*cos(a1) - 1.37*kf*sin(a1) - 29.8*km*sin(a1), 1.37*kf*sin(a2) - 6.25*kf*cos(a2) - 29.8*km*sin(a2),
          6.25*kf*cos(a3) + 1.37*kf*sin(a3) + 29.8*km*sin(a3), 29.8*km*sin(a4) - 1.37*kf*sin(a4) - 6.25*kf*cos(a4),
          -2.75*kf*sin(a5), 2.75*kf*sin(a6),
          - 18.1*km*cos(a1) - 3.8*kf*sin(a1), 18.1*km*cos(a2) - 3.8*kf*sin(a2),
          18.1*km*cos(a3) - 3.8*kf*sin(a3), - 18.1*km*cos(a4) - 3.8*kf*sin(a4),
          18.1*km*cos(a5) - 3.8*kf*sin(a5), - 18.1*km*cos(a6) - 3.8*kf*sin(a6);
    
    _J_inv = _J.inverse();
}

void TiltedControl::run(){

    ros::Rate rate(_rate);

    //Initialization
    if( ros::ok() )
        ros::spinOnce();
    
    Eigen::Vector3d e_pos, e_vel, e_att, e_d_att, e_p_int, e_att_int;
    Eigen::Vector3d ni_pos, ni_att;
    Eigen::Matrix<double, 6, 1> control_output, ni, f;

    f << 0.0, 0.0, -1.0*_f, 0.0, 0.0, 0.0;
    e_p_int = Eigen::Vector3d::Zero();
    e_att_int = Eigen::Vector3d::Zero();
    e_pos = Eigen::Vector3d::Zero();
    e_att = Eigen::Vector3d::Zero();
    e_vel = Eigen::Vector3d::Zero();
    e_d_att = Eigen::Vector3d::Zero();

    std_msgs::Float32MultiArray rotor_speed;
    rotor_speed.data.resize(6);

    bool nan = false;

    while ( ros::ok() ){
    
        if(!_motors_tilted)
            TiltMotors(_alpha);
        
        e_pos = _mis_pos - _des_pos;
        e_vel = _mis_vel - _des_vel;
        e_att = 0.5*skew2vec(_mis_att, _des_att);
        e_d_att = _mis_d_att - _mis_att.transpose() * _des_att * _des_d_att;
        e_p_int +=  e_pos* (1.0/float(_rate));
        e_att_int += e_att* (1.0/float(_rate));
        
        cout << "e_p: " << e_pos.transpose() << endl;
        cout << "e_v: " << e_vel.transpose() << endl;
        cout << "e_a: " << e_att.transpose() << endl;
        cout << "e_d_a: " << e_d_att.transpose() << endl;
        cout << "e_p_int: " << e_p_int.transpose() << endl;
        cout << "e_a_int: " << e_att_int.transpose() << endl;
        
        ni_att(0) = _des_dd_att(0) - _Kdr * e_d_att(0) - _Kpr * e_att(0) - _Kir * e_att_int(0); 
        ni_att(1) = _des_dd_att(1) - _Kdr * e_d_att(1) - _Kpr * e_att(1) - _Kir * e_att_int(1); 
        ni_att(2) = _des_dd_att(2) - _Kdr_z * e_d_att(2) - _Kpr_z * e_att(2) - _Kir_z * e_att_int(2); 

        //ni_att << 1.0*ni_att(0), 1.0*ni_att(1), ni_att(2);
        ni_pos(0) = _des_acc(0) - _Kdp * e_vel(0) - _Kpp * e_pos(0) - _Kip * e_p_int(0);
        ni_pos(1) = _des_acc(1) - _Kdp * e_vel(1) - _Kpp * e_pos(1) - _Kip * e_p_int(1);
        ni_pos(2) = _des_acc(2) - _Kdp_z * e_vel(2) - _Kpp_z * e_pos(2) - _Kip_z * e_p_int(2);
        ni << ni_pos, ni_att;
        
        cout << "ni_pos: " << ni_pos.transpose() << endl;
        cout << "ni_att: " << ni_att.transpose() << endl;
        cout << "ni: " << ni.transpose() << endl;
        
        EvaluateJ();

        control_output = _J_inv * (-f + ni);
        
        cout << "J inv: \n" << _J_inv << endl;
        cout << "f: " << f.transpose() << endl;
        //control_output = control_output.cwiseMax(Eigen::VectorXd::Zero(6));
        cout << "control output: " << control_output.transpose() << endl;
        
        cout << "rotor speed: ";
        for(int i=0; i<6; i++){
            rotor_speed.data[i] = sqrt(fabs(control_output(i)));
            cout << rotor_speed.data[i] << "  ";
            if(isnan(rotor_speed.data[i]))
                nan =true;
            else
                nan = false;
        }
        cout << endl;

        if(!nan)
            _motor_pub.publish(rotor_speed);
        
        rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv ) {

    ros::init( argc, argv, "tilted_control");
    TiltedControl tc;
    tc.run();

    return 0;
}
