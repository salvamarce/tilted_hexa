#include "tilted_control.h"
#include "Eigen/Dense"
#include "Eigen/QR"
#include "tf/tf.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <fstream>

using namespace std;

fstream mis_file;
fstream des_file;
fstream data_file;

//---Get params from ros parameter server-----
void load_param( string & p, string def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  //cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  //cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  //cout << name << ": " << "\t" << p << endl;
}

void load_param( double & p, double def, string name ) {
  ros::NodeHandle n_param("~");
  if( !n_param.getParam( name, p))
    p = def;
  //cout << name << ": " << "\t" << p << endl;
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
        Eigen::Matrix<double, 6, 6>  _J, _J_inv;
        Eigen::Matrix<double, 6, 6> _JR;
        Eigen::Matrix<double, 6, 6> _Allocation;

        double _Kpp, _Kdp, _Kip, _Kpr, _Kdr, _Kir;
        double _Kpr_z, _Kdr_z, _Kir_z;
        double _Kpp_z, _Kdp_z, _Kip_z;
        double _alpha;
        double _Km;
        double _Kf;
        double _L;
        double _Z;
        double _M;
        double _IBxx, _IByy, _IBzz;
        double _rate;
        double _f;
        
        void pose_cb( geometry_msgs::PoseStamped pose );
        void vel_cb( geometry_msgs::TwistStamped vel );
        void des_pose_cb( geometry_msgs::Pose pose);
        void des_vel_cb( geometry_msgs::Twist vel);
        void des_acc_cb( geometry_msgs::Twist acc);
        Eigen::Vector3d skew2vec(Eigen::Matrix3d mis_att, Eigen::Matrix3d des_att);

        Eigen::Vector3d _des_rpy, _mis_rpy;

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
    load_param( _M, 1.15, "Mass");
    load_param( _IBxx, 0.0291, "IBxx");
    load_param( _IByy, 0.0291, "IByy");
    load_param( _IBzz, 0.0552, "IBzz");
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
    
    Eigen::Quaterniond q_eig( pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,  pose.pose.orientation.z);
    //q_eig.normalize();
	_mis_att = q_eig.toRotationMatrix();
    _first_pose = true;
    cout << "mis pos: " << _mis_pos.transpose() << endl;
    //cout << "mis att: \n" << _mis_att << endl;
    
    tf::Quaternion q( pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,  pose.pose.orientation.w);
	tf::Matrix3x3(q).getRPY(_mis_rpy(0), _mis_rpy(1), _mis_rpy(2));
    cout <<"mis rpy: "<< _mis_rpy(0) << " "  << _mis_rpy(1)<< " "<< _mis_rpy(2) << endl;
    

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
    //cout << "des_att: \n" << _des_att << endl;
    double roll, pitch, yaw;
    tf::Quaternion q( pose.orientation.x, pose.orientation.y, pose.orientation.z,  pose.orientation.w);
	tf::Matrix3x3(q).getRPY(_des_rpy(0), _des_rpy(1), _des_rpy(2));
    cout <<"des_rpy: "<< _des_rpy(0) << " "  << _des_rpy(1)<< " "<< _des_rpy(2) << endl;

    EvaluateJ();

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
    
    //tilt con terna body NED
    _tilt[0].data = alpha;
    _tilt[1].data = -alpha;
    _tilt[2].data = -alpha;
    _tilt[3].data = alpha;
    _tilt[4].data = -alpha;
    _tilt[5].data = alpha;
    /*
    //tilt con terna body NOU
    _tilt[0].data = -alpha;
    _tilt[1].data = alpha;
    _tilt[2].data = alpha;
    _tilt[3].data = -alpha;
    _tilt[4].data = alpha;
    _tilt[5].data = -alpha;
    */
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
            //cout << "Motors not tilted" << endl;
        }

}

Eigen::Vector3d TiltedControl::skew2vec(Eigen::Matrix3d mis_att, Eigen::Matrix3d des_att){

    Eigen::Vector3d eR;
    Eigen::Matrix<double, 3, 3> eM;

    eM = des_att.transpose()*mis_att - mis_att.transpose()*des_att;
    eR << eM(2, 1), eM(0,2), eM(1, 0);
    
    //cout << "a mis: \n" << mis_att << endl;
    //cout << "a des: \n" << des_att << endl;
    //cout << "skew: \n" << eM << endl;
    //cout << "eR: \n" << eR.transpose() << endl;
    
    return eR;
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

    Eigen::Matrix<double, 3, 3> Rned2nou;
    Rned2nou << 1, 0, 0, 0, -1, 0, 0, 0, -1; 

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
        ////cout << "e_p_int: " << e_p_int.transpose() << endl;
        ////cout << "e_a_int: " << e_att_int.transpose() << endl;
        
        ni_att(0) = _des_dd_att(0) - _Kdr * e_d_att(0) - _Kpr * e_att(0) - _Kir * e_att_int(0); 
        ni_att(1) = _des_dd_att(1) - _Kdr * e_d_att(1) - _Kpr * e_att(1) - _Kir * e_att_int(1); 
        ni_att(2) = _des_dd_att(2) - _Kdr_z * e_d_att(2) - _Kpr_z * e_att(2) - _Kir_z * e_att_int(2); 

        //ni_att << 0.0, 0.0, 0.0;
        ni_pos(0) = _des_acc(0) - _Kdp * e_vel(0) - _Kpp * e_pos(0) - _Kip * e_p_int(0);
        ni_pos(1) = _des_acc(1) - _Kdp * e_vel(1) - _Kpp * e_pos(1) - _Kip * e_p_int(1);
        ni_pos(2) = _des_acc(2) - _Kdp_z * e_vel(2) - _Kpp_z * e_pos(2) - _Kip_z * e_p_int(2);
        //ni_pos = Rned2nou * ni_pos;
        //ni_att = Rned2nou * ni_att;
        ni << ni_pos, ni_att;

        cout << "ni_pos: " << ni_pos.transpose() << endl;
        cout << "ni_att: " << ni_att.transpose() << endl;
        ////cout << "ni: " << ni.transpose() << endl;

        control_output = _J_inv * (-f + ni);
        //control_output = _J.lu().solve(-f+ni);
        ////cout << "J inv: \n" << _J_inv << endl;
        ////cout << "f: " << f.transpose() << endl;
        //control_output = control_output.cwiseMax(Eigen::VectorXd::Zero(6));
        cout << "control output: " << control_output.transpose() << endl;
        
        cout << "rotor speed: ";
        for(int i=0; i<6; i++){
            rotor_speed.data[i] = sqrt(fabs(control_output(i)));
            cout << rotor_speed.data[i] << "  ";
            if(isnan(rotor_speed.data[i]))
                nan = true;
        }
        cout << endl;

        if(!nan)
            _motor_pub.publish(rotor_speed);
        
        data_file.open("/home/uav/csv/data.csv", ios::out | ios::app);
        mis_file.open("/home/uav/csv/mis.csv", ios::out | ios::app);
        des_file.open("/home/uav/csv/des.csv", ios::out | ios::app);

        des_file << _des_pos(0) << "," << _des_pos(1) << "," << _des_pos(2) << ","
                 << _des_vel(0) << "," << _des_vel(1) << "," << _des_vel(2) << ","
                 << _des_acc(0) << "," << _des_acc(1) << "," << _des_acc(2) << ","
                 << _des_att(0) << "," << _des_att(1) << "," << _des_att(2) << ","
                 << _des_d_att(0) << "," << _des_d_att(1) << "," << _des_d_att(2) << ","
                 << _des_dd_att(0) << "," << _des_dd_att(1) << "," << _des_dd_att(2) << "\n";

        mis_file << _mis_pos(0) << "," << _mis_pos(1) << "," << _mis_pos(2) << ","
                 << _mis_vel(0) << "," << _mis_vel(1) << "," << _mis_vel(2) << ","
                 << _mis_att(0) << "," << _mis_att(1) << "," << _mis_att(2) << ","
                 << _mis_d_att(0) << "," << _mis_d_att(1) << "," << _mis_d_att(2) << "\n";

        data_file << e_pos(0) << "," << e_pos(1) << "," << e_pos(2) << ","
                  << e_vel(0) << "," << e_vel(1) << "," << e_vel(2) << ","
                  << e_att(0) << "," << e_att(1) << "," << e_att(2) << ","
                  << e_d_att(0) << "," << e_d_att(1) << "," << e_d_att(2) << ","
                  << ni_pos(0) << "," << ni_pos(1) << "," << ni_pos(2) << ","
                  << ni_att(0) << "," << ni_att(1) << "," << ni_att(2) << ","
                  << rotor_speed.data[0] << "," << rotor_speed.data[1] << ","
                  << rotor_speed.data[2] << "," << rotor_speed.data[3] << ","
                  << rotor_speed.data[4] << "," << rotor_speed.data[5] << "\n";

        data_file.close();
        mis_file.close();
        des_file.close();

        nan = false;
        rate.sleep();
        ros::spinOnce();
    }
}

void TiltedControl::EvaluateJ(){
    double a1, a2, a3, a4, a5, a6;
    double ang1, ang2, ang3, ang4, ang5, ang6;
    double _pi = 3.1415;
    
    
    //Angoli motori in terna body NED
    ang1 = 7*_pi/6;
    ang2 = _pi/6;
    ang3 = 5*_pi/6;
    ang4 = 11*_pi/6;
    ang5 = 3*_pi/2;
    ang6 = _pi/2;
    
    //cw:1  ccw:-1 
    double sign1 = -1;
    double sign2 = 1;
    double sign3 = 1;
    double sign4 = -1;
    double sign5 = 1;
    double sign6 = -1;
    
    /*
    //Angoli motori in terna body NOU
    ang1 = 5*_pi/6;
    ang2 = 11*_pi/6;
    ang3 = 7*_pi/6;
    ang4 = _pi/6;
    ang5 = _pi/2;
    ang6 = 3*_pi/2;

    //cw: -1  ccw: 1 (nella matrice c'è -Kf e -Km) -> quidndi con questo devo mettere dei - nel launch per avere +
    double sign1 = 1;
    double sign2 = -1;
    double sign3 = -1;
    double sign4 = 1;
    double sign5 = -1;
    double sign6 = 1;
    */
    double c_ang1 = cos(ang1);
    double c_ang2 = cos(ang2);
    double c_ang3 = cos(ang3);
    double c_ang4 = cos(ang4);
    double c_ang5 = cos(ang5);
    double c_ang6 = cos(ang6);
    double s_ang1 = sin(ang1);
    double s_ang2 = sin(ang2);
    double s_ang3 = sin(ang3);
    double s_ang4 = sin(ang4);
    double s_ang5 = sin(ang5);
    double s_ang6 = sin(ang6);

    a1 = _tilt[0].data;
    a2 = _tilt[1].data;
    a3 = _tilt[2].data;
    a4 = _tilt[3].data;
    a5 = _tilt[4].data;
    a6 = _tilt[5].data;

    double c_a1 = cos(a1);
    double c_a2 = cos(a2);
    double c_a3 = cos(a3);
    double c_a4 = cos(a4);
    double c_a5 = cos(a5);
    double c_a6 = cos(a6);
    double s_a1 = sin(a1);
    double s_a2 = sin(a2);
    double s_a3 = sin(a3);
    double s_a4 = sin(a4);
    double s_a5 = sin(a5);
    double s_a6 = sin(a6);
    
    _JR << (1.0/_M)*_mis_att(0,0), (1.0/_M)*_mis_att(0,1), (1.0/_M)*_mis_att(0,2), 0.0, 0.0, 0.0,
           (1.0/_M)*_mis_att(1,0), (1.0/_M)*_mis_att(1,1), (1.0/_M)*_mis_att(1,2), 0.0, 0.0, 0.0,
           (1.0/_M)*_mis_att(2,0), (1.0/_M)*_mis_att(2,1), (1.0/_M)*_mis_att(2,2), 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 1.0/_IBxx, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 1.0/_IByy, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 1.0/_IBzz;
       
    cout << "JR: \n" << _JR << endl;
    //TO DO: _Allocation è statica, calcolarla solo quando setto gli angoli
    //nella matrice c'è -Kf e -Km
    _Allocation << -_Kf*s_a1*s_ang1, -_Kf*s_a2*s_ang2, -_Kf*s_a3*s_ang3, -_Kf*s_a4*s_ang4, -_Kf*s_a5*s_ang5, -_Kf*s_a6*s_ang6, //row 1
                   _Kf*c_ang1*s_a1, _Kf*c_ang2*s_a2, _Kf*c_ang3*s_a3, _Kf*c_ang4*s_a4, _Kf*c_ang5*s_a5, _Kf*c_ang6*s_a6, //row 2
                   -_Kf*c_a1, -_Kf*c_a2, -_Kf*c_a3, -_Kf*c_a4, -_Kf*c_a5, -_Kf*c_a6, //row 3
                   _Kf*_Z*c_ang1*s_a1 - _Kf*_L*c_a1*s_ang1 - _Km*sign1*s_a1*s_ang1,
                   _Kf*_Z*c_ang2*s_a2 - _Kf*_L*c_a2*s_ang2 - _Km*sign2*s_a2*s_ang2,
                   _Kf*_Z*c_ang3*s_a3 - _Kf*_L*c_a3*s_ang3 - _Km*sign3*s_a3*s_ang3,
                   _Kf*_Z*c_ang4*s_a4 - _Kf*_L*c_a4*s_ang4 - _Km*sign4*s_a4*s_ang4,
                   _Kf*_Z*c_ang5*s_a5 - _Kf*_L*c_a5*s_ang5 - _Km*sign5*s_a5*s_ang5,
                   _Kf*_Z*c_ang6*s_a6 - _Kf*_L*c_a6*s_ang6 - _Km*sign6*s_a6*s_ang6, //row(4)
                   _Kf*_L*c_a1*c_ang1 + _Kf*_Z*s_a1*s_ang1 + _Km*sign1*c_ang1*s_a1,
                   _Kf*_L*c_a2*c_ang2 + _Kf*_Z*s_a2*s_ang2 + _Km*sign2*c_ang2*s_a2,
                   _Kf*_L*c_a3*c_ang3 + _Kf*_Z*s_a3*s_ang3 + _Km*sign3*c_ang3*s_a3,
                   _Kf*_L*c_a4*c_ang4 + _Kf*_Z*s_a4*s_ang4 + _Km*sign4*c_ang4*s_a4,
                   _Kf*_L*c_a5*c_ang5 + _Kf*_Z*s_a5*s_ang5 + _Km*sign5*c_ang5*s_a5,
                   _Kf*_L*c_a6*c_ang6 + _Kf*_Z*s_a6*s_ang6 + _Km*sign6*c_ang6*s_a6, //row 5
                   _Kf*_L*pow(c_ang1,2)*s_a1 - _Km*sign1*c_a1 + _Kf*_L*s_a1*pow(s_ang1,2),
                   _Kf*_L*pow(c_ang2,2)*s_a2 - _Km*sign2*c_a2 + _Kf*_L*s_a2*pow(s_ang2,2),
                   _Kf*_L*pow(c_ang3,2)*s_a3 - _Km*sign3*c_a3 + _Kf*_L*s_a3*pow(s_ang3,2),
                   _Kf*_L*pow(c_ang4,2)*s_a4 - _Km*sign4*c_a4 + _Kf*_L*s_a4*pow(s_ang4,2),
                   _Kf*_L*pow(c_ang5,2)*s_a5 - _Km*sign5*c_a5 + _Kf*_L*s_a5*pow(s_ang5,2),
                   _Kf*_L*pow(c_ang6,2)*s_a6 - _Km*sign6*c_a6 + _Kf*_L*s_a6*pow(s_ang6,2); //row 6

    ////cout << "Allor: \n" << _Allocation << endl;
    _J = _JR*_Allocation;
    _J_inv = _J.inverse();
}


int main(int argc, char** argv ) {

    ros::init( argc, argv, "tilted_control");
    TiltedControl tc;

    data_file.open("/home/uav/csv/data.csv", ios::out );
    mis_file.open("/home/uav/csv/mis.csv", ios::out );
    des_file.open("/home/uav/csv/des.csv", ios::out );

    des_file << "des_p_x, des_p_y, des_p_z, des_vel_x, des_vel_y, des_vel_z, des_acc_x, des_acc_y, des_acc_z, des_att_roll, des_att_pitch, des_att_yaw, des_d_att_roll, des_d_att_pitch, des_d_att_yaw, des_dd_att_roll, des_dd_att_pitch, des_dd_att_yaw \n";
    mis_file << "mis_p_x, mis_p_y, mis_p_z, mis_vel_x, mis_vel_y, mis_vel_z, mis_acc_x, mis_acc_y, mis_acc_z, mis_att_roll, mis_att_pitch, mis_att_yaw, mis_d_att_roll, mis_d_att_pitch, mis_d_att_yaw, mis_dd_att_roll, mis_dd_att_pitch, mis_dd_att_yaw \n";
    data_file << "e_p_x, e_p_y, e_p_z, e_vel_x, e_vel_y, e_vel_z, e_att_roll, e_att_pitch, e_att_yaw, e_d_att_roll, e_d_att_pitch, e_d_att_yaw, ni_x, ni_y, ni_z, ni_roll, ni_pitch, ni_yaw, w1, w2, w3, w4, w5, w6 \n";
    
    data_file.close();
    mis_file.close();
    des_file.close();

    tc.run();
    return 0;
}