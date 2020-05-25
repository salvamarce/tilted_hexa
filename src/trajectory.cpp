#include "trajectory.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/tf.h"

using namespace std;

//---Get params from ros parameter server
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
//---------------------------------

class SMTraj{
    public:
        SMTraj();
        void new_ref_cb(std_msgs::Float32MultiArray);
        void publish_values();

        void run();

    private:
        ros::NodeHandle nh;
        ros::Subscriber new_ref;
        ros::Publisher pose;
        ros::Publisher vel;
        ros::Publisher acc;
        
        geometry_msgs::Pose _pose;
        geometry_msgs::Twist _vel;
        geometry_msgs::Twist _acc;

        double _rate;
        double _x0;
        double _y0;
        double _z0;
        double _roll0;
        double _pitch0;
        double _yaw0;
        double _V_max;
        double _A_max;
        double _J_max;

        float _new_ref[6];
        float _old_ref[6];
        bool _new_ref_present[6];

};

SMTraj::SMTraj() {

    string pose_topic_name;
    string vel_topic_name;
    string acc_topic_name;

    load_param( pose_topic_name, "/trajectory/pose", "pose_topic_name");
    load_param( vel_topic_name, "/trajectory/vel", "vel_topic_name");
    load_param( acc_topic_name, "/trajectory/acc", "acc_topic_name");
    load_param( _rate, 250, "trajectory_rate");
    load_param( _V_max, 0.5, "V_max");
    load_param( _A_max, 0.25, "A_max");
    load_param( _J_max, 0.125, "J_max");
    load_param( _x0, 0.0, "x");
    load_param( _y0, 0.0, "y");
    load_param( _z0, -0.288, "z");
    load_param( _roll0, 0.0, "roll");
    load_param( _pitch0, 0.0, "pitch");
    load_param( _yaw0, 0.0, "yaw");

    new_ref = nh.subscribe( "/des_pos", 0, &SMTraj::new_ref_cb, this);
    pose = nh.advertise< geometry_msgs::Pose > ( pose_topic_name.c_str(), 0);
    vel = nh.advertise< geometry_msgs::Twist > ( vel_topic_name.c_str(), 0);
    acc = nh.advertise< geometry_msgs::Twist > ( acc_topic_name.c_str(), 0);

    for(int i=0; i<6; i++){
        _new_ref_present[i] = false;
    }
    _new_ref[0] = _old_ref[0] = _x0;
    _new_ref[1] = _old_ref[1] = _y0;
    _new_ref[2] = _old_ref[2] = _z0;
    _new_ref[3] = _old_ref[3] = _roll0;
    _new_ref[4] = _old_ref[4] = _pitch0;
    _new_ref[5] = _old_ref[5] = _yaw0;

}

void SMTraj::new_ref_cb(std_msgs::Float32MultiArray des){

    for(int i=0; i<6; i++){
        if(des.data[i] != _old_ref[i])
            _old_ref[i] = _new_ref[i];
            _new_ref[i] = des.data[i];
            _new_ref_present[i] = true;
    }
    
}

void SMTraj::run(){
    
    ros::Rate r(_rate);
    double t0_x, t0_y, t0_z,t0_roll, t0_pitch, t0_yaw;
    double px, vx, ax, jx;
    double py, vy, ay, jy;
    double pz, vz, az, jz;
    double proll, vroll, aroll, jroll;
    double ppitch, vpitch, apitch, jpitch;
    double pyaw, vyaw, ayaw, jyaw;
    tf::Quaternion q;
    double count_loop;

    MotionProfile traj_x;
    MotionProfile traj_y;
    MotionProfile traj_z;
    MotionProfile traj_roll;
    MotionProfile traj_pitch;
    MotionProfile traj_yaw;

    px= _x0;
    py = _y0;
    pz = _z0;
    proll= _roll0;
    ppitch= _pitch0;
    pyaw= _yaw0;
    vx = vy = vz = vyaw = vpitch = vroll = 0;
    ax = ay = az = ayaw = apitch = aroll = 0;
    count_loop = 0.0;
    t0_x = t0_y = t0_z = t0_roll = t0_pitch = t0_yaw = 0.0;

    //cout << "euler 0: " << proll << " " << ppitch << " " << pyaw << endl;
    while(ros::ok() ){

      if(_new_ref_present[0]){
        traj_x.setParam(_old_ref[0], _new_ref[0], _V_max, _A_max, _J_max);
        t0_x= count_loop;
        _new_ref_present[0] = false;
      }
      
      if(_new_ref_present[1]){
        traj_y.setParam(_old_ref[1], _new_ref[1], _V_max, _A_max, _J_max);
        t0_y= count_loop;
        _new_ref_present[1] = false;
      }
      
      if(_new_ref_present[2]) {
        traj_z.setParam(_old_ref[2], _new_ref[2], _V_max, _A_max, _J_max);
        t0_z= count_loop;
        _new_ref_present[2] = false;
      }
      
      if(_new_ref_present[3]){
        traj_roll.setParam(_old_ref[3], _new_ref[3], _V_max, _A_max, _J_max);
        t0_roll= count_loop;
        _new_ref_present[3] = false;
        //cout << "nuovo roll" << _new_ref[3] << endl;
      }

      if(_new_ref_present[4]){
        traj_pitch.setParam(_old_ref[4], _new_ref[4], _V_max, _A_max, _J_max);
        t0_pitch= count_loop;
        _new_ref_present[4] = false;
        //cout << "nuovo pitch" << _new_ref[4] << endl;
      }

      if(_new_ref_present[5]){
        traj_yaw.setParam(_old_ref[5], _new_ref[5], _V_max, _A_max, _J_max);
        t0_yaw= count_loop;
        _new_ref_present[5] = false;
        //cout << "nuovo yaw" << _new_ref[5] << endl;
      }
      
      if(count_loop < traj_x.Duration()+t0_x)
        traj_x.Compute(count_loop, t0_x, px, vx, ax, jx);
      
      if(count_loop < traj_y.Duration()+t0_y)
        traj_y.Compute(count_loop, t0_y, py, vy, ay, jy);
      
      if(count_loop < traj_z.Duration()+t0_z)
        traj_z.Compute(count_loop, t0_z, pz, vz, az, jz);

      if(count_loop < traj_roll.Duration()+t0_roll){
        traj_roll.Compute(count_loop, t0_roll, proll, vroll, aroll, jroll);
        //cout << "entrato roll \n";
      }

      if(count_loop < traj_pitch.Duration()+t0_pitch){
        traj_pitch.Compute(count_loop, t0_pitch, ppitch, vpitch, apitch, jpitch);
        //cout << "entrato pitch \n";
      }

      if(count_loop < traj_yaw.Duration()+t0_yaw){
        traj_yaw.Compute(count_loop, t0_yaw, pyaw, vyaw, ayaw, jyaw);
        //cout << "entrato yaw \n";
      }

      _pose.position.x = px;
      _pose.position.y = py;
      _pose.position.z = pz;

      q.setRPY(proll, ppitch, pyaw);
      //q = q.normalize();
      //cout << "loop: " << count_loop << endl;
      //cout << "pitch dur: " << traj_pitch.Duration() << endl;
      //cout << "roll dur: " << traj_roll.Duration() << endl;
      //cout << "yaw dur: " << traj_yaw.Duration() << endl;
      //cout << "euler: " << proll << " " << ppitch << " " << pyaw << endl;

      _pose.orientation.x =  q.x();
      _pose.orientation.y =  q.y();
      _pose.orientation.z =  q.z();
      _pose.orientation.w =  q.w();

      _vel.linear.x = vx;
      _vel.linear.y = vy;
      _vel.linear.z = vz;

      _vel.angular.x = vroll;
      _vel.angular.y = vpitch;
      _vel.angular.z = vyaw;

      _acc.linear.x = ax;
      _acc.linear.y = ay;
      _acc.linear.z = az;

      _acc.angular.x = aroll;
      _acc.angular.y = apitch;
      _acc.angular.z = ayaw;
      
      
      pose.publish(_pose);
      vel.publish(_vel);
      acc.publish(_acc);
      
      count_loop += 1/_rate;
      r.sleep();
      ros::spinOnce();  
    }    
}

int main(int argc, char** argv ) {
    ros::init( argc, argv, "trajectory");
    SMTraj t;

    t.run();

    return 0;
}