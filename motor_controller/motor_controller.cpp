#include <ros/ros.h>
#include <std_msgs/Float64.h>


class MotorControllerNode {
 public:
  MotorControllerNode(): phi_tm1(0.0), omega_tm1(0.0), integrator(0.0) {
    ros::NodeHandle nh;
		sub_ = nh.subscribe("gazebo_motor_plugin/motor_angle", 100, &MotorControllerNode::update, this);
		pub_ = nh.advertise<std_msgs::Float64>("motor_correction_variable", 100);
  }

  virtual ~MotorControllerNode() {

  }

  void init() {

  }

	void update(const std_msgs::Float64::ConstPtr &msg_in) {
    double phi_t = msg_in->data;
		double omega_t = (phi_t - phi_tm1)/1e-3;
		double error_t = 1 - omega_t;
		double con_p = error_t * 16.5;
		double con_i = integrator + (error_t + error_tm1)/2.0 * 32.5;
		double torque = (con_p + con_i)*km;

		std_msgs::Float64 msg_out;
		msg_out.data = torque;
		pub_.publish(msg_out);

    ROS_INFO("Motor phi: %f", phi_t);
    ROS_INFO("Motor omega: %f", omega_t);
    ROS_INFO("Force to set: %f", torque);

		phi_tm1 = phi_t;
		omega_tm1 = omega_t;
  }


 private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
	double phi_tm1;
	double omega_tm1;
	double integrator;
	double error_tm1;

	static const double km = 0.0163;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_controller");
	//pub = nh.advertise<std_msgs::Float64>("motor_command", 100);
	//sub = nh.subscribe("motor_position", 100, update);
	//Rate loop_rate(1000);

	//while(ros::ok())
	//{
	//	ros::spinOnce
	//}

	MotorControllerNode motor_controller_node;

	ros::spin();

	return 0;
}

