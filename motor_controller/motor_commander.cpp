#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_controller");
	//pub = nh.advertise<std_msgs::Float64>("motor_command", 100);
	//sub = nh.subscribe("motor_position", 100, update);

  ros::NodeHandle nh;
	ros::Rate loop_rate(1000);

	ros::Publisher pub = nh.advertise<std_msgs::Float64>("rosNodeTalker/TestTopic1", 10);


	int a = 50;
	double t_a = 1.0;

	double output_increment = a/1000.0;

//	int counter = 0;
	double max = t_a*50;


	double output = 0.0;

	bool accelerate = true;

	while(ros::ok())
	{
		//ros::spinOnce;
		std_msgs::Float64 msg;
		msg.data = output;
		pub.publish(msg);

		if(accelerate) {
			output += output_increment;
			if(output>max) {
				accelerate = !accelerate;
			}
		} else {
			output -= output_increment;
			if(output<0.0) {
				accelerate = !accelerate;
			}
		}



		loop_rate.sleep();
	}

//	ros::spin();

	return 0;
}

