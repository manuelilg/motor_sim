#ifndef GAZEBO_ROS_MOTOR_H
#define GAZEBO_ROS_MOTOR_H

#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>

namespace gazebo
{

class GazeboRosMotor : public ModelPlugin
{
public:
	GazeboRosMotor(): force_to_set_(0.0), error_tm1(0.0), integrator(0.0), controller_i_tm1(0.0)
	{
		printf("Hello from the GazeboRosMotor!\n");
	}

	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		this->model_ = parent;
		this->sdf_ = sdf;

		if(prepareMotorAxis() == false) return;


//		update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboRosMotor::OnUpdate, this));
		update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosMotor::OnUpdate, this));
		if(!update_connection_) printf("Error no Update Connection\n");

		if(!ros::isInitialized()) {
 			ROS_FATAL_STREAM("A ROS node for Gazegbo has not been initialized, unable to load plugin.");
 			return;
		}
 		this->rosNode_.reset(new ros::NodeHandle("gazebo_motor_plugin"));

		rosPub_ = rosNode_->advertise<std_msgs::Float64>("motor_angle", 100);

		ros::SubscribeOptions subOps = ros::SubscribeOptions::create<std_msgs::Float64>(
			"/motor_correction_variable",
			100,
			boost::bind(&GazeboRosMotor::OnRosMsg, this, _1),
			ros::VoidPtr(),
			&this->rosQueue_);
		rosSub_ = rosNode_->subscribe(subOps);

		printf("End of Load-Function\n");
	}

	void OnUpdate() {

		std_msgs::Float64 msg;
		msg.data = joint_->GetAngle(0).Radian();
		rosPub_.publish(msg);
		//rosQueue_.callOne();
		//joint_->SetForce(0, force_to_set_);

		double omega_t = joint_->GetVelocity(0);
		double error_t = 1 - omega_t;
		double controller_p = error_t * 16.5;
		double controller_i_t = error_t * 32.5;
		integrator = integrator + (controller_i_t + controller_i_tm1)/2*1e-3;
		double torque = (integrator + controller_p)*0.0163;

		joint_->SetForce(0, torque);

		error_tm1 = error_t;
		controller_i_tm1 = controller_i_t;

/*		//printf("%s\n", std::to_string(counter_++).c_str());
		if(!force_set_) {
			joint_->SetForce(0, 0.00539135);
			//link_->SetTorque(ignition::math::Vector3d  0.000053657);
			//force_set_ = true;
		} else {
			joint_->SetForce(0, 0);
		}*/

		//if(counter_++ >= 99) {
			printf("GetForce(0): %f\n", joint_->GetForce(0));
      printf("GetEffortLimit(0): %f\n", joint_->GetEffortLimit(0));
      printf("GetVelocity(0): %f\n", joint_->GetVelocity(0));
      printf("GetAngle(0): %f\n", joint_->GetAngle(0).Radian());
     // printf("Force_to_set: %f\n", force_to_set_);
		//	counter_ = 0;
		//}
	}

	void OnRosMsg(const std_msgs::Float64ConstPtr &msg) {
		force_to_set_ = msg->data;
		printf("Callback: %f\n", msg->data);
	}

	void OnRosMsg2(const std_msgs::String::ConstPtr& msg) {
    printf("OnRosMsg called, Msg: %s\n", msg->data.c_str());
	}

private: 
	bool prepareMotorAxis() {
		if(sdf_->HasElement("motor_axis_joint_name")) {
			motor_axis_joint_name_ = sdf_->GetElement("motor_axis_joint_name")->Get<std::string>();
			printf("motor_axis_joint_name: %s\n", motor_axis_joint_name_.c_str());
		} else {
			printf("Error no XML-Element motor_axis_joint_name found\n");
			return false;
		}
		joint_ = model_->GetJoint(motor_axis_joint_name_);
		if(!joint_){
			printf("Error motor_axis_joint_name is not valid\n");
			return false;
		}

		return true;
	}

	bool prepareRotorLink() {
//		link_ = model_->getLink("rotor");
//		if(!link_) printf("ERRRRRROOOOR!!!!!!!!\n");
	} 


private: 
	physics::ModelPtr model_;
	sdf::ElementPtr sdf_;
	event::ConnectionPtr update_connection_;
	physics::JointPtr joint_;
	physics::LinkPtr link_;
	std::unique_ptr<ros::NodeHandle> rosNode_;
	ros::Subscriber rosSub_;
	ros::Publisher rosPub_;
	GazeboRosPtr gazebo_ros_;
	ros::CallbackQueue rosQueue_;
	double force_to_set_;
	double error_tm1;
	double integrator;
	double controller_i_tm1;

	std::string motor_axis_joint_name_;
	int counter_ = 0;
	bool force_set_ = false;
	bool parameter_read_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotor)

} // namespace gazebo

#endif /* GAZEBO_ROS_MOTOR_H */
