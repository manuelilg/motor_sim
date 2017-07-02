#ifndef GAZEBO_ROS_MOTOR_H
#define GAZEBO_ROS_MOTOR_H

#include <stdio.h>
#include <unistd.h>

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
	GazeboRosMotor(): force_to_set_(0.0)
	{
		printf("Hello from the GazeboRosMotor!\n");
	}

	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		this->model_ = parent;
		this->sdf_ = sdf;

		if(prepareMotorAxis() == false) return;


//		update_connection_end_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboRosMotor::OnUpdate, this));
		update_connection_begin_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosMotor::OnUpdate, this));
		if(!update_connection_begin_) printf("Error no Update Connection\n");


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

		rosQueue_.clear();

		std_msgs::Float64 msg;
		msg.data = joint_->GetAngle(0).Radian();
		rosPub_.publish(msg);

		while(rosQueue_.isEmpty()) {
			usleep(50);
			ROS_INFO("Waiting for motor_controller_node");
		}

		if(rosQueue_.callOne() != ros::CallbackQueue::CallOneResult::Called) {
			ROS_ERROR("Error in gazebo_ros_motor plugin callback not successful called");
		}

		joint_->SetForce(0, force_to_set_);

		ROS_INFO("GetForce(0): %f", joint_->GetForce(0));
    ROS_INFO("GetEffortLimit(0): %f", joint_->GetEffortLimit(0));
    ROS_INFO("GetVelocity(0): %f", joint_->GetVelocity(0));
    ROS_INFO("GetAngle(0): %f", joint_->GetAngle(0).Radian());
	}

	void OnRosMsg(const std_msgs::Float64ConstPtr &msg) {
		force_to_set_ = msg->data;
		//ROS_INFO("Callback: %f", msg->data);
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

private:
	physics::ModelPtr model_;
	sdf::ElementPtr sdf_;
	event::ConnectionPtr update_connection_begin_;
	event::ConnectionPtr update_connection_end_;
	physics::JointPtr joint_;
	physics::LinkPtr link_;
	std::unique_ptr<ros::NodeHandle> rosNode_;
	ros::Subscriber rosSub_;
	ros::Publisher rosPub_;
	ros::CallbackQueue rosQueue_;

	double force_to_set_;

	std::string motor_axis_joint_name_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotor)

} // namespace gazebo

#endif /* GAZEBO_ROS_MOTOR_H */
