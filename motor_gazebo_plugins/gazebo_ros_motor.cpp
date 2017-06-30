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

namespace gazebo
{

class GazeboRosMotor : public ModelPlugin
{
public:
	GazeboRosMotor()
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

		// Variant1
		printf("Start ROS with variant 1\n");
		if(!ros::isInitialized()) {
			printf("ros:isInitialized() -> false\n");
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv,"gazebo_client", ros::init_options::NoSigintHandler);
		}

		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
		this->rosSub_ = this->rosNode->subscribe("chatter", 1000, &GazeboRosMotor::OnRosMsg2, this);
		

		// Varinat2
		/*
		printf("Start ROS with variant 2\n");
		gazebo_ros_ = GazeboRosPtr(new GazeboRos(model_, sdf_, "Motor"));
		if(!ros::isInitialized()) return;

		rosSub_ = gazebo_ros_->node()->subscribe("gazebo_test", 1000, &GazeboRosMotor::OnRosMsg, this);
		*/
		printf("End of Load-Function\n");
	}

/*	void OnUpdate() {

		//printf("%s\n", std::to_string(counter_++).c_str());
		if(!force_set_) {
			joint_->SetForce(0, 0.00539135);
			//link_->SetTorque(ignition::math::Vector3d  0.000053657);
			//force_set_ = true;
		} else {
			joint_->SetForce(0, 0);
		}

		if(counter_++ >= 99) {
			double force = joint_->GetForce(0);
			printf("GetForce(0): %s\n", std::to_string(force).c_str());
      printf("GetEffortLimit(0): %s\n", std::to_string(joint_->GetEffortLimit(0)).c_str());
      printf("GetVelocity(0): %s\n", std::to_string(joint_->GetVelocity(0)).c_str());
      printf("GetAngle(0): %s\n", std::to_string(joint_->GetAngle(0).Radian()).c_str());
			counter_ = 0;
		}
	} */

	void OnUpdate() {
		printf("OnUpdate called\n");
		ros::spinOnce();
		printf("OnUpdate end\n");
	}

	void OnRosMsg(const std_msgs::Float64ConstPtr &msg) {
		printf("OnRosMsg called, Msg: %f\n", msg->data);
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
	std::unique_ptr<ros::NodeHandle> rosNode;
	ros::Subscriber rosSub_;
	GazeboRosPtr gazebo_ros_;
//	ros::CallbackQueue rosQueue;
	std::string motor_axis_joint_name_;
	int counter_ = 0;
	bool force_set_ = false;
	bool parameter_read_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotor)

} // namespace gazebo

#endif /* GAZEBO_ROS_MOTOR_H */
