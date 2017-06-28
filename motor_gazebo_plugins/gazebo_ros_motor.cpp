#ifndef GAZEBO_ROS_MOTOR_H
#define GAZEBO_ROS_MOTOR_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

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
	}

	void OnUpdate() {
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
	std::string motor_axis_joint_name_;
	int counter_ = 0;
	bool force_set_ = false;
	bool parameter_read_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotor)

} // namespace gazebo

#endif /* GAZEBO_ROS_MOTOR_H */
