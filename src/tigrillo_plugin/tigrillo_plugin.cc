#ifndef _TIGRILLO_PLUGIN_HH_
#define _TIGRILLO_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/console.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h" // To replace with dynamixel messages?
#include "std_msgs/MultiArrayLayout.h"

namespace gazebo
{
	/// \brief A plugin to control a Tigrillo quadruped with ROS.
	class TigrilloPlugin : public ModelPlugin
	{
		/// \brief Pointer to the model.
		private: physics::ModelPtr model;
		
		/// \brief Name of the joints.
		std::string name_shoulder_L = "tigrillo::FLUJ";
		std::string name_shoulder_R = "tigrillo::FRUJ";
		std::string name_hip_L = "tigrillo::BLUJ";
		std::string name_hip_R = "tigrillo::BRUJ";

		/// \brief Pointers to the joints.
		private: physics::JointPtr joint_shoulder_L;
		private: physics::JointPtr joint_shoulder_R;
		private: physics::JointPtr joint_hip_L;
		private: physics::JointPtr joint_hip_R;

		/// \brief PID controllers for the joints.
		private: common::PID pid_shoulder_L;
		private: common::PID pid_shoulder_R;
		private: common::PID pid_hip_L;
		private: common::PID pid_hip_R;
		
		/// \brief Constructor
		public: TigrilloPlugin() {}
		
		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{

			ROS_INFO("Loading Tigrillo ROS plugin");
			ROS_INFO_STREAM("Gazebo is using the physics engine: " <<
			_model->GetWorld()->GetPhysicsEngine()->GetType());

			// Set PID parameters
			double p = 5;
			double i = 0;
			double d = 0;
			if (_sdf->HasElement("p")) {
				p = _sdf->Get<double>("p");
			}
			if (_sdf->HasElement("i")) {
				i = _sdf->Get<double>("i");
			}
			if (_sdf->HasElement("d")) {
				d = _sdf->Get<double>("d");
			}

			ROS_INFO_STREAM("PID values are set to: " << p << " " << i << " " << d);

			// Safety check
			if (_model->GetJointCount() == 0)
			{
				ROS_ERROR("Invalid joint count, plugin not loaded");
				return;
			}

			// Store the model pointer for convenience.
			this->model = _model;

			// Print all joints
			//std::vector<physics::JointPtr> jvec = this->model->GetJoints();
			//for(std::vector<physics::JointPtr>::iterator it = jvec.begin(); it != jvec.end(); ++it) {
			//	std::cerr << (*it)->GetName () << "\n";
			//}

			// Get the four actuated joints
			this->joint_shoulder_L = this->model->GetJoint(this->name_shoulder_L);
			this->joint_shoulder_R = this->model->GetJoint(this->name_shoulder_R);
			this->joint_hip_L = this->model->GetJoint(this->name_hip_L);
			this->joint_hip_R = this->model->GetJoint(this->name_hip_R);

			// Setup a P-controller, with a gain of 0.1.
			this->pid_shoulder_L = common::PID(p, i, d);
			this->pid_shoulder_R = common::PID(p, i, d);
			this->pid_hip_L = common::PID(p, i, d);
			this->pid_hip_R = common::PID(p, i, d);

			// Apply the P-controller to the joint.
			this->model->GetJointController()->SetPositionPID(
				this->joint_shoulder_L->GetScopedName(), this->pid_shoulder_L);
			this->model->GetJointController()->SetPositionPID(
				this->joint_shoulder_R->GetScopedName(), this->pid_shoulder_R);
			this->model->GetJointController()->SetPositionPID(
				this->joint_hip_L->GetScopedName(), this->pid_hip_L);
			this->model->GetJointController()->SetPositionPID(
				this->joint_hip_R->GetScopedName(), this->pid_hip_R);

			// Default to zero position
			float position[4] = {0, 0, 0, 0};
			this->SetPositionTarget(position);

			// Initialize ROS
			this->RosInit();
		}

		/// \brief Set the position of all actuators
		/// \param[in] _pos Array of target position
		public: void SetPositionTarget(float _pos[])
		{
			// Set the joint's target position.
			float p_pos_0 = this->joint_shoulder_L->GetAngle(0).Radian();
			float p_pos_1 = this->joint_shoulder_R->GetAngle(0).Radian();
			float p_pos_2 = this->joint_hip_L->GetAngle(0).Radian();
			float p_pos_3 = this->joint_hip_R->GetAngle(0).Radian();
			ROS_INFO_STREAM("Updating position. Previous " << p_pos_0 << " "
			<< p_pos_1 << " " << p_pos_2 << " " << p_pos_3 << " and new: "
			<< _pos[0] << " " <<_pos[1] << " " << _pos[2] << " " << _pos[3]);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_shoulder_L->GetScopedName(), _pos[0]);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_shoulder_R->GetScopedName(), _pos[1]);
			this->model->GetJointController()->SetJointPosition(
				this->joint_hip_L->GetScopedName(), _pos[2]);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_hip_R->GetScopedName(), _pos[3]);
		}
		
		
		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the actuators 
		// positions
		public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
		{
			
			float position[4];
			int i = 0;
			for(std::vector<float>::const_iterator it = _msg->data.begin(); 
				it != _msg->data.end(); ++it) {
				position[i] = *it;
				i++;
			}

			this->SetPositionTarget(position);
		}

		/// \brief Implement ROS initialization
		private: void RosInit()
		{
			
			ROS_INFO("Create ROS Node and subscribe to legs_cmd topic");
			
			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client",
					ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
				"/" + this->model->GetName() + "/legs_cmd",
				1,
				boost::bind(&TigrilloPlugin::OnRosMsg, this, _1),
				ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			std::thread(std::bind(&TigrilloPlugin::QueueThread, this));
		}
		
		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(TigrilloPlugin);
}
#endif
