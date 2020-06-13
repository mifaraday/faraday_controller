/*
Created on Wed Sep 20 11:30:15 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include "faraday_controller/fy_hardware_interface.h"

boost::mutex cute_io_mutex;

FaradayHWInterface::FaradayHWInterface()
{
    for(int i=0; i<5; i++)
    {
        std::string joint_name="joint";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_name.append(joint_num);
        joint_names_.push_back(joint_name);

        client_controller_list=nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
        loaded_flag=false;
    }
    joint_command_pub_=nh_.advertise<sensor_msgs::JointState>("/slide_block/joint_command", 1000);
    joint_state_sub_=nh_.subscribe("/slide_block/joint_states", 1000, &FaradayHWInterface::read, this);

    // Resize vectors
    num_joints_=joint_names_.size();
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);

    for(int i=0;i<num_joints_;i++)
    {
        joint_velocity_[i]=0;
        joint_effort_[i]=0;
        joint_velocity_command_[i]=0;
        joint_effort_command_[i]=0;
    }
    joint_position_[0]=227.139;
    joint_position_[1]=225.652;
    joint_position_[2]=225.652;
    joint_position_[3]=227.139;
    joint_position_[4]=0.0;

    joint_position_command_[0]=227.139;
    joint_position_command_[1]=225.652;
    joint_position_command_[2]=225.652;
    joint_position_command_[3]=227.139;
    joint_position_command_[4]=0.0;

    last_pose[0]=0.0;
    last_pose[1]=0.0;
    last_pose[2]=390.0;
    last_pose[3]=0.0;
    last_pose[4]=0.0;

    //resize topic message size
    slide_block_msg.name.resize(num_joints_+2);
    slide_block_msg.position.resize(num_joints_+2);
    slide_block_msg.velocity.resize(num_joints_+2);
    slide_block_msg.effort.resize(num_joints_+2);

    // Initialize controller
	for (std::size_t i = 0; i < num_joints_; ++i) 
    {
		ROS_DEBUG_STREAM_NAMED("fy_hardware_interface",
				"Loading joint name: " << joint_names_[i]);

		// Create joint state interface
		joint_state_interface_.registerHandle(
				hardware_interface::JointStateHandle(joint_names_[i],
						&joint_position_[i], &joint_velocity_[i],
						&joint_effort_[i]));

		// Create position joint interface
		position_joint_interface_.registerHandle(
				hardware_interface::JointHandle(
						joint_state_interface_.getHandle(joint_names_[i]),
						&joint_position_command_[i]));

        // Create velocity joint interface
		velocity_joint_interface_.registerHandle(
				hardware_interface::JointHandle(
						joint_state_interface_.getHandle(joint_names_[i]),
						&joint_velocity_command_[i]));

        // Create effort joint interface
		effort_joint_interface_.registerHandle(
				hardware_interface::JointHandle(
						joint_state_interface_.getHandle(joint_names_[i]),
						&joint_effort_command_[i]));
	}
    registerInterface(&joint_state_interface_); // From RobotHW base class.
	registerInterface(&position_joint_interface_); // From RobotHW base class
    registerInterface(&velocity_joint_interface_); // From RobotHW base class
    registerInterface(&effort_joint_interface_); // From RobotHW base class

    //joint_limits
    for(std::size_t i=0;i<num_joints_;++i)
    {
        JointLimits limits_;

        //通过rosparam获取limits
        const bool rosparam_limits_ok = getJointLimits(joint_names_[i], nh_, limits_);

        hardware_interface::JointHandle joint_handle=position_joint_interface_.getHandle(joint_names_[i]);
        PositionJointSaturationHandle handle(joint_handle,limits_);
        joint_limits_interface_.registerHandle(handle);   
    }
    // registerInterface(&joint_limits_interface_);
    

    start_time_=ros::Time::now();
    start_dur_.operator +=(ros::Duration(3));
}

void FaradayHWInterface::publishCmd()
{
    if(ros::Time::now()-start_time_<start_dur_)
        return;

    joint_limits_interface_.enforceLimits(start_dur_);
    // ROS_INFO_STREAM("start_dur_"<<start_dur_.toSec());
    // joint_limits_interface_.enforceLimits(ros::Duration(10.0));

    for (unsigned int i = 0; i < num_joints_; i++)
    {
        slide_block_msg.name[i]=joint_names_[i];
        slide_block_msg.position[i]=joint_position_command_[i];     
        slide_block_msg.velocity[i]=joint_velocity_command_[i];
        slide_block_msg.effort[i]=joint_effort_command_[i];
    }
    
    double actuators[5];
	for (int i = 0; i < 5; ++i)
		actuators[i]=joint_position_command_[i];

    double pose[5];
	if(!faraday_kinematics::forward_kinematics(actuators,pose,last_pose))
    {
        ROS_WARN("Could not calculate FK for given pose");
        return;
    }

    for (int i = 0; i < 3; ++i)
    	last_pose[i]=pose[i];

    faraday_kinematics::IntermediatePoints pts;
    faraday_kinematics::calcIntermediatePoints(actuators,pose,pts);

    slide_block_msg.name[5]="joint6";
    slide_block_msg.name[6]="joint7";
    slide_block_msg.position[5]=pts.C[1];
    slide_block_msg.position[6]=pts.C[2];

    slide_block_msg.header.stamp=ros::Time::now();


	joint_command_pub_.publish(slide_block_msg);
    
}

void FaradayHWInterface::read(const sensor_msgs::JointStateConstPtr& msg)
{
    std::vector<double> pos, vel, current;
    boost::mutex::scoped_lock lock(cute_io_mutex);

	pos=msg->position;
	vel=msg->velocity;
	current=msg->effort;

    for (std::size_t i = 0; i < num_joints_; ++i)
	{
		joint_position_[i] = pos[i];
		joint_velocity_[i] = vel[i];
		joint_effort_[i] = current[i];
	}
}

ros::NodeHandle FaradayHWInterface::getnode()
{
    return nh_;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

void* update_loop(void* threadarg)
{
    controller_manager::ControllerManager *c=(controller_manager::ControllerManager *)threadarg;
    ros::Rate r(50);
    ros::Duration d(0.02);

    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(cute_io_mutex);
        c->update(ros::Time::now(), d);
        lock.unlock();
        r.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"fy_hardware_interface", ros::init_options::AnonymousName);
    FaradayHWInterface c1;

    // init joint states
    ros::Rate r_tmp(10);
    for(int i=0; i<50; i++)
    {
        ros::spinOnce();
        r_tmp.sleep();
        if(i%10==0)
            ROS_INFO("fy bring up in %i", (10-i/10));
    }

    controller_manager::ControllerManager cm(&c1);
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, (void *)&cm);

    for(int i=0; i<50; i++)
    {
        ros::spinOnce();
        r_tmp.sleep();
        if(i%10==0)
            ROS_INFO("fy bring up in %i", (5-i/10));
    }
    ROS_INFO("fy bring up successfully");
    // loop
    ros::Rate r(50);
    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(cute_io_mutex);
        c1.publishCmd();
        lock.unlock();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
