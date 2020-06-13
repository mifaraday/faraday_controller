#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,
			const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
	ROS_INFO_STREAM("Yay! The goal is done.");
}

void activeCb()
{
	ROS_INFO("Goal just went active");
}

void trajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr& traj,Client& client_1)
{	
	control_msgs::FollowJointTrajectoryGoal arm_goal;
	arm_goal.trajectory=*traj;
	arm_goal.goal_time_tolerance=ros::Duration(0.0);

	client_1.sendGoal(arm_goal,&doneCb,&activeCb);

    bool isSuccess=client_1.waitForResult();

	if(isSuccess)
		ROS_INFO_STREAM("...done");
	else
		ROS_INFO_STREAM("Action did not finish before the time out.");
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"trajectory_client_test");
	ros::NodeHandle nh;

	Client client("fy_pm_controller/follow_joint_trajectory",true);

	ros::Subscriber sub=nh.subscribe<trajectory_msgs::JointTrajectory>("joint_path_command",1000,boost::bind(trajectoryCallback,_1,boost::ref(client)));

	ROS_INFO("Waiting for action server to start.");
	client.waitForServer();
	ROS_INFO("Action server started,sending goal.");

	ros::spin();

	return 0;
}
