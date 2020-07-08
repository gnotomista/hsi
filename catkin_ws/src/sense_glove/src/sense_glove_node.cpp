#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <DeviceList.h>
#include <SenseGlove.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>

std::vector<int> jointForcesLevels;

void callbackForceFeedback(const sensor_msgs::JointState::ConstPtr& jointForces) {
	jointForcesLevels.clear();
	for (int i = 0; i < jointForces->effort.size(); i++) {
		jointForcesLevels.push_back(int(jointForces->effort[i]));
	}
}

int main (int argc, char** argv) {
	// init ros node
	ros::init(argc, argv, "sense_glove_node");
	ros::NodeHandle n;
	ros::Rate rate(50);

	// publisher
	ros::Publisher publisherJoints = n.advertise<sensor_msgs::JointState>("sense_glove_joints", 1);
	ros::Publisher publisherImu = n.advertise<geometry_msgs::Pose>("sense_glove_imu", 1);
	sensor_msgs::JointState messageJoints;
	geometry_msgs::Pose messageImu;

	// subscriber
	ros::Subscriber subscriberForces = n.subscribe("sense_glove_forces", 1, callbackForceFeedback);

	// init SenseGlove
	SGCore::SG::SenseGlove senseGlove;
	SGCore::SG::SG_SensorData senseGloveSensorData;
	SGCore::SG::SG_GlovePose senseGlovePose;

	if (!SGCore::DeviceList::SenseCommRunning()) {
		std::cout << "SenseCom not running. Please start SenseCom before launching this node." << std::endl;
		return -1;
	}

	if (!SGCore::SG::SenseGlove::GetSenseGlove(senseGlove)) {
		std::cout << "No SenseGlove connected. Ensure you have rights to access the USB device." << std::endl;
		return -1;
	}

	std::cout << "------------------------------" << std::endl;
	std::cout << "Activating " << senseGlove.ToString() << std::endl;

	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(50, 0, 0, 0, 0));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(0, 50, 0, 0, 0));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(0, 0, 50, 0, 0));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(0, 0, 0, 50, 0));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(0, 0, 0, 0, 50));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	SGCore::SG::SG_Model model = senseGlove.GetGloveModel();
	std::cout << model.ToString(true) << std::endl;
	std::cout << "------------------------------" << std::endl;

	// main loop
	while (ros::ok()) {
		// send forces to SenseGlove
		if (!jointForcesLevels.empty()) {
			senseGlove.SendHaptics(SGCore::Haptics::SG_FFBCmd(jointForcesLevels));
		}

		// read joint angles and pose from SenseGlove
		messageJoints.position.clear();
		if (senseGlove.GetSensorData(senseGloveSensorData)) {
			std::vector<float> allJoints = senseGloveSensorData.GetAngleSequence();
			for (int i = 0; i < allJoints.size(); i++) {
				messageJoints.position.push_back(allJoints[i]); // LHS is double, RHS is float
			}
			messageImu.orientation.x = senseGloveSensorData.imuValues.x;
			messageImu.orientation.y = senseGloveSensorData.imuValues.y;
			messageImu.orientation.z = senseGloveSensorData.imuValues.z;
			messageImu.orientation.w = senseGloveSensorData.imuValues.w;

			// publish joint angles and pose of SenseGlove
			publisherJoints.publish(messageJoints);
			publisherImu.publish(messageImu);
		}

		rate.sleep();
		ros::spinOnce();
	}

	std::cout << "Deactivating Sense Glove..." << std::flush;
	senseGlove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off);
	senseGlove.SendHaptics(SGCore::Haptics::SG_FFBCmd::off);
	std::cout << "done" << std::endl;

	return 0;
}
