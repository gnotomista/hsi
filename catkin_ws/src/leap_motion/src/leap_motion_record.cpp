#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <leap_motion_joints/Leap.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <vector>

using namespace Leap;

#define RECORD 1

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

class SampleListener : public Listener {
public:
  virtual void onInit(const Controller&);
  virtual void onConnect(const Controller&);
  virtual void onDisconnect(const Controller&);
  virtual void onExit(const Controller&);
  virtual void onFrame(const Controller&);
  virtual void onFocusGained(const Controller&);
  virtual void onFocusLost(const Controller&);
  virtual void onDeviceChange(const Controller&);
  virtual void onServiceConnect(const Controller&);
  virtual void onServiceDisconnect(const Controller&);
  bool publishing;
  std::vector<double> jntPos;

private:
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();
  std::vector<double> data;
//  system("clear");
  HandList hands = frame.hands();

  for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
#if RECORD
      std::ofstream outputFile("frame.txt", std::ios::app);
#endif
      const Hand hand = *hl;
      const FingerList fingers = hand.fingers();
      for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
          const Finger finger = *fl;
          //std::cout << std::setw(12) <<  fingerNames[finger.type()];
          for (int b = 0; b < 4; ++b) {

            Bone::Type boneType = static_cast<Bone::Type>(b);
            Bone bone = finger.bone(boneType);
            if (b != 0){
              Bone::Type previousBoneType = static_cast<Bone::Type>(b-1);
              Bone previousBone = finger.bone(previousBoneType);
              Leap::Matrix previousBoneRot  = previousBone.basis();
              previousBoneRot = previousBoneRot.rigidInverse();
              Leap::Vector dir = previousBoneRot.transformDirection(bone.direction());
              dir.z = -dir.z;
              float pitch = dir.pitch();
              float yaw = dir.yaw();

              if (b == 1)
              {
                //std::cout << std::fixed << std::setw(12) << yaw << " " << std::setw(12) << pitch;
                data.push_back(yaw);
                data.push_back(pitch);
#if RECORD
                outputFile << yaw << " " << pitch << " ";
#endif
            }
            else 
            {
                //std::cout << std::setw(12) << pitch ;
                data.push_back(pitch);
#if RECORD 
                outputFile << pitch << " ";
#endif            

            }
        }
    }
}
jntPos = data;
#if RECORD
outputFile << "\n";
outputFile.close();
#endif
publishing = true;
}
}

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
}
}

void SampleListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}


int main (int argc, char** argv){

    ros::init(argc,argv,"leap_motion_joints");
    ROS_INFO("leap_motion_joints.");
    ros::NodeHandle n;
    SampleListener listener;
    Controller controller;
    controller.addListener(listener);
    ros::Publisher leapJointsPub = n.advertise<sensor_msgs::JointState>("leap_motion_joints", 1);
    sensor_msgs::JointState jnts;
    listener.publishing = false;
    
    std::vector<float> offset(20,0.0);
    char cNum[20] ;
    int i=0;
    
    std::ifstream infile;
    infile.open ("offset.txt", std::ifstream::in);
    if (infile.is_open())
    {
        offset.clear();
        while (infile.good())
        {
            infile.getline(cNum, 256, ',');
            offset.push_back(atof(cNum));
            i++ ;
        }
        infile.close();
        std::cout << "offset loaded!" << std::endl;
    }
    else
        std::cout << "unable to load offset!" << std::endl;


    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (kbhit())
        {
            if (listener.publishing)
            {
                std::ofstream outputFile("offset.txt");
                if (outputFile.is_open())
                {
                    std::copy(listener.jntPos.begin(), listener.jntPos.end(), std::ostream_iterator<float>(outputFile , ", ")); 
                    outputFile.flush();
                }
                std::cout << "offset saved!" << std::endl;
            }
            else
                std::cout << "offset not saved!" << std::endl;
            break; //check get character
        }

        if (listener.publishing){
            std::transform(listener.jntPos.begin(), listener.jntPos.end(), offset.begin(), listener.jntPos.begin(), std::minus<float>());
            jnts.position = listener.jntPos;
            leapJointsPub.publish(jnts);
            listener.publishing = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
