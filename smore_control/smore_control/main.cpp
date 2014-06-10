#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <unistd.h>
#include <termios.h>

#include <msgs/command_message.pb.h>
#include <msgs/world_command_message.pb.h>
#include <boost/thread.hpp>

#include <iostream>
#include "GLViewer.h"
#include "smorescontrollerwindow.h"
#include <QtGui>

struct SMOREControlCommand
{
	float velLeftWheel, velRightWheel;
	float angle1, angle2;
	std::string topicName;
	bool doRotate;
	bool doSwipeUD;
	bool movingUp;
	
	SMOREControlCommand()
	{
		velLeftWheel = 0;
		velRightWheel = 0;
		angle1 = 0;
		angle2 = 0;
		doRotate = false;
		doSwipeUD = false;
		movingUp = false;
		topicName = "";
	}
};

// global variables
bool foundNameIndices = false;
int poseNameIndex = -1;
int lidarPoseNameIndex = -1;
GLViewer *glViewer;

gazebo::math::Pose robotPose, lidarPose;
gazebo::transport::PublisherPtr pub;
gazebo::transport::PublisherPtr worldMessagePublisher;

gazebo::transport::NodePtr node;

double leftWheel = 0, rightWheel = 0, joint1 = 0, joint2 = 0;
bool doSwippingUD = false, doRotate = false;
std::string currentlySelectedTopicName;
std::map<std::string, SMOREControlCommand> topicNameToCommand;

SmoresControllerWindow *gui;

typedef const boost::shared_ptr<const command_message::msgs::WorldMessage> WorldMessagePtr;

char getch() {
  
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

void poseCallback(ConstPosesStampedPtr &_msg) {
//     std::cout << _msg->time().DebugString() << std::endl;
    if(!foundNameIndices) {
      for(int i = 0; i < _msg->pose_size(); i++) {
	gazebo::msgs::Pose poseMsg = _msg->pose(i);
	if(poseMsg.name().compare("SMORES6Uriah") == 0) {
	  poseNameIndex = i;
	}
	if(poseMsg.name().compare("SMORES6Uriah::SMORES_LIDAR::model") == 0) {
	  lidarPoseNameIndex = i;
	}
      }
      if(poseNameIndex != -1 && lidarPoseNameIndex != -1)
	foundNameIndices = true;
    }
    else{
      robotPose = gazebo::msgs::Convert(_msg->pose(poseNameIndex));
      lidarPose = gazebo::msgs::Convert(_msg->pose(lidarPoseNameIndex));
      
      glViewer->SetLidarPose(lidarPose + robotPose);
    }
}

void lidarCallback(ConstLaserScanStampedPtr &msg) 
{
  msg->scan().ranges().data();
  glViewer->SetLidarPoints((double*)(msg->scan().ranges().data()), msg->scan().ranges_size(), msg->scan().angle_min(), msg->scan().angle_max(), msg->scan().angle_step(),
			   msg->scan().range_min(), msg->scan().range_max());
}

void SMORESWorldStatusCallback(WorldMessagePtr &msg)
{
	if(msg->messagetype() == 11)
	{
        std::vector<std::string> names;
		for(int i = 0; i < msg->stringmessages_size(); i++)
		{
			names.push_back(msg->stringmessages(i));
			bool foundSameTopicName = false;
			
			for(std::map<std::string, SMOREControlCommand>::iterator it = topicNameToCommand.begin(); it != topicNameToCommand.end(); ++it)
			{
				if(it->first.compare(msg->stringmessages(i)) == 0)
				{
					foundSameTopicName = true;
					break;
				}
			}
			if(!foundSameTopicName)
			{
				topicNameToCommand[msg->stringmessages(i)] = SMOREControlCommand();
			}
		}
        std::cout << "Set SMORE NAMES" << std::endl;
        gui->SetGUIComponents(SmoresControllerWindow::SMORES_NAMES_LIST_VIEW, names);
	}
}

void GUIButtonCallback(SmoresControllerWindow::GUIBtnType type)
{
    std::cout << "Got Message" << std::endl;
	currentlySelectedTopicName = gui->GetCurrentSelectedTopic();
	
    if(type == SmoresControllerWindow::GETMODULES)
    {
        command_message::msgs::WorldMessage worldMsg;
        worldMsg.set_messagetype(1);
        worldMessagePublisher->Publish(worldMsg);
    }
    else if(type == SmoresControllerWindow::VEL_UP)
    {
		topicNameToCommand[currentlySelectedTopicName].velLeftWheel += 0.1;
		topicNameToCommand[currentlySelectedTopicName].velRightWheel += 0.1;
//         leftWheel += 0.1;
//         rightWheel += 0.1;
    }
    else if(type == SmoresControllerWindow::VEL_DOWN)
    {
        topicNameToCommand[currentlySelectedTopicName].velLeftWheel -= 0.1;
        topicNameToCommand[currentlySelectedTopicName].velRightWheel -= 0.1;
    }
    else if(type == SmoresControllerWindow::VEL_RIGHT)
    {
        topicNameToCommand[currentlySelectedTopicName].velLeftWheel += 0.1;
        topicNameToCommand[currentlySelectedTopicName].velRightWheel -= 0.1;
    }
    else if(type == SmoresControllerWindow::VEL_LEFT)
    {
        topicNameToCommand[currentlySelectedTopicName].velLeftWheel -= 0.1;
        topicNameToCommand[currentlySelectedTopicName].velRightWheel += 0.1;
    }
    else if(type == SmoresControllerWindow::STOP)
	{
		topicNameToCommand[currentlySelectedTopicName].velLeftWheel = 0;
		topicNameToCommand[currentlySelectedTopicName].velRightWheel = 0;
	}
    else if(type == SmoresControllerWindow::JOINT_A1_P)
    {
        topicNameToCommand[currentlySelectedTopicName].angle1 += 0.1;
    }
    else if(type == SmoresControllerWindow::JOINT_A1_M)
    {
        topicNameToCommand[currentlySelectedTopicName].angle1 -= 0.1;
    }
    else if(type == SmoresControllerWindow::JOINT_A2_P)
    {
        topicNameToCommand[currentlySelectedTopicName].angle2 += 0.1;
    }
    else if(type == SmoresControllerWindow::JOINT_A2_M)
    {
        topicNameToCommand[currentlySelectedTopicName].angle2 -= 0.1;
    }
    else if(type == SmoresControllerWindow::JOINT_A1_TOGGLE)
    {
        topicNameToCommand[currentlySelectedTopicName].doRotate = !topicNameToCommand[currentlySelectedTopicName].doRotate;
    }
    else if(type == SmoresControllerWindow::JOINT_A2_TOGGLE)
    {
        topicNameToCommand[currentlySelectedTopicName].doSwipeUD = !topicNameToCommand[currentlySelectedTopicName].doSwipeUD;
    }
    command_message::msgs::CommandMessage msg;
	msg.add_jointgaittablestatus(true);
	msg.add_jointgaittablestatus(true);
	msg.add_jointgaittablestatus(true);
	msg.add_jointgaittablestatus(true);
	
	msg.add_jointgaittable(0);
	msg.add_jointgaittable(0);
	msg.add_jointgaittable(0);
	msg.add_jointgaittable(0);
    msg.set_messagetype(4);

    msg.set_jointgaittable(0, topicNameToCommand[currentlySelectedTopicName].angle1);
    msg.set_jointgaittable(1, topicNameToCommand[currentlySelectedTopicName].velLeftWheel);
    msg.set_jointgaittable(2, topicNameToCommand[currentlySelectedTopicName].velRightWheel);
    msg.set_jointgaittable(3, topicNameToCommand[currentlySelectedTopicName].angle2);

	node->Publish<command_message::msgs::CommandMessage>(currentlySelectedTopicName, msg);
    std::printf("Topic: %s\tJ1: %f\tJ2: %f\tLW: %f\tRW:%f\n", 
				topicNameToCommand[currentlySelectedTopicName].topicName.c_str(),
				topicNameToCommand[currentlySelectedTopicName].angle1, 
				topicNameToCommand[currentlySelectedTopicName].angle2, 
				topicNameToCommand[currentlySelectedTopicName].velLeftWheel, 
				topicNameToCommand[currentlySelectedTopicName].velRightWheel);
}

void GLViewerThread(int argc, char** argv) {
  try{
    glViewer = new GLViewer();
    glViewer->Run(argc, argv);
  }
  catch(boost::thread_interrupted&) {
    std::cout << "Thread is stopped" << std::endl;
    return;
  }
}

void SwipeThread() {
	
	command_message::msgs::CommandMessage msg;
	msg.add_jointgaittablestatus(true);
	msg.add_jointgaittablestatus(true);
	msg.add_jointgaittablestatus(true);
	msg.add_jointgaittablestatus(true);
	
	msg.add_jointgaittable(0);
	msg.add_jointgaittable(0);
	msg.add_jointgaittable(0);
	msg.add_jointgaittable(0);
	
	while(true)
	{
		for(std::map<std::string, SMOREControlCommand>::iterator it = topicNameToCommand.begin(); it != topicNameToCommand.end(); ++it)
		{
			if(it->second.doSwipeUD)
			{
				if(it->second.movingUp)
				{
					it->second.angle2 += 0.025;
					if(it->second.angle2 > 1.5)
						it->second.movingUp = false;
				}
				else
				{
					it->second.angle2 -= 0.025;
					if(it->second.angle2 < 0.35)
						it->second.movingUp = true;
				}
			}
			if(it->second.doRotate)
			{
				it->second.angle1 += 0.01;
			}
			msg.set_messagetype(4);
			msg.set_jointgaittable(0, it->second.angle1); 
			msg.set_jointgaittable(1, it->second.velLeftWheel);
			msg.set_jointgaittable(2, it->second.velRightWheel);
			msg.set_jointgaittable(3, it->second.angle2);
			if(it->second.doSwipeUD || it->second.doRotate)
				node->Publish<command_message::msgs::CommandMessage>(it->first, msg);
// 			pub->Publish(msg);
		}
		boost::this_thread::sleep(boost::posix_time::millisec(100));
	}
}

void SmoresControlGUI(int argc, char **argv)
{   
    QApplication app(argc, argv);
	SmoresControllerWindow window;
	gui = &window;
    // event handler setup
    window.btnCalled.connect(boost::bind(GUIButtonCallback, _1));

    window.show();	
	app.exec();
}

int main(int argc, char **argv) {
    
    // load Gazebo
    gazebo::load(argc, argv);
    gazebo::run();

    gazebo::physics::WorldPtr worldPtr;

    // create our node for communication
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init("SMORES6Uriah");

	// publish to a Gazebo topic
    std::string robotName = "SMORES6Uriah";
    std::string pubName = "~/" + robotName + "_world";
	pub = node->Advertise<command_message::msgs::CommandMessage>(pubName);

    // subscribe to Pose topic
    std::string poseName = "/gazebo/default/pose/info";
    gazebo::transport::SubscriberPtr poseSub = node->Subscribe<gazebo::msgs::PosesStamped>(poseName, poseCallback);

    // subscribe to LIDAR message
    std::string lidarName = "/gazebo/default/SMORES6Uriah/SMORES_LIDAR/model/LIDAR_sensor/scan";
    gazebo::transport::SubscriberPtr lidarSub = node->Subscribe<gazebo::msgs::LaserScanStamped>(lidarName, lidarCallback);
	
    worldMessagePublisher = node->Advertise<command_message::msgs::WorldMessage>("/gazebo/default/SMORES_WorldMessage");
	gazebo::transport::SubscriberPtr worldMessageSubscriber = node->Subscribe<command_message::msgs::WorldMessage>("/gazebo/default/SMORES_WorldStatus", SMORESWorldStatusCallback);
        
    std::cout << "getting in to the loop..." << std::endl;
    command_message::msgs::CommandMessage msg;
    msg.add_jointgaittablestatus(true);
    msg.add_jointgaittablestatus(true);
    msg.add_jointgaittablestatus(true);
    msg.add_jointgaittablestatus(true);
    
    msg.add_jointgaittable(0);
    msg.add_jointgaittable(0);
    msg.add_jointgaittable(0);
    msg.add_jointgaittable(0);
    
	boost::thread t(&GLViewerThread, argc, argv);
	boost::thread swipeThread(&SwipeThread);
    boost::thread smoresControlGUIThread(&SmoresControlGUI, argc, argv);
	
    char key_pressed;
	bool isRunning = true;
    while(isRunning)
    {
		key_pressed = getch();
		bool changed = false;
		switch(key_pressed)
		{
			case 'i':
				leftWheel += 0.1;
				rightWheel += 0.1;
				changed = true;
				break;
			
			case 'k':
				leftWheel -= 0.1;
				rightWheel -= 0.1;
				changed = true;
				break;
			
			case 'j':
				leftWheel -= 0.1;
				rightWheel += 0.1;
				changed = true;
				break;
			
			case 'l':
				leftWheel += 0.1;
				rightWheel -= 0.1;
				changed = true;
				break;
			
			case 's':
				leftWheel = 0.0;
				rightWheel = 0.0;
				changed = true;
				break;
			
			case 'r':
				joint1 += 0.1;
				changed = true;
				break;
			
			case 't':
				joint1 -= 0.1;
				changed = true;
				break;
			
			case 'y':
				joint2 += 0.1;
				changed = true;
				break;
			case 'u':
				joint2 -= 0.1;
				changed = true;
				break;
			case 'q':
				doSwippingUD = !doSwippingUD;
				changed = true;
				break;
			case 'w':
				doRotate = !doRotate;
				changed = true;
				break;
			case 'z':
				isRunning = false;
				break;
		}
		if(changed)
		{
			msg.set_messagetype(4);
		
			msg.set_jointgaittable(0, joint1);
			msg.set_jointgaittable(1, leftWheel);
			msg.set_jointgaittable(2, rightWheel);
			msg.set_jointgaittable(3, joint2);
	
			node->Publish<command_message::msgs::CommandMessage>(currentlySelectedTopicName, msg);
			std::printf("J1: %f\tJ2: %f\tLW: %f\tRW:%f\n", joint1, joint2, leftWheel, rightWheel);
		}
				
		gazebo::common::Time::MSleep(100);
    }
    glViewer->~GLViewer();
    gazebo::transport::fini();
    
}

