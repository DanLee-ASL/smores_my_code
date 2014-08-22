#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <unistd.h>
#include <termios.h>

#include <MessageDefinition/command_message.pb.h>
#include <MessageDefinition/world_status_message.pb.h>
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

struct SMOREModuleInfo
{
    std::string moduleName;
    std::string lidarSubscriberName;
    gazebo::transport::SubscriberPtr lidarSubscriber;
    gazebo::msgs::Pose modulePose;
    gazebo::msgs::Pose lidarPose;
    bool lidarSubscriberInitialized;
    SMOREModuleInfo()
    {
      moduleName = "";
      lidarSubscriberName = "";
      lidarSubscriberInitialized = false;
    }
};

// global variables
bool foundNameIndices = false;
int poseNameIndex = -1;
int lidarPoseNameIndex = -1;
GLViewer *glViewer;

gazebo::math::Pose robotPose, lidarPose;
gazebo::transport::PublisherPtr worldMessagePublisher;
gazebo::transport::PublisherPtr occupiedCellPublisher;

gazebo::transport::NodePtr node;

double leftWheel = 0, rightWheel = 0, joint1 = 0, joint2 = 0;
bool doSwippingUD = false, doRotate = false;
std::string currentlySelectedTopicName;
std::map<std::string, SMOREControlCommand> topicNameToCommand;
std::map<std::string, SMOREModuleInfo> robotNameToInfo;

SmoresControllerWindow *gui;

typedef const boost::shared_ptr<const command_message::msgs::CommandMessage> CommandMessagePtr;
typedef const boost::shared_ptr<const command_message::msgs::WorldStatusMessage> WorldStatusMesagePtr;

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

  for(int i = 0; i < _msg->pose_size(); i++)
  {
//     std::cout << _msg->pose(i).name() << std::endl;
    gazebo::msgs::Pose poseMsg = _msg->pose(i);
    if(robotNameToInfo.count(poseMsg.name()) > 0)
    {
      robotNameToInfo[poseMsg.name()].modulePose = poseMsg;
//       std::cout << "GOT POSE" << std::endl;
    }
    std::string::size_type foundLidar = poseMsg.name().find("SMORES_LIDAR");
    if(foundLidar != std::string::npos)
    {
      // then find the name of the module
      std::string robotName = poseMsg.name().substr(0, poseMsg.name().find("::"));
      if(robotNameToInfo.count(robotName) > 0){
	robotNameToInfo[robotName].lidarPose = poseMsg;
// 	gazebo::math::Pose world_pose_lidar = gazebo::msgs::Convert(robotNameToInfo[robotName].lidarPose) + gazebo::msgs::Convert(robotNameToInfo[robotName].modulePose);
// 	glViewer->SetLidarPose(world_pose_lidar);	
// 	std::cout << "GOT LIDAR POSE" << std::endl;
      }
    }
  }
}

void lidarCallback(ConstLaserScanStampedPtr &msg) 
{
  gazebo::msgs::Pose world_pose_lidar =  msg->scan().world_pose();
  glViewer->SetLidarPose(gazebo::msgs::Convert(world_pose_lidar));
  glViewer->SetLidarPoints((double*)(msg->scan().ranges().data()), msg->scan().ranges_size(), msg->scan().angle_min(), msg->scan().angle_max(), msg->scan().angle_step(),
			   msg->scan().range_min(), msg->scan().range_max());
}

void SMORESWorldStatusCallback(WorldStatusMesagePtr &msg)
{
  	if(msg->messagetype() == 1)
	{
	  std::vector<std::string> names;
		for(int i = 0; i < msg->stringmessages_size(); i++)
		{
		  std::string name = msg->stringmessages(i);
			names.push_back(msg->stringmessages(i));
			bool foundSameTopicName = false, foundSameModuleName = false;
			
			for(std::map<std::string, SMOREControlCommand>::iterator it = topicNameToCommand.begin(); it != topicNameToCommand.end(); ++it)
			{
				if(it->first.compare(msg->stringmessages(i)) == 0)
				{
					foundSameTopicName = true;
					break;
				}
			}
			for(std::map<std::string, SMOREModuleInfo>::iterator it = robotNameToInfo.begin(); it != robotNameToInfo.end(); ++it)
			{
				if(it->first.compare(msg->stringmessages(i)) == 0)
				{
					foundSameModuleName = true;
					break;
				}
			}
			if(!foundSameTopicName)
			{
				topicNameToCommand[msg->stringmessages(i)] = SMOREControlCommand();
				topicNameToCommand[msg->stringmessages(i)].topicName = "/gazebo/" + name + "/" + name + "_world";
			}
			if(!foundSameModuleName)
			{
			  std::cout << msg->stringmessages(i) << std::endl;
			  robotNameToInfo[msg->stringmessages(i)] = SMOREModuleInfo();
			  robotNameToInfo[msg->stringmessages(i)].moduleName = msg->stringmessages(i);
			  robotNameToInfo[msg->stringmessages(i)].lidarSubscriberName = 
			    "/gazebo/default/" + msg->stringmessages(i) + "/SMORES_LIDAR/model/range_sensor/scan";
			  robotNameToInfo[msg->stringmessages(i)].lidarSubscriber = 
			      node->Subscribe<gazebo::msgs::LaserScanStamped>(robotNameToInfo[msg->stringmessages(i)].lidarSubscriberName, lidarCallback);
			  robotNameToInfo[msg->stringmessages(i)].lidarSubscriberInitialized = true;
			  std::cout << "Module Info initialized" << std::endl;
			}
		}
	  std::cout << "Set SMORE NAMES" << std::endl;
	  gui->SetGUIComponents(SmoresControllerWindow::SMORES_NAMES_LIST_VIEW, names);
	
	}
}

void GUIButtonCallback(SmoresControllerWindow::GUIBtnType type)
{
//     std::cout << "Got Message" << std::endl;
    currentlySelectedTopicName = gui->GetCurrentSelectedTopic();
	
    if(type == SmoresControllerWindow::GETMODULES)
    {
        command_message::msgs::WorldStatusMessage worldMsg;
        worldMsg.set_messagetype(1);
	std::cout << "Topic to send: " << worldMessagePublisher->GetTopic() << std::endl;
        worldMessagePublisher->Publish(worldMsg);
    }
    else if(type == SmoresControllerWindow::VEL_UP)
    {
	topicNameToCommand[currentlySelectedTopicName].velLeftWheel -= 0.1;
	topicNameToCommand[currentlySelectedTopicName].velRightWheel -= 0.1;
    }
    else if(type == SmoresControllerWindow::VEL_DOWN)
    {
        topicNameToCommand[currentlySelectedTopicName].velLeftWheel += 0.1;
        topicNameToCommand[currentlySelectedTopicName].velRightWheel += 0.1;
    }
    else if(type == SmoresControllerWindow::VEL_RIGHT)
    {
        topicNameToCommand[currentlySelectedTopicName].velLeftWheel -= 0.1;
        topicNameToCommand[currentlySelectedTopicName].velRightWheel += 0.1;
    }
    else if(type == SmoresControllerWindow::VEL_LEFT)
    {
        topicNameToCommand[currentlySelectedTopicName].velLeftWheel += 0.1;
        topicNameToCommand[currentlySelectedTopicName].velRightWheel -= 0.1;
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
    msg.add_jointgaittablestatus(false);
    msg.add_jointgaittablestatus(true);
    msg.add_jointgaittablestatus(true);
    msg.add_jointgaittablestatus(false);
    
//     msg.set_timer(1000);
	
    msg.add_jointgaittable(0);
    msg.add_jointgaittable(0);
    msg.add_jointgaittable(0);
    msg.add_jointgaittable(0);
    msg.set_messagetype(3);

    msg.set_jointgaittable(0, topicNameToCommand[currentlySelectedTopicName].angle1);
    msg.set_jointgaittable(1, topicNameToCommand[currentlySelectedTopicName].velLeftWheel);
    msg.set_jointgaittable(2, topicNameToCommand[currentlySelectedTopicName].velRightWheel);
    msg.set_jointgaittable(3, topicNameToCommand[currentlySelectedTopicName].angle2);

    node->Publish<command_message::msgs::CommandMessage>(topicNameToCommand[currentlySelectedTopicName].topicName, msg);
    std::printf("Topic: %s\tJ1: %f\tJ2: %f\tLW: %f\tRW:%f\n", 
				topicNameToCommand[currentlySelectedTopicName].topicName.c_str(),
				topicNameToCommand[currentlySelectedTopicName].angle1, 
				topicNameToCommand[currentlySelectedTopicName].angle2, 
				topicNameToCommand[currentlySelectedTopicName].velLeftWheel, 
				topicNameToCommand[currentlySelectedTopicName].velRightWheel);
}


void OccupiedCellUpdatedCallback(std::vector<octomap::point3d> points)
{
    std::cout << "GOT Voxels" << std::endl;
    for(int i = 0; i < points.size(); i++)
    {
        gazebo::msgs::Vector3d v;
        v.set_x(points[i].x());
        v.set_y(points[i].y());
        v.set_z(points[i].z());
        occupiedCellPublisher->Publish(v);
    }
}

void GLViewerThread(int argc, char** argv) {
  try{
    glViewer = new GLViewer();
    glViewer->occupiedCellUpdated.connect(boost::bind(OccupiedCellUpdatedCallback, _1));
    glViewer->Run(argc, argv);
  }
  catch(boost::thread_interrupted&) {
    std::cout << "Thread is stopped" << std::endl;
    return;
  }
}

void SwipeThread() {
	
	command_message::msgs::CommandMessage msg;
	msg.add_jointgaittablestatus(false);
	msg.add_jointgaittablestatus(false);
	msg.add_jointgaittablestatus(false);
	msg.add_jointgaittablestatus(false);
	
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
			msg.set_messagetype(3);
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
//     gazebo::load(argc, argv);
//     gazebo::run();

    gazebo::setupClient(argc, argv);
    gazebo::physics::WorldPtr worldPtr;

    // create our node for communication
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init("Octomap");

    // subscribe to Pose topic
    std::string poseName = "/gazebo/default/pose/info";
    gazebo::transport::SubscriberPtr poseSub = node->Subscribe<gazebo::msgs::PosesStamped>(poseName, poseCallback);
	
    // world status message 
    worldMessagePublisher = node->Advertise<command_message::msgs::WorldStatusMessage>("/gazebo/default/SMORES_WorldMessage");
    gazebo::transport::SubscriberPtr worldMessageSubscriber = node->Subscribe<command_message::msgs::WorldStatusMessage>("/gazebo/default/SMORES_WorldStatus", SMORESWorldStatusCallback);
        
    // occupied cell message
    occupiedCellPublisher = node->Advertise<gazebo::msgs::Vector3d>("~/OccupiedCellsVector3d");
    
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
				GUIButtonCallback(SmoresControllerWindow::VEL_UP);
				break;
			
			case 'k':
				GUIButtonCallback(SmoresControllerWindow::VEL_DOWN);
				break;
			
			case 'j':
				GUIButtonCallback(SmoresControllerWindow::VEL_LEFT);
				break;
			
			case 'l':
				GUIButtonCallback(SmoresControllerWindow::VEL_RIGHT);
				break;
			
			case 's':
				GUIButtonCallback(SmoresControllerWindow::STOP);
				break;
			
			case 'r':
				GUIButtonCallback(SmoresControllerWindow::JOINT_A1_M);
				break;
			
			case 't':
				GUIButtonCallback(SmoresControllerWindow::JOINT_A1_P);
				break;
			
			case 'y':
				GUIButtonCallback(SmoresControllerWindow::JOINT_A2_M);
				break;
			case 'u':
				GUIButtonCallback(SmoresControllerWindow::JOINT_A2_P);
				break;
			case 'q':
				GUIButtonCallback(SmoresControllerWindow::JOINT_A1_TOGGLE);
				break;
			case 'w':
				GUIButtonCallback(SmoresControllerWindow::JOINT_A2_TOGGLE);
				break;
			case 'z':
				isRunning = false;
				break;
		}
				
		gazebo::common::Time::MSleep(100);
    }
    glViewer->~GLViewer();
    gazebo::transport::fini();
    
}

