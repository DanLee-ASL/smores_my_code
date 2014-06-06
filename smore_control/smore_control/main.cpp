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

// global variables
boost::shared_ptr<google::protobuf::Message> g_echoMsg;
bool foundNameIndices = false;
int poseNameIndex = -1;
int lidarPoseNameIndex = -1;
GLViewer* glViewer;

gazebo::math::Pose robotPose, lidarPose;
gazebo::transport::PublisherPtr pub;

double leftWheel = 0, rightWheel = 0, joint1 = 0, joint2 = 0;
bool doSwippingUD = false, doRotate = false;

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
      
//       std::cout << "Robot" << std::endl << (gazebo::msgs::Convert(robotPose).DebugString()) << std::endl;
//       std::cout << "LIDAR" << std::endl << (gazebo::msgs::Convert(lidarPose).DebugString()) << std::endl;
//       std::cout << "Robot + LIDAR" << std::endl << (gazebo::msgs::Convert(lidarPose + robotPose).DebugString()) << std::endl;
    }
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {
  msg->scan().ranges().data();
  glViewer->SetLidarPoints((double*)(msg->scan().ranges().data()), msg->scan().ranges_size(), msg->scan().angle_min(), msg->scan().angle_max(), msg->scan().angle_step(),
			   msg->scan().range_min(), msg->scan().range_max());
  
}

void SMORESWorldStatusCallback(WorldMessagePtr &msg)
{
	if(msg->messagetype() == 11)
	{
		for(int i = 0; i < msg->stringmessages_size(); i++)
		{
			std::cout << "Name: " << msg->stringmessages(i) << std::endl;
		}
	}
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
	
	bool movingUp = false;
	while(true)
	{
		if(doSwippingUD)
		{
			if(movingUp)
			{
				joint2 += 0.05;
				if(joint2 > 1.5)
					movingUp = false;
			}
			else
			{
				joint2 -= 0.05;
				if(joint2 < 0.35)
					movingUp = true;
			}
		}
		if(doRotate)
		{
			joint1 += 0.01;
		}
		msg.set_messagetype(4);
		msg.set_jointgaittable(0, joint1); 
		msg.set_jointgaittable(1, leftWheel);
		msg.set_jointgaittable(2, rightWheel);
		msg.set_jointgaittable(3, joint2);
		if(doSwippingUD || doRotate)
			pub->Publish(msg);
		boost::this_thread::sleep(boost::posix_time::millisec(100));
	}
}

void SmoresControlGUI(int argc, char **argv)
{
    QApplication app(argc, argv);
    SmoresControllerWindow window;
    window.show();
    app.exec();
}

int main(int argc, char **argv) {
    
    // load Gazebo
    gazebo::load(argc, argv);
    gazebo::run();


    gazebo::physics::WorldPtr worldPtr;

    // create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
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
	
	gazebo::transport::PublisherPtr worldMessagePublisher = node->Advertise<command_message::msgs::WorldMessage>("/gazebo/default/SMORES_WorldMessage");
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
	
			pub->Publish(msg);
			std::printf("J1: %f\tJ2: %f\tLW: %f\tRW:%f\n", joint1, joint2, leftWheel, rightWheel);
		}
		
		command_message::msgs::WorldMessage worldMsg;
		worldMsg.set_messagetype(1);
		worldMessagePublisher->Publish(worldMsg);
		std::cout << "Published" << std::endl;
		
		gazebo::common::Time::MSleep(100);
    }
    glViewer->~GLViewer();
    gazebo::transport::fini();
    
}

