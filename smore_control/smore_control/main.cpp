#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <math/gzmath.hh>
#include <unistd.h>
#include <termios.h>

#include "command_message.pb.h"

#include <iostream>


boost::shared_ptr<google::protobuf::Message> g_echoMsg;

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

bool foundNameIndex = false;
int nameIndex = 0;
void poseCallback(ConstPosesStampedPtr &_msg) {
//     std::cout << _msg->time().DebugString() << std::endl;
    if(!foundNameIndex) {
      for(int i = 0; i < _msg->pose_size(); i++) {
	gazebo::msgs::Pose poseMsg = _msg->pose(i);
	if(poseMsg.name().compare("SMORES6Uriah") == 0) {
	  foundNameIndex = true;
	  nameIndex = i;
	}
      }
    }
    else{
//       printf("Pose: x:%f\ty:%f\tz:%f, | x:%f\ty:%f\tz:%f\tw:%f\n", _msg->pose(nameIndex).position().x(), _msg->pose(nameIndex).position().y(), _msg->pose(nameIndex).position().z(),
// 	     _msg->pose(nameIndex).orientation().x(), _msg->pose(nameIndex).orientation().y(), _msg->pose(nameIndex).orientation().z(), _msg->pose(nameIndex).orientation().w());
    }
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {
  
  int numPoint = msg->scan().ranges_size();
  for(int i = 0; i < numPoint; i++) {
    std::cout << msg->scan().ranges(i) << "\t";
  }
  
}


int main(int argc, char **argv) {
    
    // load Gazebo
    gazebo::load(argc, argv);
    gazebo::run();
    
    // create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init("SMORES6Uriah");
    
    // start transport(?)? what the hack is transport...?
    
    // publish to a Gazebo topic
    std::string robotName = "SMORES6Uriah";
    std::string pubName = "~/" + robotName + "_world";
    gazebo::transport::PublisherPtr pub = node->Advertise<command_message::msgs::CommandMessage>(pubName);
//     pub->WaitForConnection();
    
    // subscribe to Pose topic
     std::string poseName = "/gazebo/default/pose/info";
     gazebo::transport::SubscriberPtr poseSub = node->Subscribe<gazebo::msgs::PosesStamped>(poseName, poseCallback);

    // subscribe to LIDAR message
     std::string lidarName = "/gazebo/default/SMORES6Uriah/SMORES_LIDAR/model/range_sensor/scan";
     gazebo::transport::SubscriberPtr lidarSub = node->Subscribe<gazebo::msgs::LaserScanStamped>(lidarName, lidarCallback);
    
        
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
    
    char key_pressed;
    double leftWheel = 0, rightWheel = 0, joint1 = 0, joint2 = 0;
    while(true)
    {
//       std::cin >> key_pressed;
      key_pressed = getch();
      switch(key_pressed)
      {
	case 'i':
	  leftWheel += 0.1;
	  rightWheel += 0.1;
	  break;
	  
	case 'k':
	  leftWheel -= 0.1;
	  rightWheel -= 0.1;
	  break;
	  
	case 'j':
	  leftWheel -= 0.1;
	  rightWheel += 0.1;
	  break;
	  
	case 'l':
	  leftWheel += 0.1;
	  rightWheel -= 0.1;
	  break;
	  
	case 's':
	  leftWheel = 0.0;
	  rightWheel = 0.0;
	  break;
	  
	case 'r':
	  joint1 += 0.1;
	  break;
	  
	case 't':
	  joint1 -= 0.1;
	  break;
	  
	case 'y':
	  joint2 += 0.1;
	  break;
	case 'u':
	  joint2 -= 0.1;
	  break;
      }
      msg.set_messagetype(4);
      
      msg.set_jointgaittable(0, joint1); 
      msg.set_jointgaittable(1, leftWheel);
      msg.set_jointgaittable(2, rightWheel);
      msg.set_jointgaittable(3, joint2);

      
      pub->Publish(msg);
      std::printf("J1: %f\tJ2: %f\tLW: %f\tRW:%f\n", joint1, joint2, leftWheel, rightWheel);
      gazebo::common::Time::MSleep(100);
    }
    
    gazebo::transport::fini();
    
}

