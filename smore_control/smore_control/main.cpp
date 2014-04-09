#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <math/gzmath.hh>

#include "command_message.pb.h"

#include <iostream>

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
    std::string pubName = "~/" + robotName + "_model";
    gazebo::transport::PublisherPtr pub = node->Advertise<command_message::msgs::CommandMessage>(pubName);
    pub->WaitForConnection();
        
    std::cout << "getting in to the loop..." << std::endl;
    command_message::msgs::CommandMessage msg;
    while(true)
    {
      msg.set_messagetype(0);
      
      gazebo::common::Time::MSleep(100);
    }
    
    gazebo::transport::fini();
    
}