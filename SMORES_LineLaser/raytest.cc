/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_LINELASER_PLUGIN_HH
#define _GAZEBO_LINELASER_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{ 
  
  static std::string raySensorString =
"<sdf version='1.3'>"
"  <sensor name='laser' type='ray'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <ray>"
"      <scan>"
"        <horizontal>"
"          <samples>640</samples>"
"          <resolution>1.000000</resolution>"
"          <min_angle>-2.2689</min_angle>"
"          <max_angle>2.2689</max_angle>"
"        </horizontal>"
"      </scan>"
"      <range>"
"        <min>0.08</min>"
"        <max>10.0</max>"
"        <resolution>0.01</resolution>"
"      </range>"
"    </ray>"
"  </sensor>"
"</sdf>";


  class RayTest : public SensorPlugin
  {  
    public: RayTest() : SensorPlugin()
    {
    }
    
    public: void Load(sensors::SensorPtr sensor, sdf::ElementPtr _sdf)
    {

      std::string sdfValue = "";
      std::cout << _sdf->GetDescription() << std::endl;
      
      sdf::ElementPtr sdf(new sdf::Element);
      sdf::initFile("sensor.sdf", sdf);
      sdf::readString(raySensorString, sdf);
      this->sdf = sdf;
      
      sdf->PrintDescription(sdfValue);
      std::cout << sdfValue << std::endl;
      
      this->sensor = sensor;
//       this->sdf = _sdf;
      // Get then name of the parent model
      std::string sensorName = sensor->GetName();
      std::cout << "MODEL NAME: " << sensorName << std::endl;
      std::string modelName = sensor->GetParentName();
      std::cout << "MODEL/PARENT NAME: " << modelName << std::endl;
      // Get the world name.
      std::string worldName = sensor->GetWorldName();
      std::cout << "WORLD NAME: " << worldName << std::endl;
      world = physics::get_world(worldName);

      // Get a pointer to the model
      std::cout << "Number of Models: " << (int)(world->GetModelCount()) << std::endl;
      std::cout << "GOT WORLD, now getting the model" << std::endl;
      for(int i = 0; i < world->GetModelCount(); i++) {
	std::cout << world->GetModels()[i]->GetName() << std::endl;
      }
//       this->model = world->GetModel(modelName);
//       std::cout << "GOT MODEL" << std::endl;

      // Error message if the model couldn't be found
//       if (!this->model)
//         gzerr << "Unable to get parent model\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RayTest::OnUpdate, this));
      gzdbg << "plugin model name: " << modelName << "\n";
      std::cout << "BOUND" << std::endl;


//       this->node = transport::NodePtr(new transport::Node());
//       this->node->Init(worldName);
//       this->statsSub =
//         this->node->Subscribe("~/world_stats", &RayTest::OnStats, this);
      std::cout << "DONE LOADING" << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // do something on update
      // gzdbg << "plugin update\n";
      if(!raySensor)
      {
	sensors::SensorManager *mgr = sensors::SensorManager::Instance();
	std::cout << "PARENT NAME?? " << this->sensor->GetParentName() << std::endl;
	for(int i = 0; i < mgr->GetSensors().size(); i++)
	{
	  std::cout << "SENSOR: " << mgr->GetSensors()[i]->GetName() << "\tParent: " << mgr->GetSensors()[i]->GetParentName() << "\tType:" << mgr->GetSensors()[i]->GetType() << std::endl;
	}
	std::cout << "SDF NAME: " << this->sdf->GetName() << std::endl;
	std::string createdSensorName = mgr->CreateSensor(this->sdf, this->sensor->GetWorldName(), this->sensor->GetName());
	
	std::cout << "Sensor Created: " << createdSensorName << std::endl;
// 	sensors::RaySensorPtr sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(mgr->GetSensor(createdSensorName));
	raySensor = boost::dynamic_pointer_cast<sensors::RaySensor>(mgr->GetSensor(createdSensorName));
      }
    }

    public: void OnStats(
                const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
    {
      this->simTime  = msgs::Convert(_msg->sim_time());

      math::Pose pose;
      pose.pos.x = 0.5*sin(0.01*this->simTime.Double());
      math::Pose orig_pose = this->model->GetWorldPose();

      if (this->simTime.Double() > 20.0)
        this->model->SetWorldPose(pose);

      gzdbg << "plugin simTime [" << this->simTime.Double()
            << "] update pose [" << pose.pos.x << "] orig pose ["
            << orig_pose << "]\n";
    }

    private: physics::WorldPtr world;
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    private: common::Time simTime;
    private: sensors::RaySensorPtr raySensor;
    private: sensors::SensorPtr sensor;
    private: sdf::ElementPtr sdf;
  };  
  
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(RayTest)
}

#endif