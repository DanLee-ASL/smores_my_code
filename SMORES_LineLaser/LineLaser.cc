/*
    Copyright (c) 2014, <copyright holder> <email>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "LineLaser.hh"
#include <gazebo/sensors/Sensor.hh>

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(LineLaser)


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

LineLaser::LineLaser() : SensorPlugin()
{

}

LineLaser::~LineLaser()
{

}

void LineLaser::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  std::cout << _sensor->GetName() << std::endl;
  std::cout << _sensor->GetParentName() << std::endl;
  std::cout << _sensor->GetWorldName() << std::endl;
  
  this->parentSensor = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
//   sdf::ElementPtr sdf(new sdf::Element);
//   sdf::initFile("sensor.sdf", sdf);
//   sdf::readString(raySensorString, sdf);
  
  std::cout << "REally done?" << std::endl;
  std::cout << this->parentSensor->GetName() << std::endl;
  std::cout << this->parentSensor->GetParentName() << std::endl;
  std::cout << _sdf->ToString("ASDF") << std::endl;
  
  std::string sensorName = mgr->CreateSensor(_sdf, _sensor->GetWorldName(), "SMORES6Uriah::FrontWheel");
  std::cout << sensorName << std::endl;
  
  mgr->Update(); // this update the sensor manager so that it can process new sensors
  
  // get a pointer to the ray sensor
  sensors::RaySensorPtr sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(mgr->GetSensor(sensorName));
//   sensor->Update(true);
  std::cout << "HMM???" << std::endl;
}

/// \brief so this is what happens when this sensor is running...?
void LineLaser::OnUpdate()
{
  
}

