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
#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/rendering.hh>
#include <math.h>

using namespace gazebo;

LineLaser::LineLaser() : SensorPlugin()
{
  
}

LineLaser::~LineLaser()
{

}

void LineLaser::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
	std::cout << "Is it getting here..? Loading..." << std::endl;
	this->parentSensor = boost::shared_dynamic_cast<sensors::RaySensor>(_sensor);

	if(!this->parentSensor)
	{
		gzerr << "LineLaser requires a RaySensor\n";
		return;
	}

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = this->parentSensor->ConnectUpdated(
	boost::bind(&LineLaser::OnUpdate, this));  

	this->parentSensor->SetActive(true);

	transport::NodePtr node(new transport::Node());
	std::string parentModelName = this->parentSensor->GetParentName().substr(0, this->parentSensor->GetParentName().find("::"));
	std::cout << "Parent Model Name : " << parentModelName;

	node->Init(parentModelName);
	std::string topicName = "~/" + parentModelName + "/linelaser";// this->parentSensor->GetParentName() + "::" + this->parentSensor->GetName() + "/linelaser";
	std::cout << "\nTopic Name: " << topicName << std::endl;
	this->lineLaserPub = node->Advertise<msgs::LaserScanStamped>(topicName);
  
}

/// \brief so this is what happens when this sensor is running...?
void LineLaser::OnUpdate()
{
	std::vector<double> rangesReading;
	common::Time timeReceived = this->parentSensor->GetLastUpdateTime();
	this->parentSensor->GetRanges(rangesReading);

	msgs::LaserScanStamped msg;
	
	rendering::DynamicLinesPtr dynLinePtr(new rendering::DynamicLines());
	dynLinePtr->Init(gazebo::rendering::RENDERING_POINT_LIST);
	dynLinePtr->Clear();
	
	// get the pose of the sensor
	math::Pose sensorPose = this->parentSensor->GetPose();
	for(int i = 0; i < rangesReading.size(); i++)
	{
		msg.mutable_scan()->add_ranges(rangesReading[i]);
		double angle = this->parentSensor->GetAngleMin().Radian() + (i * this->parentSensor->GetAngleResolution());
		double x = rangesReading[i] * cos(angle);
		double y = rangesReading[i] * sin(angle);
		gazebo::math::Pose lidarPtPose(x, y, 0, 0, 0, 0);
		gazebo::math::Pose globalLidarPtPose = lidarPtPose + sensorPose;
		dynLinePtr->AddPoint(globalLidarPtPose.pos, common::Color::Red);
	}
// 	dynLinePtr->Update();
// 	std::cout << "UPdated" << std::endl;
	//   std::cout << "SIZE: " << msg.scan().ranges_size() << std::endl;
	msg.mutable_time()->set_sec(timeReceived.sec);
	msg.mutable_time()->set_nsec(timeReceived.nsec);
	msg.mutable_scan()->set_angle_max(this->parentSensor->GetAngleMax().Radian());
	msg.mutable_scan()->set_angle_min(this->parentSensor->GetAngleMin().Radian());
	msg.mutable_scan()->set_angle_step(this->parentSensor->GetAngleResolution());
	msg.mutable_scan()->set_count(this->parentSensor->GetRangeCount());
	msg.mutable_scan()->set_range_max(this->parentSensor->GetRangeMax());
	msg.mutable_scan()->set_range_min(this->parentSensor->GetRangeMin());
	msg.mutable_scan()->set_vertical_angle_max(this->parentSensor->GetVerticalAngleMax().Radian());
	msg.mutable_scan()->set_vertical_angle_min(this->parentSensor->GetVerticalAngleMin().Radian());
	msg.mutable_scan()->set_vertical_angle_step(this->parentSensor->GetVerticalAngleResolution());
	msg.mutable_scan()->set_vertical_count(this->parentSensor->GetVerticalRangeCount());
// 	this->lineLaserPub->Publish(msg);
	//   std::cout << "MSG Sent" << std::endl;
}

GZ_REGISTER_SENSOR_PLUGIN(LineLaser)