#pragma once

#include "../perception/IPerception.h"
#include "PackageDecoder.h"
#include "PackageEncoder.h"
#include "detection/FloorNormalDetection.h"
#include "detection/ILaneDetection.h"
#include "detection/IObjectDetection.h"
#include "detection/lanedetection/LaneAssist.h"
#include <action/IAction.h>
#include <chrono>
#include <string>
#include <thread>

static const int PREFIX_SIZE = 4;
static const std::chrono::duration<double, std::milli> RESPONSIVENESS_THRESHOLD = std::chrono::milliseconds(100);

class Communication
{
  public:
	void start();
	void stop();
	Communication(taco::ICarMetaModel::Ptr carMetaModel, taco::IAction::Ptr action, taco::IPerception::Ptr perception,
			taco::ILaneDetection::Ptr laneDetection, taco::IObjectDetection::Ptr objectDetection,
			taco::FloorNormalDetection &floorNormal, uint16_t port = 63236);
	~Communication();
	bool update();
	void receive();

  private:
	taco::ICarMetaModel::Ptr _carMetaModel;
	taco::IAction::Ptr _action;
	PackageDecoder *decoder;
	PackageEncoder *encoder;
	bool serverRunning;
	bool clientResponsive;
	uint16_t port;
	int socketFD;
	int clientFD;
	std::thread receiveThread;
	std::chrono::system_clock::time_point currentTime;
	std::chrono::system_clock::time_point lastClientConnectionTime;
	void openSocket();
	void checkResponsiveness(std::chrono::system_clock::time_point lastReceiveTime);
	void onClientDisconnected();

	void log(std::string message);
	void error(std::string message);
};
