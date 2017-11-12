#pragma once

#define FILTER_CLASS_ID "adtf.taco.CameraImageRelay"
#define FILTER_VERSION_ID "adtf.taco.CameraImageRelay"
#define FILTER_CLASS_LABEL "TACO Camera Image Relay"
#define FILTER_VERSION_MAJOR 1
#define FILTER_VERSION_MINOR 0
#define FILTER_VERSION_BUILD 0
#define FILTER_VERSION_LABEL "Beta Version"

#define FILTER_PIN_NAME_VIDEOIN "Video_input"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>

using namespace cv;
using namespace adtf;
/*!
  this filter creates sends the images to an python server
 */
class CameraImageRelay : public cFilter
{
	ADTF_DECLARE_FILTER_VERSION(FILTER_CLASS_ID, FILTER_CLASS_LABEL, adtf::OBJCAT_Tool, FILTER_VERSION_ID,
			FILTER_VERSION_MAJOR, FILTER_VERSION_MINOR, FILTER_VERSION_BUILD, FILTER_VERSION_LABEL);

  public:
	CameraImageRelay(const tChar *__info);
	~CameraImageRelay();
	tResult Init(tInitStage eStage, __exception);

  protected:
	cVideoPin m_oPinInputVideo; /**< output Pin for video */
	tResult Start(__exception);
	tResult Stop(__exception);
	tResult Shutdown(tInitStage eStage, __exception = NULL);
	tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

  private:
	const tBitmapFormat *_VideoInputFormat;
	void initializeVideoParams();
	tResult PropertyChanged(const tChar *propertyName);
	int startServer();
	bool firstFrame;
	int fps;
	double msPerFrame;
	std::chrono::system_clock::time_point now;
	std::chrono::system_clock::time_point lastUpdate;
	int serverPort;
	int socketFD;
	int clientFD = -1;
	bool serverStarted = false;

	void log(std::string message);
	void error(std::string message);
};
