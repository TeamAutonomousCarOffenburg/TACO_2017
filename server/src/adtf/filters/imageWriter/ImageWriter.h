#pragma once

#define FILTER_CLASS_ID "adtf.taco.ImageWriter"
#define FILTER_VERSION_ID "adtf.taco.ImageWriter"
#define FILTER_CLASS_LABEL "TACO Image Writer"
#define FILTER_VERSION_MAJOR 1
#define FILTER_VERSION_MINOR 0
#define FILTER_VERSION_BUILD 0
#define FILTER_VERSION_LABEL "Beta Version"

#define FILTER_PIN_NAME_VIDEOIN "Video_input"
#define UPDATE_RATE_PROPERTY "Update rate in ms"

// ADTF headers
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <chrono>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace adtf;
/*!
  this filter creates a video file from the received images
 */
class ImageWriter : public cFilter
{
	ADTF_DECLARE_FILTER_VERSION(FILTER_CLASS_ID, FILTER_CLASS_LABEL, adtf::OBJCAT_Tool, FILTER_VERSION_ID,
			FILTER_VERSION_MAJOR, FILTER_VERSION_MINOR, FILTER_VERSION_BUILD, FILTER_VERSION_LABEL);

  public:
	ImageWriter(const tChar *__info);
	~ImageWriter();
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
	ImageWriter *writer;
	bool firstFrame;
	cString path;
	int fps;
	double msPerFrame;
	std::chrono::system_clock::time_point now;
	std::chrono::system_clock::time_point lastUpdate;
};
