#include "ImageWriter.h"
#include <chrono>
#include <ctime>
#include <iostream>

using namespace std;

#define FILTER_PROPERTY_NAME_PATH "Folder for the images"
#define FILTER_PROPERTY_NAME_FPS "Frames per second"

ADTF_FILTER_PLUGIN(FILTER_CLASS_LABEL, FILTER_CLASS_ID, ImageWriter)

void ImageWriter::initializeVideoParams()
{
	cObjectPtr<IMediaType> pType;
	m_oPinInputVideo.GetMediaType(&pType);
	cObjectPtr<IMediaTypeVideo> pTypeVideo;
	pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo);
	_VideoInputFormat = pTypeVideo->GetFormat();
}

tResult ImageWriter::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	now = std::chrono::system_clock::now();

	if (firstFrame) {
		initializeVideoParams();
		lastUpdate = std::chrono::system_clock::now();
		firstFrame = false;
	}

	std::chrono::duration<double, std::milli> timeSinceLastUpdate = now - lastUpdate;

	if (timeSinceLastUpdate.count() >= msPerFrame) {
		if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
			const tVoid *buffer;
			if (IS_OK(pMediaSample->Lock(&buffer))) {
				IplImage *img = cvCreateImageHeader(
						cvSize(m_oPinInputVideo.GetFormat()->nWidth, m_oPinInputVideo.GetFormat()->nHeight),
						IPL_DEPTH_8U, 3);

				img->imageData = (char *) buffer;

				Mat image(cvarrToMat(img));

				vector<int> params;
				params.push_back(CV_IMWRITE_JPEG_QUALITY);
				params.push_back(100);

				unsigned long milliseconds_since_epoch = static_cast<unsigned long>(
						std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

				stringstream ss;
				ss << path << "/" << milliseconds_since_epoch << ".jpg";
				string filename = ss.str();

				// custom created images has to be converted before imwrite() because of openCV's BGR order
				Mat destMat;
				cvtColor(image, destMat, CV_RGB2BGR);

				imwrite(filename, destMat, params);

				pMediaSample->Unlock(buffer);
			}
		}

		lastUpdate = now;
	}

	RETURN_NOERROR;
}

ImageWriter::ImageWriter(const tChar *__info) : cFilter(__info)
{
	firstFrame = true;
	SetPropertyStr(FILTER_PROPERTY_NAME_PATH, "images/");
	SetPropertyBool(FILTER_PROPERTY_NAME_PATH NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(FILTER_PROPERTY_NAME_FPS, 15);
	SetPropertyBool(FILTER_PROPERTY_NAME_FPS NSSUBPROP_ISCHANGEABLE, tTrue);
}

ImageWriter::~ImageWriter()
{
}

tResult ImageWriter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst) {
		PropertyChanged(FILTER_PROPERTY_NAME_PATH);
		PropertyChanged(FILTER_PROPERTY_NAME_FPS);
		// create the output VideoPin
		RETURN_IF_FAILED(m_oPinInputVideo.Create(
				FILTER_PIN_NAME_VIDEOIN, adtf::IPin::PD_Input, static_cast<IPinEventSink *>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));
	} else if (eStage == StageNormal) {
	} else if (eStage == StageGraphReady) {
	}
	RETURN_NOERROR;
}

tResult ImageWriter::PropertyChanged(const tChar *propertyName)
{
	if (strcmp(propertyName, FILTER_PROPERTY_NAME_PATH) == 0) {
		path = GetPropertyStr(FILTER_PROPERTY_NAME_PATH);
	} else if (strcmp(propertyName, FILTER_PROPERTY_NAME_FPS) == 0) {
		fps = GetPropertyInt(FILTER_PROPERTY_NAME_FPS);
		msPerFrame = (1.0 / (double) fps) * 1000.0;
	}
	RETURN_NOERROR;
}

tResult ImageWriter::Shutdown(tInitStage eStage, __exception)
{
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult ImageWriter::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult ImageWriter::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}
