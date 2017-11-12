#include "CameraImageRelay.h"
#include <iostream>

using namespace std;

#define FILTER_PROPERTY_NAME_PORT "Destination Server PORT"
#define FILTER_PROPERTY_NAME_FPS "Frames per second"

ADTF_FILTER_PLUGIN(FILTER_CLASS_LABEL, FILTER_CLASS_ID, CameraImageRelay)

void CameraImageRelay::initializeVideoParams()
{
	cObjectPtr<IMediaType> pType;
	m_oPinInputVideo.GetMediaType(&pType);
	cObjectPtr<IMediaTypeVideo> pTypeVideo;
	pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo);
	_VideoInputFormat = pTypeVideo->GetFormat();
}

tResult CameraImageRelay::OnPinEvent(
		IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	if (firstFrame) {
		initializeVideoParams();
		lastUpdate = std::chrono::system_clock::now();
		firstFrame = false;
	}

	if (serverStarted && clientFD == -1) {
		clientFD = accept(socketFD, nullptr, nullptr);
		if (clientFD == -1 && (errno != EAGAIN || errno != EWOULDBLOCK)) {
			error("accept()");
			RETURN_NOERROR;
		}

		if (clientFD != -1) {
			log("Client connected");
		}
	}

	if (clientFD != -1) {
		now = std::chrono::system_clock::now();

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

					Mat destMat;
					cvtColor(image, destMat, CV_RGB2BGR);

					image = destMat.reshape(0, 1);
					size_t image_size = image.total() * image.elemSize();
					ssize_t num_bytes = send(clientFD, image.data, image_size, 0);
					if (num_bytes == -1) {
						log("Client disconnected");
						shutdown(clientFD, SHUT_RDWR);
						close(clientFD);
						clientFD = -1;
					}

					pMediaSample->Unlock(buffer);
				}
			}

			lastUpdate = now;
		}
	}

	RETURN_NOERROR;
}

CameraImageRelay::CameraImageRelay(const tChar *__info) : cFilter(__info)
{
	firstFrame = true;
	SetPropertyInt(FILTER_PROPERTY_NAME_PORT, 1337);
	SetPropertyBool(FILTER_PROPERTY_NAME_PORT NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt(FILTER_PROPERTY_NAME_FPS, 15);
	SetPropertyBool(FILTER_PROPERTY_NAME_FPS NSSUBPROP_ISCHANGEABLE, tTrue);
}

CameraImageRelay::~CameraImageRelay() = default;

tResult CameraImageRelay::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst) {
		PropertyChanged(FILTER_PROPERTY_NAME_PORT);
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

tResult CameraImageRelay::PropertyChanged(const tChar *propertyName)
{
	if (strcmp(propertyName, FILTER_PROPERTY_NAME_PORT) == 0) {
		serverPort = GetPropertyInt(FILTER_PROPERTY_NAME_PORT);
	} else if (strcmp(propertyName, FILTER_PROPERTY_NAME_FPS) == 0) {
		fps = GetPropertyInt(FILTER_PROPERTY_NAME_FPS);
		msPerFrame = (1.0 / (double) fps) * 1000.0;
	}
	RETURN_NOERROR;
}

tResult CameraImageRelay::Shutdown(tInitStage eStage, __exception)
{
	close(socketFD);
	close(clientFD);
	RETURN_NOERROR;
}

tResult CameraImageRelay::Start(__exception)
{
	return CameraImageRelay::startServer();
}

tResult CameraImageRelay::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

int CameraImageRelay::startServer()
{
	socketFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (socketFD == -1) {
		error("socket()");
		return 1;
	}
	fcntl(socketFD, F_SETFL, O_NONBLOCK);

	int enable = 1;
	if (setsockopt(socketFD, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
		error("setsockopt(SO_REUSEADDR)");
	}

	struct sockaddr_in address;
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(serverPort);

	if (bind(socketFD, (struct sockaddr *) &address, sizeof(address)) == -1) {
		error("bind()");
		return 2;
	}

	if (listen(socketFD, 5) == -1) {
		error("listen()");
		return 3;
	}

	clientFD = -1;
	serverStarted = true;

	return 0;
}

void CameraImageRelay::log(std::string message)
{
	cout << "[CameraImageRelay] " << message << endl;
}

void CameraImageRelay::error(std::string message)
{
	cerr << "[CameraImageRelay] " << message << " failed: " << strerror(errno) << endl;
}
