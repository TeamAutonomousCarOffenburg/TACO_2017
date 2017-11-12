/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer. 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other materials provided with the distribution. 3.  All advertising
materials mentioning features or use of this software must display the following acknowledgement: �This product includes
software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.� 4.  Neither the name of Audi
nor the names of its contributors may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "FisheyeUndistortionROI.h"
#include "ADTF_OpenCV_helper.h"
#include "detection/lanedetection/LaneDetectionRenderer.h"
#include "detection/lanedetection/WideAngleCameraConfig.h"
#include <detection/lanedetection/EdgeDetector.h>

using namespace taco;

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, FisheyeUndistortionROI)

// define the ADTF property names to avoid errors
FisheyeUndistortionROI::FisheyeUndistortionROI(const tChar *__info) : cFilter(__info)
{
	SetPropertyStr("Calibration File", "");
	SetPropertyStr("Calibration File" NSSUBPROP_DESCRIPTION, "Set the calibration file here");
	SetPropertyBool("Calibration File" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr("Calibration File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");

	m_bGpuProcessing = tFalse;
	m_bGpuScaling = 1.0f;

	try {
		if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
			m_bGpuAvailable = tTrue;

			// only set the properties if we have a cuda capable gpu.
			SetPropertyBool("GPU Processing", tFalse);
			SetPropertyStr("GPU Processing" NSSUBPROP_DESCRIPTION, "Activate the gpu image processing.");

			SetPropertyFloat("GPU Scaling", 1.0f);
			SetPropertyStr("GPU Scaling" NSSUBPROP_DESCRIPTION, "Set the scale of the output image.");
			SetPropertyFloat("GPU Scaling" NSSUBPROP_MIN, 0.0f);
		} else {
			m_bGpuAvailable = tFalse;
		}
	} catch (cv::Exception &e) {
		LOG_ERROR(cString::Format("CV exception caught: %s", e.what()));
	}

	m_rectifyMapsSet = false;

	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::ProcessStart", m_oProcessStart);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::PreRemap", m_oPreRemap);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::PostRemap", m_oPostRemap);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::ProcessEnd", m_oProcessEnd);
}

FisheyeUndistortionROI::~FisheyeUndistortionROI()
{
}

tResult FisheyeUndistortionROI::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult FisheyeUndistortionROI::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}
tResult FisheyeUndistortionROI::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst) {
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
				(tVoid **) &pDescManager, __exception_ptr));

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

		// Video Input
		RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

	} else if (eStage == StageNormal) {
		// Get path of calibration file with camera paramters
		cFilename fileCalibration = GetPropertyStr("Calibration File");

		// check if calibration file with camera paramters exits
		if (fileCalibration.IsEmpty()) {
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
		}

		// Get path of calibration file with camera paramters
		ADTF_GET_CONFIG_FILENAME(fileCalibration);
		fileCalibration = fileCalibration.CreateAbsolutePath(".");
		if (!(cFileSystem::Exists(fileCalibration))) {
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
		} else {
			// read the calibration file with camera paramters exits and save to member variable
			cv::FileStorage camera_data(fileCalibration.GetPtr(), cv::FileStorage::READ);
			camera_data["camera_matrix"] >> m_cameraMatrix;
			camera_data["distortion_coefficients"] >> m_distorsionMatrix;
		}

		// check the gpu processing
		if (m_bGpuAvailable) {
			m_bGpuProcessing = GetPropertyBool("GPU Processing", tFalse);
			m_bGpuScaling = GetPropertyFloat("GPU Scaling", 1.0f);

			LOG_INFO(cString::Format("Cuda enabled gpu found. Gpu processing is %s. Scale factor is %f.",
					m_bGpuProcessing ? "enabled" : "disabled", m_bGpuScaling));
		}
	}

	else if (eStage == StageGraphReady) {
		// get the image format of the input video pin
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));

		// set the image format of the input video pin
		if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat()))) {
			LOG_ERROR("Invalid Input Format for this filter");
		}
	}

	RETURN_NOERROR;
}

tResult FisheyeUndistortionROI::Shutdown(tInitStage eStage, ucom::IException **__exception_ptr)
{
	if (eStage == StageGraphReady) {
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult FisheyeUndistortionROI::OnPinEvent(
		IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (pSource == &m_oVideoInputPin) {
			UCOM_TIMING_SPOT(m_oProcessStart);
			ProcessVideo(pMediaSample);
			UCOM_TIMING_SPOT(m_oProcessEnd);
		}
	} else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
		if (pSource == &m_oVideoInputPin) {
			// the input format was changed, so the imageformat has to changed in this filter also
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid **) &pTypeVideo));

			RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
		}
	}
	RETURN_NOERROR;
}

tResult FisheyeUndistortionROI::ProcessVideo(IMediaSample *pSample)
{
	RETURN_IF_POINTER_NULL(pSample);

	if (m_cameraMatrix.empty() || m_distorsionMatrix.empty()) {
		RETURN_NOERROR;
	}

	// pointer for media sample data
	const tVoid *l_pSrcBuffer;
	cv::Mat outputImage;

	if (!m_rectifyMapsSet) {
		// estimate the new, undistorted camera matrix
		cv::Mat newCameraMatrix;
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(m_cameraMatrix, m_distorsionMatrix,
				cv::Size(m_sInputFormat.nWidth, m_sInputFormat.nHeight), cv::Matx33d::eye(), newCameraMatrix, 1.0,
				cv::Size(m_sInputFormat.nWidth, m_sInputFormat.nHeight), 1.0);

		cv::fisheye::initUndistortRectifyMap(m_cameraMatrix, m_distorsionMatrix, cv::Matx33d::eye(), newCameraMatrix,
				cv::Size(m_sInputFormat.nWidth, m_sInputFormat.nHeight), CV_16SC2, m_rectifyMap1, m_rectifyMap2);

		if (m_bGpuAvailable && m_bGpuProcessing) {
			// convert maps to single channel floating point (used for cuda remap)
			cv::Mat mapX_32FC1, mapY_32FC1;
			cv::convertMaps(m_rectifyMap1, m_rectifyMap2, mapX_32FC1, mapY_32FC1, CV_32FC1);

			// upload the maps
			m_GpuRectifyMap1.upload(mapX_32FC1);
			m_GpuRectifyMap2.upload(mapY_32FC1);
		}

		m_rectifyMapsSet = true;
	}

	// receiving data from input sample, and saving to TheInputImage
	if (IS_OK(pSample->Lock(&l_pSrcBuffer))) {
		// convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize) {
			// copy the data to matrix (make a copy, not change the sample content itself!)
			// memcpy(m_inputImage.data, l_pSrcBuffer, m_sInputFormat.nSize);
			// or just set the data pointer of matrix because we create a new matrix later one
			m_inputImage.data = (uchar *) (l_pSrcBuffer);
			if (m_rectifyMapsSet) {
				// Timing spot before the remaping
				UCOM_TIMING_SPOT(m_oPreRemap);
				try {
					if (!m_bGpuAvailable || !m_bGpuProcessing)
						cv::remap(m_inputImage, outputImage, m_rectifyMap1, m_rectifyMap2, cv::INTER_LINEAR,
								cv::BORDER_CONSTANT, cv::Scalar::all(0));
					else {
						// stream used for queueing the instructions on the gpu
						cv::cuda::Stream stream;

						// upload the initial image
						m_GpuImgUpload.upload(m_inputImage, stream);

						// first operation is remaping, note: No in place operations on the gpu!
						cv::cuda::remap(m_GpuImgUpload, m_GpuImgRemaped, m_GpuRectifyMap1, m_GpuRectifyMap2,
								cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0), stream);

						// second is the resize
						cv::cuda::resize(m_GpuImgRemaped, m_GpuImgDownload, cv::Size(), m_bGpuScaling, m_bGpuScaling,
								cv::INTER_LINEAR, stream);

						// download the image to cpu mat
						m_GpuImgDownload.download(outputImage, stream);

						// wait for all events to finish.
						stream.waitForCompletion();
					}

				} catch (cv::Exception &e) {
					LOG_ERROR(cString::Format("CV exception caught: %s", e.what()));
				}

				// Timing spot after remaping
				UCOM_TIMING_SPOT(m_oPostRemap);
			}
		}

		pSample->Unlock(l_pSrcBuffer);
	}

	if (!outputImage.empty()) {
		// helper Matrix to keep format of outputImage but only with selected ROI
		cv::Mat helperMat = outputImage;
		const WideAngleCameraConfig cfg;
		outputImage(cv::Rect(30, 300, cfg.width, cfg.height)).copyTo(helperMat);

		// TODO: move this debug code into a separate lane detection filter?
		/*cv::Mat edges = EdgeDetector::detectEdges(helperMat, cfg.width, cfg.height / 2);
		LaneAssist laneAssist = LaneAssist(cfg);
		PixelToCamera::ConstPtr pixelToCam = boost::make_shared<PixelToCamera>(
				Eigen::Vector3d(0, 0, 1), cfg.focalVertical, cfg.focalHorizontal, cfg.focalPointX, cfg.focalPointY);
		laneAssist.setPixelToCamera(pixelToCam);
		laneAssist.detectLaneMiddle(edges);

		helperMat = LaneDetectionRenderer::draw(cfg.width, cfg.height, helperMat, laneAssist);*/

		outputImage = helperMat;
		UpdateOutputImageFormat(outputImage);

		// create a cImage from CV Matrix (not necessary, just for demonstration)
		cImage newImage;
		newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel,
				m_sOutputFormat.nBytesPerLine, outputImage.data);

		// create the new media sample
		cObjectPtr<IMediaSample> pMediaSample;
		RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
		// updating media sample
		RETURN_IF_FAILED(pMediaSample->Update(
				_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
		// transmitting
		RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

		outputImage.release();
	}

	RETURN_NOERROR;
}

tResult FisheyeUndistortionROI::UpdateInputImageFormat(const tBitmapFormat *pFormat)
{
	if (pFormat != NULL) {
		// update member variable
		m_sInputFormat = (*pFormat);
		LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
				m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize,
				m_sInputFormat.nPixelFormat));
		// create the input matrix
		RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
	}

	RETURN_NOERROR;
}

tResult FisheyeUndistortionROI::UpdateOutputImageFormat(const cv::Mat &outputImage)
{
	// check if pixelformat or size has changed
	if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize) {
		Mat2BmpFormat(outputImage, m_sOutputFormat, IImage::tPixelFormat(m_sInputFormat.nPixelFormat));

		LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
				m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize,
				m_sOutputFormat.nPixelFormat));
		// set output format for output pin
		m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
	}

	RETURN_NOERROR;
}