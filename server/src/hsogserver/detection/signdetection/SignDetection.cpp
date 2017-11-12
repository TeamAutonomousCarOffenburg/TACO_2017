#include "SignDetection.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <utils/geometry/Pose3D.h>

#if ARUCO_FOUND
#include <aruco/marker.h>
#endif

using namespace cv;
using namespace taco;

namespace taco
{
const RoadSign SignDetection::buildRoadSign(taco::IRoadSignPerceptor::ConstPtr perceptor)
{
#if ARUCO_FOUND
	aruco::Marker marker;
	marker.id = perceptor->getId();
	marker.ssize = perceptor->getImageSize();
	marker.Tvec = perceptor->getTVec();
	marker.Rvec = perceptor->getRVec();

	// create a matrix that contains '0' for valid values and '255' for NaN
	cv::Mat tVecNanCheck = cv::Mat(marker.Tvec != marker.Tvec);
	cv::Mat rVecNanCheck = cv::Mat(marker.Rvec != marker.Rvec);
	// if the check matrix does contains something else than zeros, the data is invalid
	if (countNonZero(tVecNanCheck) > 0 || countNonZero(rVecNanCheck) > 0 || countNonZero(marker.Tvec) == 0) {
		return RoadSign();
	}

	cv::Mat rotationMat;
	Rodrigues(marker.Rvec, rotationMat);

	// convert to eigen mats
	Eigen::Matrix3d eigenRot;
	cv2eigen(rotationMat, eigenRot);

	Eigen::Vector3d eigenTrans;
	cv2eigen(marker.Tvec, eigenTrans);

	// TODO rotate correctly, so we can use the resulting quat for the pose
	Eigen::Quaterniond quat(eigenRot);
	Eigen::AngleAxisd axis1(M_PI / 2.0, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd axis2((M_PI / 2.0), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd axis3((M_PI / 2.0), Eigen::Vector3d::UnitX());

	Eigen::Quaterniond rotatedQuat =
			Eigen::Quaterniond(axis1) * Eigen::Quaterniond(axis2) * Eigen::Quaterniond(axis3) * quat;

	// this gives us the angle we want
	double bank = atan2(2 * rotatedQuat.x() * rotatedQuat.w() - 2 * rotatedQuat.y() * rotatedQuat.z(),
			1 - 2 * rotatedQuat.x() * rotatedQuat.x() - 2 * rotatedQuat.z() * rotatedQuat.z());

	taco::Pose3D signPose(eigenTrans, quat);
	taco::Pose3D signPose2(signPose.z(), -signPose.x(), -signPose.y());

	taco::Pose2D signPose2d(signPose2.x(), signPose2.y(), Angle(bank));
	return RoadSign(static_cast<SignType>(marker.id), signPose2d);
#else
	return RoadSign();
#endif
}
}
