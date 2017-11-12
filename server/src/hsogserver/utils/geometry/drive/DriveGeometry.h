#pragma once

#include "../Angle.h"
#include "../Circle2D.h"
#include "../Cosine.h"
#include "../Pose2D.h"
#include "../Pose3D.h"
#include "Driveable.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace taco
{
/** Util class providing different geometry calculations to different drive systems.
 *
 * \author Stefan Glaser, Peter Walden
 */
class DriveGeometry
{
  public:
	/** \brief Calculate the driving pose based on the rear wheel spacing and their rolling-distances.
	 *
	 * Calculate the pose of the (center of the) rear axle after rolling leftWheelDistance and rightWheelDistance on the
	 * wheels. In order to to so, the wheel spacing (distance between the wheels along the axle).
	 *
	 * \param leftWheelDistance - the distance the left wheel progressed
	 * \param rightWheelDistance - the distance the right wheel progressed
	 * \param wheelSpacing - the distance between the two wheels along the axle
	 */
	static const Pose2D calculateCurvePose(
			const double &leftWheelDistance, const double &rightWheelDistance, const double &wheelSpacing);

	/** \brief Calculate the steering angle to drive a certain curve.
	 *
	 * Calculate the necessary steering angle to drive a curve with the given radius.
	 * Note: A positive/negative curve radius results in an positive/negative steering angle.
	 *
	 * \param curveRadius - the radius of the intended curve in meter
	 * \param axleSpacing - the distance between the front and rear axle
	 * \returns the steering angle resulting in the given curve radius
	 */
	static const Angle calculateSteeringAngle(const double &curveRadius, const double &axleSpacing);

	/** \brief Calculate the curve radius from a certain steering angle.
	 *
	 * Calculate the resulting curve radius by driving with the given steering angle.
	 * Note: A positive/negative steering angle results in an positive/negative curve radius.
	 *
	 * \param steeringAngle - the steering angle
	 * \param axleSpacing - the distance between the front and rear axle
	 * \returns the curve radius resulting from the given steering angle
	 */
	static const double calculateCurveRadius(const Angle &steeringAngle, const double &axleSpacing);

	/** get "line before circle" geometry in world coordinates
	 *  \return Driveable line before circle
	 */
	static const Driveable getLineBeforeCircleGeometry(const Pose2D &carPose, const Pose2D &nextWayPoint);

	/** get "circle before line" geometry in world coordinates
	 *  \return Driveable: circle before line
	 */
	static const Driveable getCircleBeforeLineGeometry(const Pose2D &carPose, const Pose2D &nextWayPoint);

	/** get "cosine before line" geometry in world coordinates
	 *  \return Driveable: cosine with max angle 45°
	 */
	static const Driveable getSCurveBeforeLineGeometry(const Pose2D &carPose, const Pose2D &nextWayPoint);

	/** get circle geometry in world coordinates
	 *  \return Driveable: circle
	 */
	static const Driveable getCircle(const Pose2D &carPose, const Pose2D &nextWayPoint);

	/** get line geometry in world coordinates
	 *  \return Driveable: line
	 */
	static const Driveable getLine(const Pose2D &carPose, const Pose2D &nextWayPoint);

	/** Returns a curve from two poses in intervall 0, [1/2*pi, 3/2*pi]
	 *
	 * \param directionLocal pose will be set on x = 0, y = 1 * spreading
	 * \param other other pose on cosine in intervall 0.5*pi , 3/2*pi or -0.5*pi , -3/2*pi: CAUTION: angle of "other" is
	 * inaccurate if ratio != 1
	 */
	static const Cosine getConnectingCosine(const Pose2D &directionLocal, const Pose2D &other);
};
}
