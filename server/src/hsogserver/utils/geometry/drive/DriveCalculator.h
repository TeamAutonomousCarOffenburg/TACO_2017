#pragma once

#include "../Angle.h"
#include "../Pose2D.h"

#include "boost/smart_ptr.hpp"

namespace taco
{
/** The DriveCalculator class provides calculations in the context of an Ackermann Drive.
 *
 * \author Stefan Glaser
 */
class DriveCalculator
{
  public:
	typedef boost::shared_ptr<DriveCalculator> Ptr;
	typedef boost::shared_ptr<const DriveCalculator> ConstPtr;

	/** Default constructor, creating a drive with 1m wheel spacing and 2m axle spacing. */
	DriveCalculator();

	/** Construct a DriveCalculator based on the given wheel spacing and axle spacing.
	 * \param wheelSpacing - the distance between the left and right wheel
	 * \param axleSpacing - the distance between the front and rear axle
	 */
	DriveCalculator(const double &wheelSpacing, const double &axleSpacing);

	~DriveCalculator();

	/** \brief Calculate the driving pose based on the rear wheel spacing and their rolling-distances.
	 *
	 * Calculate the pose of the (center of the) rear axle after rolling leftWheelDistance and rightWheelDistance on the
	 * wheels.
	 *
	 * \param leftWheelDistance - the distance the left wheel progressed
	 * \param rightWheelDistance - the distance the right wheel progressed
	 * \returns the resulting pose in 2D space
	 */
	virtual const Pose2D calculateCurvePose(const double &leftWheelDistance, const double &rightWheelDistance) const;

	/** \brief Calculate the steering angle to drive a certain curve.
	 *
	 * Calculate the necessary steering angle to drive a curve with the given radius.
	 * Note: A positive/negative curve radius results in an positive/negative steering angle.
	 *
	 * \param curveRadius - the radius of the intended curve in meter
	 * \returns the steering angle resulting in the given curve radius
	 */
	virtual const Angle calculateSteeringAngle(const double &curveRadius) const;

	/** \brief Calculate the curve radius from a certain steering angle.
	 *
	 * Calculate the resulting curve radius by driving with the given steering angle.
	 * Note: A positive/negative steering angle results in an positive/negative curve radius.
	 *
	 * \param steeringAngle - the steering angle
	 * \returns the curve radius resulting from the given steering angle
	 */
	virtual const double calculateCurveRadius(const Angle &steeringAngle) const;

  protected:
	/** The distance between the left and right wheel along the axle. */
	double _wheelSpacing;

	/** The distance from the front to the rear axle. */
	double _axleSpacing;
};
}
