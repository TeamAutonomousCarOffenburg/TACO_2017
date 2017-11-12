package taco.agent.model.agentmodel;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.util.geometry.Angle;

/**
 * Representation of an inertial measurement unit.
 */
public interface IImuSensor extends IAudiCupSensor {
	/**
	 * @return the current orientation of the sensor with respect to its initial orientation
	 */
	Rotation getOrientation();

	/**
	 * @return the z-Angle of the current orientation compared to the initial z-Angle
	 */
	Angle getHorizontalAngle();

	/**
	 * @return amount and direction of current acceleration (unit ?)
	 */
	Vector3D getAcceleration();
}