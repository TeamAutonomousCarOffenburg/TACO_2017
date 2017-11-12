/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception;

import java.util.List;
import java.util.Map;
public interface IPerception {
	/**
	 * Get a specific Accelerometer Perceptor
	 *
	 * @param name Perceptor name
	 * @return Acceleration perceptor
	 */
	IAccelerometerPerceptor getAccelerationPerceptor(String name);

	/**
	 * Get a specific Gyro Perceptor
	 *
	 * @param name Perceptor name
	 * @return Gyro perceptor
	 */
	IGyroPerceptor getGyroRatePerceptor(String name);

	/**
	 * Get a specific Compass Perceptor
	 *
	 * @param name Perceptor name
	 * @return Compass perceptor
	 */
	ICompassPerceptor getCompassPerceptor(String name);

	/**
	 * Get a specific Force Resistance Perceptor
	 *
	 * @param name Perceptor name
	 * @return perceptor
	 */
	IForceResistancePerceptor getForceResistancePerceptor(String name);

	/**
	 * Get a specific Visible Object
	 *
	 * @param name Perceptor name
	 * @return perceptor
	 */
	IVisibleObjectPerceptor getVisibleObject(String name);

	/**
	 * Get the global pose perceptor
	 *
	 * @return perceptor
	 */
	IGlobalPosePerceptor getGlobalPose();

	/**
	 * Get the global time perceptor
	 *
	 * @return perceptor
	 */
	ITimerPerceptor getTime();

	/**
	 * Get the Flags perceptors.
	 *
	 * @return a list of flag perceptors
	 */
	List<IFlagPerceptor> getFlags();

	/**
	 * Get remote command
	 *
	 * @return perceptor
	 */
	ICommandPerceptor getCommandPerceptor();

	List<ILinePerceptor> getVisibleLines();

	List<IReferencePointPerceptor> getReferencePointPerceptor();

	/**
	 * Get a specific Universal Joint Perceptor
	 *
	 * @param name Perceptor name
	 * @return perceptor
	 */
	ICompositeJointPerceptor getCompositeJointPerceptor(String name);

	/**
	 * Get a specific Hinge Joint Perceptor
	 *
	 * @param name Perceptor name
	 * @return perceptor
	 */
	IHingeJointPerceptor getHingeJointPerceptor(String name);

	/**
	 * @return true, if the current perception contains vision information
	 */
	boolean containsVision();

	/**
	 * @return true, if the current perception contains vision information
	 */
	boolean containsMotion();

	void updatePerceptors(Map<String, IPerceptor> perceptor);
}
