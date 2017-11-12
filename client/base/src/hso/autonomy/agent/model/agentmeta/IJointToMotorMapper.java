/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta;

/**
 * Maps joint angles to motor angles and vice versa. Implementations will be
 * specific for a certain joint.
 *
 * @author kdorer
 */
public interface IJointToMotorMapper {
	/**
	 * Maps joint to motor angles. The length and order of the parameter and
	 * result array depends on the specific joint that is mapped.
	 *
	 * @param jointAngles the joint angles in degrees.
	 * @return the corresponding motor angles in degrees.
	 */
	double[] jointToMotorAngle(double[] jointAngles);

	/**
	 * Maps motor to joint angles. The length and order of the parameter and
	 * result array depends on the specific joint that is mapped.
	 *
	 * @param motorAngles the motor angles in degrees.
	 * @return the corresponding joint angles in degrees.
	 */
	double[] motorToJointAngle(double[] motorAngles);
}