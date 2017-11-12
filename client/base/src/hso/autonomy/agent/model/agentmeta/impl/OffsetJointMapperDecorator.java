/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta.impl;

import hso.autonomy.agent.model.agentmeta.IJointToMotorMapper;

/**
 *
 * @author kdorer
 */
public class OffsetJointMapperDecorator implements IJointToMotorMapper
{
	private IJointToMotorMapper decoratee;

	private double[] offset;

	private double[] scaling;

	public OffsetJointMapperDecorator(IJointToMotorMapper decoratee, double[] offset, double[] scaling)
	{
		this.decoratee = decoratee;
		this.offset = offset;
		this.scaling = scaling;
	}

	@Override
	public double[] jointToMotorAngle(double[] jointAngles)
	{
		double[] result = decoratee.jointToMotorAngle(jointAngles);
		for (int i = 0; i < jointAngles.length; i++) {
			result[i] = result[i] * scaling[i] + offset[i];
		}
		// System.out.println("joint angle: " + jointAngles[0] + " maps to: "
		// + result[0]);
		return result;
	}

	@Override
	public double[] motorToJointAngle(double[] motorAngles)
	{
		double[] result = decoratee.motorToJointAngle(motorAngles);
		for (int i = 0; i < motorAngles.length; i++) {
			result[i] = (result[i] - offset[i]) / scaling[i];
		}
		return result;
	}
}
