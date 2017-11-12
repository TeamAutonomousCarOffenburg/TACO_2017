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
public class OffsetJointMapper implements IJointToMotorMapper
{
	private double offset;

	private double scaling;

	public OffsetJointMapper(double offset)
	{
		this(offset, 1);
	}

	public OffsetJointMapper(double offset, double scaling)
	{
		this.offset = offset;
		this.scaling = scaling;
	}

	@Override
	public double[] jointToMotorAngle(double[] jointAngles)
	{
		double[] result = new double[jointAngles.length];
		for (int i = 0; i < jointAngles.length; i++) {
			result[i] = jointAngles[i] * scaling + offset;
		}
		// System.out.println("joint angle: " + jointAngles[0] + " maps to: "
		// + result[0]);
		return result;
	}

	@Override
	public double[] motorToJointAngle(double[] motorAngles)
	{
		double[] result = new double[motorAngles.length];
		for (int i = 0; i < motorAngles.length; i++) {
			result[i] = (motorAngles[i] - offset) / scaling;
		}
		return result;
	}
}
