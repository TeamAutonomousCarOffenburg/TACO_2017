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
public class FixedValueJointMapper implements IJointToMotorMapper
{
	private double[] motorValues;

	private double[] jointValues;

	public FixedValueJointMapper(double[] motorValues, double[] jointValues)
	{
		this.motorValues = motorValues;
		this.jointValues = jointValues;
	}

	@Override
	public double[] jointToMotorAngle(double[] jointAngles)
	{
		return motorValues;
	}

	@Override
	public double[] motorToJointAngle(double[] motorAngles)
	{
		return jointValues;
	}
}
