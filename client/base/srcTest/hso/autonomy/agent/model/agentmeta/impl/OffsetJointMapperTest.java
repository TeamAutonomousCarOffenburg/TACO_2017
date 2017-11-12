/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta.impl;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import hso.autonomy.agent.model.agentmeta.impl.OffsetJointMapper;

/**
 *
 * @author kdorer
 */
public class OffsetJointMapperTest
{
	private OffsetJointMapper testee;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception
	{
		testee = new OffsetJointMapper(180, -1);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmeta.impl.OffsetJointMapper#jointToMotorAngle(double[])}
	 * .
	 */
	@Test
	public void testJointToMotorAngle()
	{
		double[] jointAngle = {10};
		double[] motorAngle = testee.jointToMotorAngle(jointAngle);
		assertEquals(170, motorAngle[0], 0.001);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmeta.impl.OffsetJointMapper#motorToJointAngle(double[])}
	 * .
	 */
	@Test
	public void testMotorToJointAngle()
	{
		double[] motorAngles = {170};
		double[] jointAngle = testee.motorToJointAngle(motorAngles);
		assertEquals(10, jointAngle[0], 0.001);
	}
}
