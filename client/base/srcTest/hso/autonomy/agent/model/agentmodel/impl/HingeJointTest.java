/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import static org.junit.Assert.assertEquals;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Before;
import org.junit.Test;

/**
 *
 * @author kdorer
 */
public class HingeJointTest
{
	private HingeJoint testee;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception
	{
		testee = new HingeJoint("Test", "percept", "effect", Vector3D.PLUS_I, -10, 10, 7, 2, 20, false);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmodel.impl.HingeJoint#performAxisSpeed(float)}.
	 */
	@Test
	public void testPerformAxisSpeedNoLimit()
	{
		testee.performAxisSpeed(1);
		assertEquals(1, testee.getAngle(), 0.001);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmodel.impl.HingeJoint#performAxisSpeed(float)}.
	 */
	@Test
	public void testPerformAxisSpeedMaxAcceleration()
	{
		testee.performAxisSpeed(4);
		assertEquals(2, testee.getAngle(), 0.001);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmodel.impl.HingeJoint#performAxisSpeed(float)}.
	 */
	@Test
	public void testPerformAxisSpeedMaxAccelerationNegative()
	{
		testee.performAxisSpeed(-4);
		assertEquals(-2, testee.getAngle(), 0.001);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmodel.impl.HingeJoint#performAxisSpeed(float)}.
	 */
	@Test
	public void testPerformAxisSpeedMaxSpeed()
	{
		testee = new HingeJoint("Test", "percept", "effect", Vector3D.PLUS_I, -10, 10, 7, 20, 20, false);
		testee.performAxisSpeed(10);
		assertEquals(7, testee.getAngle(), 0.001);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.agent.model.agentmodel.impl.HingeJoint#performAxisPosition(double,
	 * float)}
	 * .
	 */
	@Test
	public void testPerformAxisPositionBiggerThanDefaultMaxSpeed()
	{
		testee = new HingeJoint("Test", "percept", "effect", Vector3D.PLUS_I, -20, 20, 7, 20, 20, false);
		testee.performAxisPosition(20, Integer.MAX_VALUE);
		assertEquals(20, testee.getAngle(), 0.001);
	}
}
