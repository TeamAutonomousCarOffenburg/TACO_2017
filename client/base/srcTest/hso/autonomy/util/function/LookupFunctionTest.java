/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.function;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import hso.autonomy.util.function.LookupFunction;

/**
 *
 * @author kdorer
 */
public class LookupFunctionTest
{
	private LookupFunction testee;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception
	{
		float[] data = {0, 1, 2, 3};
		testee = new LookupFunction(data, 1, 0);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.function.LookupMatrix#getValue(double, double)}.
	 */
	@Test
	public void testGetValue()
	{
		assertEquals(0, testee.getValue(0), 0.0001);
		assertEquals(3, testee.getValue(3), 0.0001);
		assertEquals(0.5, testee.getValue(0.5), 0.0001);
		assertEquals(0, testee.getValue(-3), 0.0001);
		assertEquals(3, testee.getValue(4), 0.0001);
		assertEquals(0.9, testee.getValue(0.9), 0.0001);
	}

	// commented, architecture violation, only manually triggered
	// @Test
	// public void testReadFromFile()
	// {
	// testee = new LookupFunction(
	// "motorMappings/JointToMotorLookupKneeM1LEFT.csv");
	//
	// // test difference to analytical method
	// IJointToMotorMapper mapper = new SweatyKneeJointToMotorMapper(
	// WhichKnee.LEFT);
	//
	// for (double i = 0; i < 30; i += 0.1) {
	// double[] jointAngles = { i };
	// double value1 = mapper.jointToMotorAngle(jointAngles)[0];
	// double value2 = testee.getValue(i);
	// double delta = value2 - value1;
	// if (delta > 1.0) {
	// System.out.println("Delta: " + delta + " x:" + i);
	// }
	// }
	// }
}
