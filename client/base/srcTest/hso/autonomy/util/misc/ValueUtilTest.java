/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.misc;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 *
 * @author kdorer
 */
public class ValueUtilTest
{
	/**
	 * Test method for {@link hso.autonomy.util.misc.ValueUtil#limitAbs(float, float)}.
	 */
	@Test
	public void testLimitAbsFloatFloat()
	{
		assertEquals(9, ValueUtil.limitAbs(10, 9), 0.001);
		assertEquals(-9, ValueUtil.limitAbs(-10, 9), 0.001);
		assertEquals(8, ValueUtil.limitAbs(8, 9), 0.001);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.misc.ValueUtil#getValueAdjustment(double, double, double,
	 * double)}
	 * .
	 */
	@Test
	public void testGetValueAdjustment()
	{
		assertEquals(1, ValueUtil.getValueAdjustment(5, 7, 1, 2), 0.001);
		assertEquals(-2, ValueUtil.getValueAdjustment(5, 3, 1, 2), 0.001);
		assertEquals(0.5, ValueUtil.getValueAdjustment(5, 5.5, 1, 2), 0.001);
		assertEquals(-0.5, ValueUtil.getValueAdjustment(5, 4.5, 1, 2), 0.001);
	}

	@Test
	public void testGetScale()
	{
		double[] values = {1, -2, 3};
		assertEquals(1, ValueUtil.getScale(values, 7), 0.001);
		assertEquals(0.5, ValueUtil.getScale(values, 3), 0.001);
	}
}
