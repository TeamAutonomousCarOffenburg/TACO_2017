/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.misc;

import static org.junit.Assert.assertEquals;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

/**
 * @author Ingo Schindler
 *
 */
public class FuzzyCompareTest
{
	/**
	 * Test method for {@link hso.autonomy.util.misc.FuzzyCompare#eq(double, double, double)}
	 * .
	 */
	@Test
	public void testEq()
	{
		assertEquals("EQ 10 = 10 | Range 0", true, FuzzyCompare.eq(10, 10, 0));
		assertEquals("EQ 10 = 10 | Range 5", true, FuzzyCompare.eq(10, 10, 5));
		assertEquals("EQ 10 = 9 | Range 1", true, FuzzyCompare.eq(10, 9, 1));
		assertEquals("EQ 10 = 11 | Range 1", true, FuzzyCompare.eq(10, 11, 1));
		assertEquals("EQ 10 = 8 | Range 1", false, FuzzyCompare.eq(10, 8, 1));
		assertEquals("EQ 10 = 12 | Range 1", false, FuzzyCompare.eq(10, 12, 1));
	}

	/**
	 * Test method for {@link hso.autonomy.util.misc.FuzzyCompare#gt(double, double, double)}
	 * .
	 */
	@Test
	public void testGt()
	{
		assertEquals("GT 10 > 10 | Range 0", true, FuzzyCompare.gt(10, 10, 0));
		assertEquals("GT 10 > 10 | Range 5", true, FuzzyCompare.gt(10, 10, 5));
		assertEquals("GT 10 > 9 | Range 1", true, FuzzyCompare.gt(10, 9, 1));
		assertEquals("GT 10 > 11 | Range 1", true, FuzzyCompare.gt(10, 11, 1));
		assertEquals("GT 10 > 8 | Range 1", true, FuzzyCompare.gt(10, 8, 1));
		assertEquals("GT 10 > 12 | Range 1", false, FuzzyCompare.gt(10, 12, 1));
	}

	/**
	 * Test method for {@link hso.autonomy.util.misc.FuzzyCompare#lt(double, double, double)}
	 * .
	 */
	@Test
	public void testLt()
	{
		assertEquals("LT 10 < 10 | Range 0", true, FuzzyCompare.lt(10, 10, 0));
		assertEquals("LT 10 < 10 | Range 5", true, FuzzyCompare.lt(10, 10, 5));
		assertEquals("LT 10 < 9 | Range 1", true, FuzzyCompare.lt(10, 9, 1));
		assertEquals("LT 10 < 11 | Range 1", true, FuzzyCompare.lt(10, 11, 1));
		assertEquals("LT 10 < 8 | Range 1", false, FuzzyCompare.lt(10, 8, 1));
		assertEquals("LT 10 < 12 | Range 1", true, FuzzyCompare.lt(10, 12, 1));
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.misc.FuzzyCompare#gte(double, double, double)}.
	 */
	@Test
	public void testGte()
	{
		assertEquals("GTE 10 > 10 | Range 0", true, FuzzyCompare.gte(10, 10, 0));
		assertEquals("GTE 10 > 10 | Range 5", true, FuzzyCompare.gte(10, 10, 5));
		assertEquals("GTE 10 > 9 | Range 1", true, FuzzyCompare.gte(10, 9, 1));
		assertEquals("GTE 10 > 11 | Range 1", true, FuzzyCompare.gte(10, 11, 1));
		assertEquals("GTE 10 > 8 | Range 1", true, FuzzyCompare.gte(10, 8, 1));
		assertEquals("GTE 10 > 12 | Range 1", false, FuzzyCompare.gte(10, 12, 1));
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.misc.FuzzyCompare#lte(double, double, double)}.
	 */
	@Test
	public void testLte()
	{
		assertEquals("LTE 10 < 10 | Range 0", true, FuzzyCompare.lte(10, 10, 0));
		assertEquals("LTE 10 < 10 | Range 5", true, FuzzyCompare.lte(10, 10, 5));
		assertEquals("LTE 10 < 9 | Range 1", true, FuzzyCompare.lte(10, 9, 1));
		assertEquals("LTE 10 < 11 | Range 1", true, FuzzyCompare.lte(10, 11, 1));
		assertEquals("LTE 10 < 8 | Range 1", false, FuzzyCompare.lte(10, 8, 1));
		assertEquals("LTE 10 < 12 | Range 1", true, FuzzyCompare.lte(10, 12, 1));
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.misc.FuzzyCompare#eq(Vector3D, Vector3D, double)}.
	 */
	@Test
	public void testEq3D()
	{
		// Difference on the right side
		assertEquals("EQ (1, 2, 3) == (2, 2, 3) | Range 0", false,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 3.0), new Vector3D(2.0, 2.0, 3.0), 0.0));
		assertEquals("EQ (1, 2, 3) == (2, 2, 3) | Range 1", true,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 3.0), new Vector3D(2.0, 2.0, 3.0), 1.0));

		assertEquals("EQ (1, 2, 3) == (1, 3, 3) | Range 0", false,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 3.0), new Vector3D(1.0, 3.0, 3.0), 0.0));
		assertEquals("EQ (1, 2, 3) == (1, 3, 3) | Range 1", true,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 3.0), new Vector3D(1.0, 3.0, 3.0), 1.0));

		assertEquals("EQ (1, 2, 3) == (1, 2, 4) | Range 0", false,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 3.0), new Vector3D(1.0, 2.0, 4.0), 0.0));
		assertEquals("EQ (1, 2, 3) == (1, 2, 4) | Range 1", true,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 3.0), new Vector3D(1.0, 2.0, 4.0), 1.0));

		// Difference on the left side
		assertEquals("EQ (2, 2, 3) == (1, 2, 3) | Range 0", false,
				FuzzyCompare.eq(new Vector3D(2.0, 2.0, 3.0), new Vector3D(1.0, 2.0, 3.0), 0.0));
		assertEquals("EQ (2, 2, 3) == (1, 2, 3) | Range 1", true,
				FuzzyCompare.eq(new Vector3D(2.0, 2.0, 3.0), new Vector3D(1.0, 2.0, 3.0), 1.0));

		assertEquals("EQ (1, 3, 3) == (1, 2, 3) | Range 0", false,
				FuzzyCompare.eq(new Vector3D(1.0, 3.0, 3.0), new Vector3D(1.0, 2.0, 3.0), 0.0));
		assertEquals("EQ (1, 3, 3) == (1, 2, 3) | Range 1", true,
				FuzzyCompare.eq(new Vector3D(1.0, 3.0, 3.0), new Vector3D(1.0, 2.0, 3.0), 1.0));

		assertEquals("EQ (1, 2, 4) == (1, 2, 3) | Range 0", false,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 4.0), new Vector3D(1.0, 2.0, 3.0), 0.0));
		assertEquals("EQ (1, 2, 4) == (1, 2, 3) | Range 1", true,
				FuzzyCompare.eq(new Vector3D(1.0, 2.0, 4.0), new Vector3D(1.0, 2.0, 3.0), 1.0));
	}
}
