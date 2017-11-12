/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import hso.autonomy.util.misc.FuzzyCompare;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 *
 * @author kdorer
 */
public class SphereTest
{
	private Sphere testee;

	/**
	 * @throws java.lang.Exception
	 */
	@Before
	public void setUp() throws Exception
	{
		testee = new Sphere(new Vector3D(1, -2, -3), Math.sqrt(14));
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.geometry.Sphere#intersect(hso.autonomy.util.geometry.Circle3D)}
	 * .
	 */
	@Test
	public void testIntersectTwoPointSameXZ()
	{
		testee = new Sphere(new Vector3D(0, -1, 0), 1);
		Circle3D circle = new Circle3D(new Vector3D(0, 0, 0), 1);
		Vector3D[] intersect = testee.intersect(circle);

		double cos = Math.cos(Math.toRadians(60));
		double sin = Math.sin(Math.toRadians(60));

		Vector3D expected = new Vector3D(0, -cos, sin);
		testTrue(expected, intersect[0]);
		expected = new Vector3D(0, -cos, -sin);
		testTrue(expected, intersect[1]);
	}

	@Test
	public void testIntersectOrigin()
	{
		Circle3D circle = new Circle3D(new Vector3D(0, 1, 0), 1);
		Vector3D[] intersect = testee.intersect(circle);

		Vector3D expected = Vector3D.ZERO;
		testTrue(expected, intersect[0]);
		expected = new Vector3D(0, 1, -1);
		testTrue(expected, intersect[1]);
	}

	@Test
	public void testIntersectOnePoint()
	{
		testee = new Sphere(new Vector3D(2, 2, 2), Math.sqrt(12));
		Circle3D circle = new Circle3D(new Vector3D(0, -1, -1), Math.sqrt(2));
		Vector3D[] intersect = testee.intersect(circle);

		Vector3D expected = Vector3D.ZERO;
		testTrue(expected, intersect[0]);
		expected = Vector3D.ZERO;
		testTrue(expected, intersect[1]);
	}

	@Test
	public void testNoIntersect()
	{
		Circle3D circle = new Circle3D(new Vector3D(0, 3, 0), 1);
		Vector3D[] intersect = testee.intersect(circle);
		assertEquals(0, intersect.length);
	}

	protected void testTrue(Vector3D expected, Vector3D intersect)
	{
		assertTrue("Not equal: " + expected + "/" + intersect, FuzzyCompare.eq(expected, intersect, 0.0001));
	}
}
