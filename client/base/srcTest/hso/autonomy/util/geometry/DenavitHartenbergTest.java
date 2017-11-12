/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.geometry;

import static org.junit.Assert.assertEquals;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

import hso.autonomy.util.geometry.DenavitHartenberg;
import hso.autonomy.util.geometry.DenavitHartenbergParameters;

/**
 * @author Simon Raffeiner
 *
 */
public class DenavitHartenbergTest
{
	/**
	 * Test method for
	 * {@link hso.autonomy.util.geometry.DenavitHartenberg#transform(Vector3D,
	 * DenavitHartenbergParameters)}
	 */
	@Test
	public void testTransform()
	{
		Vector3D origin = new Vector3D(2.0, 1.0, 3.0);
		DenavitHartenbergParameters p = new DenavitHartenbergParameters(0, 0, 0, new Vector3D(-0.12, -0.005, 0));

		Vector3D result = DenavitHartenberg.transform(origin, p);

		assertEquals(1.88, result.getX(), 0.001);
		assertEquals(0.995, result.getY(), 0.001);
		assertEquals(3.0, result.getZ(), 0.001);

		// Change theta value and try again
		p.setTheta(32.0);
		result = DenavitHartenberg.transform(origin, p);
		assertEquals(1.046, result.getX(), 0.001);
		assertEquals(1.902, result.getY(), 0.001);
		assertEquals(3.0, result.getZ(), 0.001);
	}
}
