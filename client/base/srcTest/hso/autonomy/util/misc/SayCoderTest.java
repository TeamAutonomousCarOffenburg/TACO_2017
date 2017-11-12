/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.misc;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Test;

/**
 *
 * @author dorer
 */
public class SayCoderTest
{
	/**
	 * Test method for {@link hso.autonomy.util.misc.SayCoder#encodeID(int)}.
	 */
	@Test
	public void testEncodeID()
	{
		assertEquals("a", SayCoder.encodeID(0));
		assertEquals("b", SayCoder.encodeID(1));
		assertEquals("k", SayCoder.encodeID(10));
	}

	/**
	 * Test method for {@link hso.autonomy.util.misc.SayCoder#decodeID(java.lang.String)}.
	 */
	@Test
	public void testDecodeID()
	{
		assertEquals(0, SayCoder.decodeID("a"));
		assertEquals(10, SayCoder.decodeID("k"));
	}

	@Test
	public void testEncodeDecodePosition()
	{
		String encodePosition = SayCoder.encodePosition(new Vector3D(12.34, -5.67, 0));
		Vector3D result = SayCoder.decodePosition(encodePosition);
		assertEquals(12.3, result.getX(), 0.00001);
		assertEquals(-5.7, result.getY(), 0.00001);

		encodePosition = SayCoder.encodePosition(new Vector3D(20.34, -19.41, 0));
		result = SayCoder.decodePosition(encodePosition);
		assertEquals(20.3, result.getX(), 0.00001);
		assertEquals(-19.5, result.getY(), 0.00001);

		assertNull(SayCoder.decodePosition("2a0200"));
		assertNull(SayCoder.decodePosition("2002000"));
	}

	@Test
	public void testEncodeName()
	{
		assertEquals("t", SayCoder.encodeName("magmaOffenburg"));
		assertEquals("e", SayCoder.encodeName("magmaOpponent"));
		assertEquals("p", SayCoder.encodeName("magma"));
		assertEquals("q", SayCoder.encodeName("magmaOpp"));
	}

	@Test
	public void testEncodeTwoDigit()
	{
		assertEquals("aa", SayCoder.encodeTwoDigits(0));
		assertEquals("az", SayCoder.encodeTwoDigits(25));
		assertEquals("ba", SayCoder.encodeTwoDigits(26));
	}

	@Test
	public void testDecodeTwoDigit()
	{
		assertEquals(0, SayCoder.decodeTwoDigits("aa"));
		assertEquals(26, SayCoder.decodeTwoDigits("ba"));
	}
}
