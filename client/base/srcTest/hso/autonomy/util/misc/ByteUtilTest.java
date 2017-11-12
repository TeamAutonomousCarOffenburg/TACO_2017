/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.misc;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 *
 * @author kdorer
 */
public class ByteUtilTest
{
	/**
	 * Test method for {@link hso.autonomy.util.misc.ByteUtil#getShortFromBytes(byte, byte)}.
	 */
	@Test
	public void testGetShortFromBytes()
	{
		assertEquals(0, ByteUtil.getShortFromBytes((byte) 0, (byte) 0));
		assertEquals(10, ByteUtil.getShortFromBytes((byte) 0, (byte) 10));
		assertEquals(256, ByteUtil.getShortFromBytes((byte) 1, (byte) 0));
		assertEquals(3170, ByteUtil.getShortFromBytes((byte) 12, (byte) 98));
		// assertEquals(255, ByteUtil.getShortFromBytes((byte) 0, (byte) -128));
	}

	/**
	 * Test method for {@link hso.autonomy.util.misc.ByteUtil#getHighByte(short)}.
	 */
	@Test
	public void testGetHighByte()
	{
		assertEquals(0, ByteUtil.getHighByte((short) 255));
		assertEquals(1, ByteUtil.getHighByte((short) 256));
	}

	/**
	 * Test method for {@link hso.autonomy.util.misc.ByteUtil#getLowByte(short)}.
	 */
	@Test
	public void testGetLowByte()
	{
		assertEquals(-1, ByteUtil.getLowByte((short) 255));
		assertEquals(127, ByteUtil.getLowByte((short) 127));
		assertEquals(0, ByteUtil.getLowByte((short) 256));
	}

	@Test
	public void testFindBytesTrue()
	{
		byte[] toSearchIn = {1, 2, 3, 4, 5};
		byte[] toSearch = {1, 2, 3, 4, 5};
		assertTrue(ByteUtil.findBytes(toSearchIn, toSearch));
	}

	@Test
	public void testFindBytesTrue2()
	{
		byte[] toSearchIn = {1, 2, 3, 4, 5};
		byte[] toSearch = {5};
		assertTrue(ByteUtil.findBytes(toSearchIn, toSearch));
	}

	@Test
	public void testFindBytesTrue3()
	{
		byte[] toSearchIn = {1, 2, 3, 4, 5};
		byte[] toSearch = {1};
		assertTrue(ByteUtil.findBytes(toSearchIn, toSearch));
	}

	@Test
	public void testFindBytesFalse()
	{
		byte[] toSearchIn = {1, 2, 3, 4, 5};
		byte[] toSearch = {1, 2, 3, 4, 5, 6};
		assertFalse(ByteUtil.findBytes(toSearchIn, toSearch));
	}

	@Test
	public void testFindBytesFalse2()
	{
		byte[] toSearchIn = {1, 2, 3, 4, 5};
		byte[] toSearch = {1, 2, 4};
		assertFalse(ByteUtil.findBytes(toSearchIn, toSearch));
	}

	@Test
	public void testFindBytesFalse3()
	{
		byte[] toSearchIn = {1, 2, 3, 4, 5};
		byte[] toSearch = {2, 3, 5};
		assertFalse(ByteUtil.findBytes(toSearchIn, toSearch));
	}
}
