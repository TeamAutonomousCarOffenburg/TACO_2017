/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.commandline;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import hso.autonomy.util.commandline.ArgumentParsingException;
import hso.autonomy.util.commandline.IntegerArgument;

public class IntegerArgumentTest
{
	private IntegerArgument testee;

	@Before
	public void setUp()
	{
		testee = new IntegerArgument("argument", 5, 3, 10, null);
	}

	@Test
	public void testParseAllowedValue()
	{
		testParse(3, "--argument=3");
		testParse(7, "--argument=7");
		testParse(10, "--argument=10");
	}

	@Test(expected = ArgumentParsingException.class)
	public void testParseValueTooSmall()
	{
		testee.parse("--argument=2");
		testee.parse("--argument=-30");
	}

	@Test(expected = ArgumentParsingException.class)
	public void testParseValueTooBig()
	{
		testParse(5, "--argument=11");
		testParse(5, "--argument=100");
	}

	@Test(expected = ArgumentParsingException.class)
	public void testParseEmptyValue()
	{
		testParse(5, "--argument=");
	}

	@Test(expected = ArgumentParsingException.class)
	public void testParseNonInteger()
	{
		testParse(5, "--argument=notInt");
	}

	@Test
	public void testConstructorNoBounds()
	{
		testee = new IntegerArgument("argument", 5, null);
		testParse(Integer.MIN_VALUE, "--argument=" + Integer.MIN_VALUE);
		testParse(Integer.MAX_VALUE, "--argument=" + Integer.MAX_VALUE);
	}

	@Test
	public void testConstructorNoMaxValue()
	{
		testee = new IntegerArgument("argument", 5, 3, null);
		testParse(Integer.MAX_VALUE, "--argument=" + Integer.MAX_VALUE);
	}

	private void testParse(Integer expected, String... args)
	{
		assertEquals(expected, testee.parse(args));
	}
}
