/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.commandline;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import hso.autonomy.util.commandline.ArgumentParsingException;
import hso.autonomy.util.commandline.StringArgument;

public class StringArgumentTest
{
	private StringArgument testee;

	@Before
	public void setUp()
	{
		testee = new StringArgument("argument", "defaultValue", null);
	}

	@Test
	public void testParseCorrectArgument()
	{
		testParse("value", "--argument=value");
	}

	@Test
	public void testParseIncorrectArgument()
	{
		testParse("defaultValue", "-argument=value");
		testParse("defaultValue", "--arg=value");
	}

	@Test
	public void testParseMultipleArgs()
	{
		testParse("value", "--argument=value", "--argument2=value", "--argument3=value");
	}

	@Test
	public void testParseNull()
	{
		testParse("defaultValue", (String[]) null);
	}

	@Test
	public void testParseNullDefaultValue()
	{
		testee = new StringArgument("argument", null, null);
		testParse(null, "--invalid=invalid");
	}

	@Test
	public void testParseUseFirstValue()
	{
		testParse("value1", "--argument=value1", "--argument=value2", "--argument=value3");
	}

	@Test(expected = ArgumentParsingException.class)
	public void testParseRequiredArgument()
	{
		testee.setRequired(true);
		testParse("", "");
	}

	@Test
	public void testParseOptionalArgument()
	{
		testee.setRequired(false);
		testParse("defaultValue", "");
	}

	private void testParse(String expected, String... args)
	{
		assertEquals(expected, testee.parse(args));
	}
}
