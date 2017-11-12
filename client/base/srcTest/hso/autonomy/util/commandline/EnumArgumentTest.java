/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.commandline;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import hso.autonomy.util.commandline.ArgumentParsingException;
import hso.autonomy.util.commandline.EnumArgument;

public class EnumArgumentTest
{
	private enum TestEnum { VALUE, OTHER_VALUE }

	@Test
	public void testValidValues()
	{
		EnumArgument<TestEnum> arg = new EnumArgument<>("test", TestEnum.VALUE, "", TestEnum.class);

		assertEquals(arg.parse(""), TestEnum.VALUE);
		assertEquals(arg.parse("--test=value"), TestEnum.VALUE);
		assertEquals(arg.parse("--test=otherValue"), TestEnum.OTHER_VALUE);
	}

	@Test
	public void testNullDefault()
	{
		EnumArgument<TestEnum> arg = new EnumArgument<>("test", null, "", TestEnum.class);
		assertEquals(arg.parse(""), null);
	}

	@Test(expected = ArgumentParsingException.class)
	public void testInvalidValue()
	{
		EnumArgument<TestEnum> arg = new EnumArgument<>("test", null, "", TestEnum.class);
		assertEquals(arg.parse("--test=invalid"), null);
	}
}
