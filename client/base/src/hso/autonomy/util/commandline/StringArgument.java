/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.commandline;

import java.util.Arrays;

public class StringArgument extends Argument<String>
{
	private String[] allowedValues;

	public StringArgument(String name, String defaultValue, String description)
	{
		this(name, defaultValue, null, description);
	}

	public StringArgument(String name, String defaultValue, String[] allowedValues, String description)
	{
		super(name, defaultValue, description);
		this.allowedValues = allowedValues;
	}

	@Override
	protected String extractValue(String value)
	{
		if (allowedValues != null && !Arrays.asList(allowedValues).contains(value)) {
			error("%s Available values are: <%s>", getInvalidArgString(value), getValues());
		}
		return value;
	}

	private String getValues()
	{
		if (allowedValues == null) {
			return "";
		}

		String values = "";
		for (String allowedValue : allowedValues) {
			values += allowedValue + "|";
		}
		if (values.length() > 0) {
			values = values.substring(0, values.length() - 1);
		}
		return values;
	}

	@Override
	protected String getValueHelp()
	{
		String values = getValues();
		if (values.equals("")) {
			return "string";
		}
		return values;
	}
}
