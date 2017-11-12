/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.IFlagPerceptor;

/**
 * Perceptor for holding flag information.
 *
 * @author Stefan Glaser
 */
public class FlagPerceptor extends Perceptor implements IFlagPerceptor
{
	/** The flag information. */
	int value;

	public FlagPerceptor(String flag, int value)
	{
		super(flag);
		this.value = value;
	}

	@Override
	public int getValue()
	{
		return value;
	}
}
