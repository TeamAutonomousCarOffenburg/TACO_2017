/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.IPerceptor;

/**
 * @author Simon Raffeiner
 */
public abstract class Perceptor implements IPerceptor
{
	private final String name;

	public Perceptor()
	{
		this("default");
	}

	public Perceptor(String name)
	{
		this.name = name;
	}

	@Override
	public String getName()
	{
		return name;
	}
}
