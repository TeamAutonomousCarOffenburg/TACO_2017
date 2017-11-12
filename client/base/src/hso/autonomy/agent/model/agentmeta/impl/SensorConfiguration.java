/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta.impl;

import hso.autonomy.agent.model.agentmeta.ISensorConfiguration;

/**
 * @author Stefan Glaser
 */
public class SensorConfiguration implements ISensorConfiguration
{
	private final String name;

	private final String perceptorName;

	public SensorConfiguration(String name, String perceptorName)
	{
		this.name = name;
		this.perceptorName = perceptorName;
	}

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public String getPerceptorName()
	{
		return perceptorName;
	}
}
