/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import hso.autonomy.agent.model.agentmodel.IJoint;

/**
 * Generic joint representation
 *
 * @author Stefan Glaser
 */
public abstract class Joint extends Sensor implements IJoint
{
	Joint(String name, String perceptorName)
	{
		super(name, perceptorName);
	}

	/**
	 * Copy constructor
	 *
	 * @param source to copy from
	 */
	Joint(Joint source)
	{
		super(source);
	}
}
