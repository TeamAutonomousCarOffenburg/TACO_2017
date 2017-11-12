/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.ITouchPerceptor;

/**
 * @author Simon Raffeiner
 */
public class TouchPerceptor extends Perceptor implements ITouchPerceptor
{
	private boolean state;

	public TouchPerceptor(String name, boolean state)
	{
		super(name);
		this.state = state;
	}

	@Override
	public boolean getState()
	{
		return this.state;
	}
}
