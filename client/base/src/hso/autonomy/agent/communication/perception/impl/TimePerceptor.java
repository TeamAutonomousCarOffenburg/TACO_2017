/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import hso.autonomy.agent.communication.perception.ITimerPerceptor;

/**
 * @author Simon Raffeiner
 */
public class TimePerceptor extends Perceptor implements ITimerPerceptor
{
	private float time;

	public TimePerceptor()
	{
		this(0.00f);
	}

	public TimePerceptor(float time)
	{
		super("time");
		this.time = time;
	}

	@Override
	public float getTime()
	{
		return time;
	}
}
