/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action.impl;

/**
 * Implementation of the "CompositeEffector" effector, used to move robot joints
 *
 * @author Klaus Dorer
 */
public class CompositeEffector extends Effector
{
	private HingeEffector[] effectors;

	/**
	 * Instantiates a new UniversalEffector and initializes all fields to zero
	 *
	 * @param name Effector name
	 */
	public CompositeEffector(String name, HingeEffector[] effectors)
	{
		super(name);

		this.effectors = effectors;
	}

	/**
	 * Set axis movement speeds
	 */
	@Override
	public void setEffectorValues(float maxGain, float... values)
	{
		for (int i = 0; i < effectors.length; i++) {
			int position = i * 5;
			effectors[i].setEffectorValues(maxGain, values[position], values[position + 1], values[position + 2],
					values[position + 3], values[position + 4]);
		}
	}

	@Override
	public void resetAfterAction()
	{
		for (HingeEffector effector : effectors) {
			effector.resetAfterAction();
		}
	}

	public float getSpeed(int i)
	{
		return effectors[i].getSpeed();
	}

	public float getDesiredAngle(int i)
	{
		return effectors[i].getDesiredAngle();
	}

	public HingeEffector getEffector(int i)
	{
		return effectors[i];
	}

	public int size()
	{
		return effectors.length;
	}
}
