/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.action.impl;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import hso.autonomy.agent.communication.action.IAction;
import hso.autonomy.agent.communication.action.IActionPerformer;
import hso.autonomy.agent.communication.action.IEffector;

/**
 * This class generates server messages from Effector objects.
 */
public class Action implements IAction
{
	protected Map<String, IEffector> effectors;

	protected Map<String, IEffector> actionEffectors;

	private IActionPerformer actionPerformer;

	protected float maxGain;

	/**
	 * Initializes all effectors that occur in the simulation
	 *
	 * @param actionPerformer the component that can send the actions
	 */
	public Action(IActionPerformer actionPerformer)
	{
		this.actionPerformer = actionPerformer;
		effectors = new HashMap<>();
		maxGain = 0;
	}

	@Override
	public void setEffectorValues(String name, float... values)
	{
		effectors.get(name).setEffectorValues(maxGain, values);
	}

	@Override
	public void sendAction()
	{
		send(actionEffectors);

		actionEffectors.values().forEach(IEffector::resetAfterAction);
	}

	protected void send(Map<String, IEffector> effectors)
	{
		actionPerformer.performAction(effectors);
	}

	@Override
	public void setMaxGain(float motorGain)
	{
		maxGain = motorGain;
	}

	@Override
	public float getMaxGain()
	{
		return maxGain;
	}

	@Override
	public void sendSync()
	{
		send(Collections.emptyMap());
	}
}
