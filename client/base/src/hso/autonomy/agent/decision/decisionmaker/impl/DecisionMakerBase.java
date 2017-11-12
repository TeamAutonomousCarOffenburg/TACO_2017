/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.decision.decisionmaker.impl;

import java.io.Serializable;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.agent.decision.decisionmaker.IDecisionMaker;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.agent.model.worldmodel.IWorldModel;

/**
 * Base class for all decision makers
 * @author Klaus Dorer
 */
public abstract class DecisionMakerBase implements IDecisionMaker, Serializable
{
	/** the behavior that is currently selected */
	protected IBehavior currentBehavior;

	/** the behaviors that are available */
	protected transient final BehaviorMap behaviors;

	/** link to the thought model */
	private final transient IThoughtModel thoughtModel;

	/** the number of decisions taken */
	protected int numberOfDecisions;

	/** the behavior that is planned to switch to */
	private IBehavior desiredBehavior;

	public DecisionMakerBase(BehaviorMap behaviors, IThoughtModel thoughtModel)
	{
		this.behaviors = behaviors;
		this.thoughtModel = thoughtModel;
		reset();
	}

	public IThoughtModel getThoughtModel()
	{
		return thoughtModel;
	}

	public IWorldModel getWorldModel()
	{
		return thoughtModel.getWorldModel();
	}

	public IAgentModel getAgentModel()
	{
		return thoughtModel.getAgentModel();
	}

	/**
	 * Retrieve a specific behavior
	 *
	 * @param name Behavior name
	 * @return The specified behavior, null if no matching behavior was found
	 */
	@Override
	public IBehavior getBehavior(String name)
	{
		return behaviors.get(name);
	}

	/**
	 * Retrieve a list of all behaviors
	 *
	 * @return Behavior list
	 */
	public BehaviorMap getBehaviors()
	{
		return behaviors;
	}

	/**
	 * Retrieve the number of times decide() was called
	 *
	 * @return Call count
	 */
	public int getNumberOfDecisions()
	{
		return numberOfDecisions;
	}

	@Override
	public boolean decide()
	{
		numberOfDecisions++;

		String desiredBehaviorName;
		if (isPaused()) {
			desiredBehaviorName = getPausedBehavior();
		} else {
			desiredBehaviorName = decideNextBehavior();
		}
		desiredBehavior = behaviors.get(desiredBehaviorName);
		if (desiredBehavior != currentBehavior) {
			currentBehavior = desiredBehavior.switchFrom(currentBehavior);
		} else {
			currentBehavior.stayIn();
		}

		currentBehavior.perform();
		onBehaviorPerformed();
		return true;
	}

	protected void onBehaviorPerformed()
	{
	}

	protected boolean isPaused()
	{
		return false;
	}

	protected String getPausedBehavior()
	{
		return IBehavior.NONE;
	}

	/**
	 * Decide which behavior to use next
	 *
	 * @return Next behavior
	 */
	public abstract String decideNextBehavior();

	@Override
	public IBehavior getCurrentBehavior()
	{
		return currentBehavior;
	}

	@Override
	public IBehavior getDesiredBehavior()
	{
		return desiredBehavior;
	}

	@Override
	public void reset()
	{
		numberOfDecisions = 0;
		if (behaviors != null) {
			currentBehavior = behaviors.get(IBehavior.NONE);
		}
		desiredBehavior = currentBehavior;
	}
}
