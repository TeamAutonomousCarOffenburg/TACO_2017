/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmodel.impl.ik.IAgentIKSolver;

/**
 * Implementation of the AgentModel. Used to represent all the information the
 * agent has about itself.
 *
 * @author Stefan Glaser, Ingo Schindler
 */
public class NoPerceptionAgentModel extends AgentModel
{
	/**
	 * initializes all known Sensors like: HingeJoints, ForceResistances and
	 * GyroRates
	 */
	public NoPerceptionAgentModel(IAgentMetaModel metaModel, IAgentIKSolver ikSolver)
	{
		super(metaModel, ikSolver);
	}

	@Override
	public boolean update(IPerception perception)
	{
		// we ignore the perception here and do as if our movement was done
		// completely
		bodyModelSensed = bodyModelFuture;
		// checkForDiscrepancies();
		bodyModelExpected = bodyModelFuture;
		// bodyModelExpected.updateFromPerception(perception);

		bodyModelFuture = createBodyModel(bodyModelSensed);

		// now inform observers about changes
		observer.onStateChange(this);
		return true;
	}
}
