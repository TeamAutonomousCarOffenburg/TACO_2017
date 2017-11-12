/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.thoughtmodel.impl;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.agent.model.worldmodel.IVisibleObject;
import hso.autonomy.agent.model.worldmodel.IWorldModel;

/**
 * @author Stefan Glaser
 */
public abstract class ThoughtModel implements IThoughtModel, Serializable
{
	private final IAgentModel agentModel;

	private final IWorldModel worldModel;

	public ThoughtModel(IAgentModel agentModel, IWorldModel worldModel)
	{
		this.agentModel = agentModel;
		this.worldModel = worldModel;
	}

	@Override
	public IAgentModel getAgentModel()
	{
		return agentModel;
	}

	@Override
	public IWorldModel getWorldModel()
	{
		return worldModel;
	}

	@Override
	public boolean update(IPerception perception)
	{
		boolean result = false;
		if (perception != null) {
			// Trigger agent model to update all internal sensor values
			result = agentModel.update(perception);

			// Trigger world model to process vision information
			result = worldModel.update(perception) || result;
		}

		return result;
	}
}
