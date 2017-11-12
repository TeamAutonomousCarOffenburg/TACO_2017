package taco.agent.decision.decisionmaker.impl;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.decision.decisionmaker.impl.DecisionMakerBase;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;

public abstract class AudiCupDecisionMakerBase extends DecisionMakerBase implements IBehaviorConstants
{
	public AudiCupDecisionMakerBase(BehaviorMap behaviors, IThoughtModel thoughtModel)
	{
		super(behaviors, thoughtModel);
	}

	@Override
	public IAudiCupAgentModel getAgentModel()
	{
		return (IAudiCupAgentModel) super.getAgentModel();
	}

	@Override
	public IAudiCupWorldModel getWorldModel()
	{
		return (IAudiCupWorldModel) super.getWorldModel();
	}

	@Override
	public IAudiCupThoughtModel getThoughtModel()
	{
		return (IAudiCupThoughtModel) super.getThoughtModel();
	}
}
