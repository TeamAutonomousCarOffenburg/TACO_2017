package taco.agent.decision.behavior.base;

import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.agent.decision.behavior.complex.ComplexBehavior;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;

import java.util.Collections;
import java.util.List;

public abstract class AudiCupComplexBehavior extends ComplexBehavior
{
	protected static final String BEHAVIOR_DRAWING = "behaviorDrawing";

	public AudiCupComplexBehavior(String name, IThoughtModel thoughtModel, BehaviorMap behaviors)
	{
		super(name, thoughtModel, behaviors);
		init(); // this should probably be done in the Behavior base class?
	}

	@Override
	protected List<IBehavior> decideNextBasicBehaviors()
	{
		return Collections.singletonList(behaviors.get(decideNextBasicBehavior()));
	}

	protected abstract String decideNextBasicBehavior();

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

	@Override
	public IAudiCupAgentModel getAgentModel()
	{
		return (IAudiCupAgentModel) super.getAgentModel();
	}
}
