package taco.agent.decision.behavior.base;

import hso.autonomy.agent.decision.behavior.basic.Behavior;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;

public class AudiCupBehavior extends Behavior
{
	protected static final String BEHAVIOR_DRAWING = "behaviorDrawing";

	public AudiCupBehavior(String name, IThoughtModel thoughtModel)
	{
		super(name, thoughtModel);
		init(); // this should probably be done in the Behavior base class?
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

	@Override
	public IAudiCupAgentModel getAgentModel()
	{
		return (IAudiCupAgentModel) super.getAgentModel();
	}
}
