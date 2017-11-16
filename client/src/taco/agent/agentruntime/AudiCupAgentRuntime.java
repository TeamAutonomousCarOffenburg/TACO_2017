package taco.agent.agentruntime;

import java.util.Map;

import hso.autonomy.agent.agentruntime.AgentRuntime;
import hso.autonomy.agent.communication.perception.IPerceptor;
import kdo.util.parameter.ParameterMap;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;

public class AudiCupAgentRuntime extends AgentRuntime
{
	private PerceptionLogger perceptionLogger;

	public AudiCupAgentRuntime(AudiCupAgentRuntimeParameters runtimeParameters)
	{
		super();

		ComponentFactory factory = runtimeParameters.getFactory();
		channelManager = factory.createChannelManager(runtimeParameters.getServer());
		agentMetaModel =
				factory.createAgentMetaModel(runtimeParameters.getMetaModelDirectory(), runtimeParameters.getVersion());

		IAudiCupAgentModel agentModel = factory.createAgentModel(agentMetaModel);
		IAudiCupWorldModel worldModel =
				factory.createWorldModel(agentModel, runtimeParameters.getScenario(), runtimeParameters.getStartPose());
		thoughtModel = factory.createThoughtModel(agentModel, worldModel);

		action = factory.createAction(channelManager, agentMetaModel);
		perception = factory.createPerception();

		ParameterMap parameterMap = factory.createParameters(runtimeParameters.getParameterMap());
		behaviors = factory.createBehaviors(getThoughtModel(), parameterMap);
		decisionMaker = factory.createDecisionMaker(
				behaviors, thoughtModel, runtimeParameters.getScenario().getDriveBehavior(), parameterMap);

		runtimeParameters.getScenario().configureDriveBehavior(
				behaviors.get(runtimeParameters.getScenario().getDriveBehavior()));

		if (runtimeParameters.isLog()) {
			perceptionLogger =
					new PerceptionLogger(runtimeParameters.getVersion(), runtimeParameters.getScenario().getName());
			perceptionLogger.start();
		}
	}

	@Override
	public void update(Map<String, IPerceptor> content)
	{
		if (perceptionLogger != null) {
			perceptionLogger.log(content);
		}

		super.update(content);
	}

	@Override
	protected void onClientStopped()
	{
		if (perceptionLogger != null) {
			perceptionLogger.stop();
		}
	}

	@Override
	public IAudiCupAgentModel getAgentModel()
	{
		return (IAudiCupAgentModel) super.getAgentModel();
	}

	@Override
	public IAudiCupPerception getPerception()
	{
		return (IAudiCupPerception) super.getPerception();
	}

	@Override
	public IAudiCupAction getAction()
	{
		return (IAudiCupAction) super.getAction();
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
