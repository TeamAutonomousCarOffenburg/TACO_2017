package taco.agent.agentruntime;

import hso.autonomy.agent.communication.action.IActionPerformer;
import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.decision.decisionmaker.IDecisionMaker;
import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.IPose2D;
import kdo.util.parameter.ParameterMap;
import taco.agent.agentruntime.scenarios.IScenario;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.model.agentmeta.impl.CarMetaModelVersion;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;

public class ComponentFactoryDecorator extends ComponentFactory
{
	protected final ComponentFactory decoratee;

	public ComponentFactoryDecorator(ComponentFactory decoratee)
	{
		this.decoratee = decoratee;
	}

	@Override
	public IChannelManager createChannelManager(String server)
	{
		return decoratee.createChannelManager(server);
	}

	@Override
	public IAgentMetaModel createAgentMetaModel(String metaModelDirectory, CarMetaModelVersion version)
	{
		return decoratee.createAgentMetaModel(metaModelDirectory, version);
	}

	@Override
	public IAudiCupAgentModel createAgentModel(IAgentMetaModel agentMetaModel)
	{
		return decoratee.createAgentModel(agentMetaModel);
	}

	@Override
	public IAudiCupWorldModel createWorldModel(IAgentModel agentModel, IScenario scenario, IPose2D startPose)
	{
		return decoratee.createWorldModel(agentModel, scenario, startPose);
	}

	@Override
	public IAudiCupThoughtModel createThoughtModel(IAgentModel agentModel, IAudiCupWorldModel worldModel)
	{
		return decoratee.createThoughtModel(agentModel, worldModel);
	}

	@Override
	public IAudiCupPerception createPerception()
	{
		return decoratee.createPerception();
	}

	@Override
	public IAudiCupAction createAction(IActionPerformer actionPerformer, IAgentMetaModel agentMetaModel)
	{
		return decoratee.createAction(actionPerformer, agentMetaModel);
	}

	@Override
	public BehaviorMap createBehaviors(IAudiCupThoughtModel thoughtModel, ParameterMap params)
	{
		return decoratee.createBehaviors(thoughtModel, params);
	}

	@Override
	public ParameterMap createParameters(ParameterMap fromExtern)
	{
		return decoratee.createParameters(fromExtern);
	}

	@Override
	protected ParameterMap createSpecificParameters()
	{
		return new ParameterMap();
	}

	@Override
	public IDecisionMaker createDecisionMaker(
			BehaviorMap behaviors, IThoughtModel thoughtModel, String driveBehavior, ParameterMap learningParam)
	{
		return decoratee.createDecisionMaker(behaviors, thoughtModel, driveBehavior, learningParam);
	}
}
