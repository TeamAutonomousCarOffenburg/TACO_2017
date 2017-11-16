package taco.agent.agentruntime;

import hso.autonomy.agent.communication.action.IActionPerformer;
import hso.autonomy.agent.communication.channel.IChannelManager;
import hso.autonomy.agent.communication.channel.impl.ChannelManager;
import hso.autonomy.agent.communication.channel.impl.InputChannel;
import hso.autonomy.agent.communication.channel.impl.InputOutputChannel;
import hso.autonomy.agent.decision.behavior.BehaviorMap;
import hso.autonomy.agent.decision.behavior.basic.None;
import hso.autonomy.agent.decision.decisionmaker.IDecisionMaker;
import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.agentmodel.impl.ik.impl.JacobianTransposeAgentIKSolver;
import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.agent.model.worldmodel.localizer.impl.LocalizerFieldNormal;
import hso.autonomy.util.geometry.IPose2D;
import kdo.util.parameter.ParameterMap;
import taco.agent.agentruntime.scenarios.IScenario;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.action.impl.AudiCupAction;
import taco.agent.communication.channel.impl.AudiCupChannel;
import taco.agent.communication.channel.impl.VisionChannel;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.communication.perception.impl.AudiCupPerception;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.impl.CrossParking;
import taco.agent.decision.behavior.impl.DriveSlalom;
import taco.agent.decision.behavior.impl.DriveToPose;
import taco.agent.decision.behavior.impl.DriveWaypoints;
import taco.agent.decision.behavior.impl.EmergencyBrake;
import taco.agent.decision.behavior.impl.FollowLaneParameters;
import taco.agent.decision.behavior.impl.FollowLeftLane;
import taco.agent.decision.behavior.impl.FollowRightLane;
import taco.agent.decision.behavior.impl.FollowRightLaneLearning;
import taco.agent.decision.behavior.impl.FollowRightLaneSlow;
import taco.agent.decision.behavior.impl.GiveWay;
import taco.agent.decision.behavior.impl.OvertakeObstacle;
import taco.agent.decision.behavior.impl.ParametrizedDrive;
import taco.agent.decision.behavior.impl.ParametrizedDriveParameters;
import taco.agent.decision.behavior.impl.PullOutLeft;
import taco.agent.decision.behavior.impl.PullOutRight;
import taco.agent.decision.behavior.impl.Stop;
import taco.agent.decision.behavior.training.DriveCircle;
import taco.agent.decision.behavior.training.DriveStraight;
import taco.agent.decision.decisionmaker.impl.AudiCupDecisionMaker;
import taco.agent.model.agentmeta.impl.AudiCupAgentMetaModel;
import taco.agent.model.agentmeta.impl.CarMetaModelVersion;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.impl.AudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.thoughtmodel.impl.AudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;
import taco.agent.model.worldmodel.impl.AudiCupWorldModel;

public class ComponentFactory
{
	public IChannelManager createChannelManager(String server)
	{
		ChannelManager channelManager = new ChannelManager();
		// Channel to our C++ server
		InputOutputChannel serverChannel = new AudiCupChannel(channelManager, server);
		channelManager.addInputChannel(serverChannel, true);
		channelManager.addOutputChannel(serverChannel);
		// Channel to the vision server
		InputChannel visionChannel = new VisionChannel(channelManager, server);
		channelManager.addInputChannel(visionChannel, false);
		return channelManager;
	}

	public IAgentMetaModel createAgentMetaModel(String metaModelDirectory, CarMetaModelVersion version)
	{
		return new AudiCupAgentMetaModel(metaModelDirectory, version);
	}

	public IAudiCupAgentModel createAgentModel(IAgentMetaModel agentMetaModel)
	{
		return new AudiCupAgentModel(agentMetaModel, new JacobianTransposeAgentIKSolver());
	}

	public IAudiCupWorldModel createWorldModel(IAgentModel agentModel, IScenario scenario, IPose2D startPose)
	{
		return new AudiCupWorldModel(agentModel, new LocalizerFieldNormal(), scenario, startPose);
	}

	public IAudiCupThoughtModel createThoughtModel(IAgentModel agentModel, IAudiCupWorldModel worldModel)
	{
		return new AudiCupThoughtModel(agentModel, worldModel);
	}

	public IAudiCupPerception createPerception()
	{
		return new AudiCupPerception();
	}

	public IAudiCupAction createAction(IActionPerformer actionPerformer, IAgentMetaModel agentMetaModel)
	{
		AudiCupAction action = new AudiCupAction(actionPerformer);
		action.init(agentMetaModel.createEffectors());
		return action;
	}

	public BehaviorMap createBehaviors(IAudiCupThoughtModel thoughtModel, ParameterMap params)
	{
		BehaviorMap behaviors = new BehaviorMap();

		behaviors.put(new FollowRightLane(thoughtModel, params));
		behaviors.put(new FollowRightLaneLearning(thoughtModel));
		behaviors.put(new FollowRightLaneSlow(thoughtModel, params));
		behaviors.put(new FollowLeftLane(thoughtModel));
		behaviors.put(new EmergencyBrake(thoughtModel));
		behaviors.put(new None(thoughtModel));
		behaviors.put(new Stop(thoughtModel));
		behaviors.put(new DriveCircle(thoughtModel));
		behaviors.put(new DriveSlalom(thoughtModel, behaviors));
		behaviors.put(new DriveToPose(thoughtModel));
		behaviors.put(new PullOutRight(thoughtModel));
		behaviors.put(new PullOutLeft(thoughtModel));
		behaviors.put(new GiveWay(thoughtModel));
		behaviors.put(new DriveStraight(thoughtModel, behaviors));
		behaviors.put(new CrossParking(thoughtModel, behaviors));
		behaviors.put(new OvertakeObstacle(thoughtModel, behaviors));
		behaviors.put(new DriveWaypoints(thoughtModel, behaviors));
		behaviors.put(new ParametrizedDrive(thoughtModel, params));

		return behaviors;
	}

	/**
	 * Creates behavior or decision maker parameters. fromExtern parameters
	 * overwrite default parameters.
	 * @param fromExtern parametrization from an external source like learning
	 * @return a parameter map that has to contain all ParameterLists needed for
	 *         default behaviors
	 */
	public ParameterMap createParameters(ParameterMap fromExtern)
	{
		ParameterMap result = new ParameterMap();

		// specific parameters have higher priority than general and are put after
		// them in order to allow overwriting
		result.putAll(createSpecificParameters());

		// external parameters (e.g. from learning) have higher priority and are
		// therefore overwriting parameters defined here
		result.putAll(fromExtern);
		return result;
	}

	protected ParameterMap createSpecificParameters()
	{
		ParameterMap result = new ParameterMap();
		result.put(IBehaviorConstants.FOLLOW_RIGHT_LANE, new FollowLaneParameters());
		result.put(IBehaviorConstants.PARAMETRIZED_DRIVE, new ParametrizedDriveParameters());
		return result;
	}

	public IDecisionMaker createDecisionMaker(
			BehaviorMap behaviors, IThoughtModel thoughtModel, String driveBehavior, ParameterMap learningParam)
	{
		// TODO: pass learningParam to decision maker once we learn params of decision maker
		return new AudiCupDecisionMaker(behaviors, thoughtModel, driveBehavior);
	}
}
