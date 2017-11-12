package taco.agent.agentruntime.scenarios;

public class FollowLaneLearningScenario extends ScenarioBase
{
	private int startSector;

	public FollowLaneLearningScenario(int startSector)
	{
		this.startSector = startSector;
	}

	@Override
	public int getStartSector()
	{
		return startSector;
	}

	@Override
	public String getDriveBehavior()
	{
		return FOLLOW_RIGHT_LANE_LEARNING;
	}
}
