package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;

public class FollowLaneScenario extends ScenarioBase
{
	private int startSector;

	public FollowLaneScenario(int startSector)
	{
		this.startSector = startSector;
	}

	@Override
	public int getStartSector()
	{
		return startSector;
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder().add(DriveInstruction.STRAIGHT_FOREVER, 0).build();
	}
}
