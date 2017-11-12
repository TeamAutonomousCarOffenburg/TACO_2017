package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;

public class PullOutScenario extends ScenarioBase
{
	private final DriveInstruction pullOutInstruction;

	public PullOutScenario(DriveInstruction pullOutInstruction)
	{
		this.pullOutInstruction = pullOutInstruction;
	}

	@Override
	public int getStartSector()
	{
		return 4;
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(pullOutInstruction)
				.add(DriveInstruction.STRAIGHT_FOREVER)
				.build();
	}
}
