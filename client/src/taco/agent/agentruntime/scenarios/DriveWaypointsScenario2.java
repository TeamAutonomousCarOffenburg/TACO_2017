package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;

public class DriveWaypointsScenario2 extends ScenarioBase
{
	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(DriveInstruction.RIGHT)
				.add(DriveInstruction.RIGHT)
				.add(DriveInstruction.LEFT)
				.add(DriveInstruction.CROSS_PARKING, 3)
				.add(DriveInstruction.PULL_OUT_RIGHT)
				.add(DriveInstruction.STRAIGHT)
				.add(DriveInstruction.LEFT)
				.build();
	}
}
