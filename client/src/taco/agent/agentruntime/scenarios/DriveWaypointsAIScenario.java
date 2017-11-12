package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;

import static taco.agent.model.worldmodel.DriveInstruction.*;

public class DriveWaypointsAIScenario extends ScenarioBase
{
	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(LEFT)
				.add(STRAIGHT)
				.add(LEFT)
				.add(RIGHT)
				.add(RIGHT)
				.add(STRAIGHT)
				.add(RIGHT)
				.add(STRAIGHT)
				.add(LEFT)
				.add(DriveInstruction.CROSS_PARKING, 2, 0)
				.build();
	}
}
