package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;

import static taco.agent.model.worldmodel.DriveInstruction.LEFT;
import static taco.agent.model.worldmodel.DriveInstruction.RIGHT;
import static taco.agent.model.worldmodel.DriveInstruction.STRAIGHT;

public class DriveWaypointsLongScenario extends ScenarioBase
{
	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(LEFT, 0)
				.add(STRAIGHT, 0)
				.add(LEFT, 0)
				.add(STRAIGHT, 1)
				.add(RIGHT, 1)
				.add(STRAIGHT, 1)
				.build();
	}
}
