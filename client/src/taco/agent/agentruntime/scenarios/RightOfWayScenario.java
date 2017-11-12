package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import static taco.agent.model.worldmodel.DriveInstruction.LEFT;
import static taco.agent.model.worldmodel.DriveInstruction.STRAIGHT;

public class RightOfWayScenario extends ScenarioBase
{
	@Override
	public int getStartSector()
	{
		return 2;
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder().add(LEFT).add(LEFT).add(STRAIGHT).add(STRAIGHT).build();
	}
}
