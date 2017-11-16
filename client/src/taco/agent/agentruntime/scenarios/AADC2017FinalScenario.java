package taco.agent.agentruntime.scenarios;

import static taco.agent.model.worldmodel.DriveInstruction.LEFT;
import static taco.agent.model.worldmodel.DriveInstruction.RIGHT;
import static taco.agent.model.worldmodel.DriveInstruction.STRAIGHT;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.agent.model.worldmodel.street.maps.AADC2017FinalMap;

public class AADC2017FinalScenario extends ScenarioBase
{
	@Override
	public StreetMap createStreetMap()
	{
		return AADC2017FinalMap.create();
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(DriveInstruction.PULL_OUT_LEFT, 0)
				.add(RIGHT, 0)
				.add(STRAIGHT, 1)
				.add(LEFT, 1)
				.add(LEFT, 1)
				.add(LEFT, 1)
				.add(STRAIGHT, 1)
				.add(STRAIGHT, 1)
				.add(LEFT, 2)
				.add(STRAIGHT, 2)
				.add(LEFT, 2)
				.add(LEFT, 2)
				.add(STRAIGHT, 2)
				.add(RIGHT, 3)
				.add(RIGHT, 4)
				.add(STRAIGHT, 5)
				.add(LEFT, 5)
				.add(STRAIGHT, 5)
				.add(LEFT, 5)
				.add(LEFT, 6)
				.add(STRAIGHT, 6)
				.add(STRAIGHT, 6)
				.add(STRAIGHT, 6)
				.add(RIGHT, 6)
				.add(STRAIGHT, 6)
				.add(DriveInstruction.CROSS_PARKING, 6, 7)
				.build();
	}
}
