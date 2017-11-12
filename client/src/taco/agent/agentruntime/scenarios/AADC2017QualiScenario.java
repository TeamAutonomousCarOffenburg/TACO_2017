package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.agent.model.worldmodel.street.maps.AADC2017QualiMap;

import static taco.agent.model.worldmodel.DriveInstruction.*;

public class AADC2017QualiScenario extends ScenarioBase
{
	@Override
	public StreetMap createStreetMap()
	{
		return AADC2017QualiMap.create();
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(LEFT, 0)
				.add(STRAIGHT, 0)
				.add(STRAIGHT, 0)
				.add(RIGHT, 0)
				.add(RIGHT, 0)
				.add(RIGHT, 0)
				.add(STRAIGHT, 0)
				.add(LEFT, 0)
				.add(STRAIGHT, 1)
				.add(STRAIGHT, 1)
				.add(STRAIGHT, 1)
				.add(STRAIGHT, 2)
				.add(DriveInstruction.CROSS_PARKING, 4, 3)
				.build();
	}
}
