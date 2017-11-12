package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.agent.model.worldmodel.street.maps.AADC2015Map;

import static taco.agent.model.worldmodel.DriveInstruction.*;

public class AADC2015Scenario extends ScenarioBase
{
	@Override
	public StreetMap createStreetMap()
	{
		return AADC2015Map.create();
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(DriveInstruction.PULL_OUT_LEFT, 0)
				.add(RIGHT, 0)
				.add(LEFT, 0)
				.add(LEFT, 0)
				.add(STRAIGHT, 1)
				.add(LEFT, 1)
				.add(LEFT, 1)
				.add(STRAIGHT, 1)
				.add(STRAIGHT, 1)
				.build();
	}
}
