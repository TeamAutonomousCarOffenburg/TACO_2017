package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.agent.model.worldmodel.street.maps.HSMapStraight;

import static taco.agent.model.worldmodel.DriveInstruction.STRAIGHT_FOREVER;

public class OvertakeStraightScenario extends ScenarioBase
{
	public StreetMap createStreetMap()
	{
		return HSMapStraight.create();
	}

	@Override
	public int getStartSector()
	{
		return 0;
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder().add(STRAIGHT_FOREVER, 0).build();
	}
}