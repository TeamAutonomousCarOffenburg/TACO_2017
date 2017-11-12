package taco.agent.agentruntime.scenarios;

import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;

import taco.agent.model.worldmodel.street.maps.AADC2017QualiMap;

import static taco.agent.model.worldmodel.DriveInstruction.*;

public class OvertakeScenario extends ScenarioBase
{
	public StreetMap createStreetMap()
	{
		return AADC2017QualiMap.create();
	}

	@Override
	public int getStartSector()
	{
		return 3;
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder().add(STRAIGHT_FOREVER, 3).build();
	}
}