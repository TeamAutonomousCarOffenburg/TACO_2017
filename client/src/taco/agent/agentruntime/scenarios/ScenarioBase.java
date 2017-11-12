package taco.agent.agentruntime.scenarios;

import static taco.agent.model.worldmodel.DriveInstruction.LEFT;
import static taco.agent.model.worldmodel.DriveInstruction.RIGHT;
import static taco.agent.model.worldmodel.DriveInstruction.STRAIGHT;

import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;
import taco.agent.model.worldmodel.street.maps.HSMap;

public abstract class ScenarioBase implements IScenario
{
	private String name;

	private StreetMap map;

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public void setName(String name)
	{
		this.name = name;
	}

	protected StreetMap createStreetMap()
	{
		return HSMap.create();
	}

	@Override
	public final StreetMap getStreetMap()
	{
		if (map == null) {
			map = createStreetMap();
		}
		return map;
	}

	@Override
	public int getStartSector()
	{
		return 0;
	}

	@Override
	public String getDriveBehavior()
	{
		return DRIVE_WAYPOINTS;
	}

	@Override
	public void configureDriveBehavior(IBehavior behavior)
	{
	}

	@Override
	public final IPose2D getStartPose()
	{
		return getStreetMap().getCurrentStartPose(getStartSector());
	}

	@Override
	public DriveInstructionManager createDriveInstructionManager()
	{
		return new DriveInstructionManager.Builder()
				.add(LEFT, 0)
				.add(DriveInstruction.CROSS_PARKING, 2, 0)
				.add(DriveInstruction.PULL_OUT_LEFT, 0)
				.add(STRAIGHT, 0)
				.add(RIGHT, 0)
				.build();
	}
}
