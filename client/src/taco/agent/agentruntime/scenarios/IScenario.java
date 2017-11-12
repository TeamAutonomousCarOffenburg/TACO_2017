package taco.agent.agentruntime.scenarios;

import hso.autonomy.agent.decision.behavior.IBehavior;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.street.StreetMap;

public interface IScenario extends IBehaviorConstants {
	void setName(String name);

	String getName();

	StreetMap getStreetMap();

	int getStartSector();

	String getDriveBehavior();

	void configureDriveBehavior(IBehavior behavior);

	IPose2D getStartPose();

	DriveInstructionManager createDriveInstructionManager();
}
