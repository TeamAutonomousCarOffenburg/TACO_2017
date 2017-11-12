package taco.agent.model.worldmodel;

import hso.autonomy.agent.model.worldmodel.IWorldModel;
import taco.agent.communication.perception.JuryAction;
import taco.agent.communication.perception.RecognizedObjectType;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.driveinstruction.JuryActionManager;
import taco.agent.model.worldmodel.impl.EnvironmentManager;
import taco.agent.model.worldmodel.impl.Obstacle;
import taco.agent.model.worldmodel.odometry.GyroOdometry;
import taco.agent.model.worldmodel.street.RuntimeSegment;
import taco.agent.model.worldmodel.street.StreetMap;

import java.util.List;

public interface IAudiCupWorldModel extends IWorldModel {
	IThisCar getThisCar();

	ILaneMiddleSensor getLaneMiddleSensor();

	StreetMap getMap();

	RuntimeSegment getCurrentSegment();

	GyroOdometry getGyroOdometry();

	JuryAction getJuryAction();

	JuryActionManager getJuryActionManager();

	/**
	 * @return true if this model has run through the initial setup process
	 */
	boolean isInitialized();

	DriveInstructionManager getDriveInstructionManager();

	void updateDrivePath();

	List<Obstacle> getRecognizedObjects();

	List<Obstacle> getRecognizedObjects(RecognizedObjectType type);

	EnvironmentManager getEnvironmentManager();

	boolean hasJuryActionChanged();

	boolean isStraightStreet(int elements);

	boolean closeToCrosswalk();

	boolean closeToCrossing();

	boolean closeToCurve();
}
