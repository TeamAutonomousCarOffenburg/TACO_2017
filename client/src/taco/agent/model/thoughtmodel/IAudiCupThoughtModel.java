package taco.agent.model.thoughtmodel;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.logging.DrawingMap;
import hso.autonomy.util.logging.PropertyMap;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.impl.DriveWay;
import taco.agent.model.worldmodel.IAudiCupWorldModel;
import taco.util.drive.DriveGeometry;

public interface IAudiCupThoughtModel extends IThoughtModel {
	@Override
	IAudiCupWorldModel getWorldModel();

	@Override
	IAudiCupAgentModel getAgentModel();

	/**
	 * @return true if this model has run through the initial setup process
	 */
	boolean isInitialized();

	boolean isObstacleInPath();

	boolean isObstacleAhead();

	boolean isObstacleBehind();

	boolean isMovingObstacleAhead();

	float followingObstacleSince();

	double getObstacleAheadDistance(double distance);

	boolean isPedestrianAhead();

	void log(String name, Object value);

	PropertyMap getProperties();

	DrawingMap getDrawings();

	DriveGeometry getDriveGeometry();

	void updateAfterPerform();

	DriveWay getDriveWay();
}
