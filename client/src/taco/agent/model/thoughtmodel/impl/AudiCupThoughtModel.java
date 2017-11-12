package taco.agent.model.thoughtmodel.impl;

import hso.autonomy.agent.communication.action.IAction;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.thoughtmodel.impl.ThoughtModel;
import hso.autonomy.util.logging.DrawingMap;
import hso.autonomy.util.logging.PropertyMap;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.model.agentmeta.impl.CarMetaModel;
import taco.agent.model.agentmeta.impl.ServoDriveConfiguration;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.thoughtmodel.IAudiCupThoughtModel;
import taco.agent.model.worldmodel.IAudiCupWorldModel;
import taco.util.drive.DriveGeometry;

public class AudiCupThoughtModel extends ThoughtModel implements IAudiCupThoughtModel
{
	private final ParkingSpaceDetection parkingSpaceDetection;

	private transient PropertyMap properties;

	private transient DrawingMap drawings;

	private final DriveGeometry driveGeometry;

	private final DriveWay driveWay;

	private final MovingObstacleDetection movingObstacleDetection;

	private final PedestrianDetection pedestrianDetection;

	public AudiCupThoughtModel(IAgentModel agentModel, IAudiCupWorldModel worldModel)
	{
		super(agentModel, worldModel);
		properties = new PropertyMap();
		drawings = new DrawingMap();
		parkingSpaceDetection = new ParkingSpaceDetection(this);

		CarMetaModel carMetaModel = getAgentModel().getCarMetaModel();
		double axleSpacing = carMetaModel.getAxleSpacing();
		ServoDriveConfiguration servo = carMetaModel.getServoDriveConfigs()[0];
		driveGeometry = new DriveGeometry(axleSpacing, servo.getMin(), servo.getMax());
		driveWay = new DriveWay(this);
		movingObstacleDetection = new MovingObstacleDetection();
		pedestrianDetection = new PedestrianDetection();
	}

	@Override
	public boolean update(IPerception perception)
	{
		if (properties != null) {
			properties.update();
		}

		boolean result = super.update(perception);

		parkingSpaceDetection.update();
		movingObstacleDetection.update(this);

		if (((IAudiCupPerception) perception).getVisionPerceptor() != null) {
			// only update if we have a camera perception
			pedestrianDetection.update(this);
		}

		return result;
	}

	@Override
	public void mapStateToAction(IAction action, boolean controlled)
	{
		getAgentModel().reflectTargetStateToAction(action);
	}

	@Override
	public IAudiCupWorldModel getWorldModel()
	{
		return (IAudiCupWorldModel) super.getWorldModel();
	}

	@Override
	public IAudiCupAgentModel getAgentModel()
	{
		return (IAudiCupAgentModel) super.getAgentModel();
	}

	@Override
	public boolean isInitialized()
	{
		return getWorldModel().isInitialized() && getAgentModel().isInitialized();
	}

	@Override
	public boolean isObstacleInPath()
	{
		double targetSpeed = getAgentModel().getMotor().getTargetSpeed();
		return (targetSpeed > 0 && isObstacleAhead()) || (targetSpeed < 0 && isObstacleBehind());
	}

	@Override
	public boolean isObstacleAhead()
	{
		return driveWay.isObstacleAhead();
	}

	/**
	 * Returns the smallest distance of an obstacle in us that is in our current drive way
	 * @param distance the distance to the object
	 */
	@Override
	public double getObstacleAheadDistance(double distance)
	{
		return driveWay.getObstacleAheadDistance(distance);
	}

	@Override
	public boolean isMovingObstacleAhead()
	{
		return movingObstacleDetection.isValid();
	}

	@Override
	public float followingObstacleSince()
	{
		if (!movingObstacleDetection.isValid()) {
			return -1f;
		}
		return movingObstacleDetection.getValidityTime(getWorldModel().getGlobalTime());
	}

	@Override
	public boolean isObstacleBehind()
	{
		return driveWay.isObstacleBehind();
	}

	@Override
	public boolean isPedestrianAhead()
	{
		return pedestrianDetection.isValid();
	}

	@Override
	public void log(String name, Object value)
	{
		if (properties != null) {
			properties.log(name, value);
		}
	}

	@Override
	public PropertyMap getProperties()
	{
		return properties;
	}

	@Override
	public DrawingMap getDrawings()
	{
		return drawings;
	}

	@Override
	public DriveGeometry getDriveGeometry()
	{
		return driveGeometry;
	}

	@Override
	public void updateAfterPerform()
	{
		driveWay.updateAfterPerform();
	}

	@Override
	public DriveWay getDriveWay()
	{
		return driveWay;
	}
}
