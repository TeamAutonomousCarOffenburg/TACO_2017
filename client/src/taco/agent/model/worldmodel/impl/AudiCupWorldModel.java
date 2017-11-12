package taco.agent.model.worldmodel.impl;

import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.worldmodel.IVisibleObject;
import hso.autonomy.agent.model.worldmodel.impl.WorldModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizer;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.Area2D;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.geometry.VectorUtils;
import taco.agent.agentruntime.scenarios.IScenario;
import taco.agent.communication.action.CarState;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.communication.perception.IEnvironmentConfigPerceptor;
import taco.agent.communication.perception.IJuryPerceptor;
import taco.agent.communication.perception.IManeuverListPerceptor;
import taco.agent.communication.perception.ISignPerceptor;
import taco.agent.communication.perception.IVisionPerceptor;
import taco.agent.communication.perception.JuryAction;
import taco.agent.communication.perception.PerceptorName;
import taco.agent.communication.perception.RecognizedObject;
import taco.agent.communication.perception.RecognizedObjectType;
import taco.agent.communication.perception.impl.LaneMiddlePerceptor;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.IDriveStatus;
import taco.agent.model.agentmodel.ITachometer;
import taco.agent.model.agentmodel.impl.enums.TachometerPosition;
import taco.agent.model.worldmodel.IAudiCupWorldModel;
import taco.agent.model.worldmodel.ILaneMiddleSensor;
import taco.agent.model.worldmodel.IThisCar;
import taco.agent.model.worldmodel.driveinstruction.DriveInstructionManager;
import taco.agent.model.worldmodel.driveinstruction.JuryActionManager;
import taco.agent.model.worldmodel.lanedetection.LaneMiddleSensor;
import taco.agent.model.worldmodel.odometry.GyroOdometry;
import taco.agent.model.worldmodel.signdetection.RoadSign;
import taco.agent.model.worldmodel.signdetection.RoadSignUtils;
import taco.agent.model.worldmodel.street.Direction;
import taco.agent.model.worldmodel.street.RuntimeSegment;
import taco.agent.model.worldmodel.street.Segment;
import taco.agent.model.worldmodel.street.SegmentLink;
import taco.agent.model.worldmodel.street.SegmentType;
import taco.agent.model.worldmodel.street.StreetMap;

public class AudiCupWorldModel extends WorldModel implements IAudiCupWorldModel
{
	private StreetMap map;

	private DriveInstructionManager driveInstructionManager;

	private ILaneMiddleSensor laneMiddleSensor;

	private JuryActionManager juryManager;

	private EnvironmentManager environmentManager;

	private IThisCar thisCar;

	private RuntimeSegment currentSegment;

	private GyroOdometry gyroOdometry;

	private boolean initialized = false;

	private boolean hasJuryActionChanged = false;

	public AudiCupWorldModel(IAgentModel agentModel, ILocalizer localizer, IScenario scenario)
	{
		super(agentModel, localizer);

		map = scenario.getStreetMap();
		driveInstructionManager = scenario.createDriveInstructionManager();
		laneMiddleSensor = new LaneMiddleSensor();
		juryManager = new JuryActionManager();
		environmentManager = new EnvironmentManager(
				getAgentModel().getParkingSpaceActuator(), getAgentModel().getRoadSignActuator(), map);

		IPose2D startPose = scenario.getStartPose();
		thisCar = new ThisCar(startPose);
		gyroOdometry = new GyroOdometry(getAgentModel().getCarMetaModel().getFrontAxle().getWheelDiameter(), startPose);
		resetCurrentSegment(startPose);
	}

	public void resetCurrentSegment(IPose2D startPose)
	{
		Segment startSegment = map.getCurrentStartSegment(driveInstructionManager.getCurrentSectorIndex());
		currentSegment = new RuntimeSegment(startSegment, startPose.getAngle());
	}

	@Override
	public boolean update(IPerception perception)
	{
		hasJuryActionChanged = false;

		super.update(perception);

		IAudiCupPerception audiCupPerception = (IAudiCupPerception) perception;

		// Process maneuver list instructions
		IManeuverListPerceptor maneuverListPerceptor = audiCupPerception.getManeuverListPerceptor();
		if (maneuverListPerceptor != null) {
			driveInstructionManager.setDriveInstructions(maneuverListPerceptor.getManeuver());
			updateDrivePath();
		}

		processVision(audiCupPerception);

		// loads the information from the roadsign.xml into the environment manager and into the map
		processEnvironmentConfiguration(audiCupPerception);

		// update roadsigns from adtf sign detection
		processSignDetection(audiCupPerception);

		if (!initialized) {
			// we wait for the agent model to get initialized
			if (getAgentModel().isInitialized()) {
				initializeOdometry();
				initialized = true;
			}
			return initialized;
		}

		// get information from middle of lane detection
		processLaneMiddle(audiCupPerception);

		// check where we are on the street
		localize();

		// Process jury module instruction. After localization because we might reset car position
		processJuryInstruction(audiCupPerception);

		// update current segment information
		progressSegment();

		getAgentModel().getCarPositionActuator().setCarPose(thisCar.getPose());

		return true;
	}

	private void processVision(IAudiCupPerception audiCupPerception)
	{
		IVisionPerceptor visionPerceptor = audiCupPerception.getVisionPerceptor();
		if (visionPerceptor != null && getAgentModel().getBaslerCamera() != null) {
			List<RecognizedObject> recognizedObjects = visionPerceptor.getRecognizedObjects();
			if (recognizedObjects.isEmpty()) {
				return;
			}

			obstacles.clear();

			for (RecognizedObject recognizedObject : recognizedObjects) {
				Area2D.Float globalArea;
				if (recognizedObject.isInCarCoordinates()) {
					Area2D.Int area = recognizedObject.getArea();
					globalArea = new Area2D.Float(area.getMinX() / 100.0, area.getMaxX() / 100.0,
							area.getMinY() / 100.0, area.getMaxY() / 100.0);
				} else {
					Area2D.Float area = getAgentModel().getBaslerCamera().pixelToCar(recognizedObject.getArea());
					if (area == null) {
						continue;
					}

					Vector2D topLeft = thisCar.getPose().applyTo(area.getTopLeft());
					Vector2D bottomRight = thisCar.getPose().applyTo(area.getBottomRight());
					globalArea =
							new Area2D.Float(topLeft.getX(), bottomRight.getX(), topLeft.getY(), bottomRight.getY());
				}
				obstacles.add(new Obstacle(globalTime, recognizedObject.getType(), globalArea));
			}

			obstacles.stream()
					.map(visibleObject -> (Obstacle) visibleObject)
					.filter(obstacle -> obstacle.getType().isObjectForBackend())
					.forEach(obstacle
							-> getAgentModel().getObstaclePositionActuator().setObstacle(
									VectorUtils.to2D(obstacle.getPosition())));
		} else {
			obstacles = getRecognizedObjects()
								.stream()
								.filter(obstacle -> obstacle.isValid(globalTime))
								.collect(Collectors.toList());
		}
	}

	/**
	 * Initializes the gyro sensor. During this time the car has to stand still
	 */
	protected void initializeOdometry()
	{
		gyroOdometry.init(getAgentModel().getTachometer(TachometerPosition.LEFT).getTicks(),
				getAgentModel().getTachometer(TachometerPosition.RIGHT).getTicks());
	}

	private void processEnvironmentConfiguration(IAudiCupPerception perception)
	{
		IEnvironmentConfigPerceptor environmentConfigPerceptor = perception.getEnvironmentPerceptor();
		if (environmentConfigPerceptor != null) {
			// TODO: the server should stop sending this when we have received it, cause it's just for initializing
			environmentManager.update(environmentConfigPerceptor, map);
		}
	}

	private void processSignDetection(IAudiCupPerception perception)
	{
		Pose3D cameraPose = getAgentModel().getSignDetectionCamera().getPose();
		IPose2D globalCameraPose = thisCar.getPose().applyTo(
				new Pose2D(cameraPose.position, Angle.rad((cameraPose).getOrientation().getAngle())));

		RoadSign roadSign = environmentManager.updateVisisbleRoadSigns(globalCameraPose, getGlobalTime());
		if (roadSign != null) {
			RoadSignUtils.removeSignFromMap(map, roadSign);
		}

		ISignPerceptor signPerceptor = perception.getSignPerceptor();
		if (signPerceptor != null) {
			for (RoadSign sign : signPerceptor.getSigns()) {
				// filter zero-pos
				if (sign.getPose().getPosition().distance(Vector3D.ZERO) < 0.1) {
					continue;
				}

				sign.setPose(globalCameraPose.applyTo(sign.getPose()));
				// don't believe signs that are not in the plausible visible area
				if (!RoadSignUtils.isInVisibleArea(globalCameraPose, sign)) {
					return;
				}

				if (RoadSignUtils.loadReceivedRoadSignIntoMap(
							map, environmentManager.getKnownRoadSigns(), sign, getGlobalTime())) {
					sign.update(true, true, globalTime);
					environmentManager.updateRoadSign(sign);
				}
			}
		}
	}

	private void processLaneMiddle(IAudiCupPerception perception)
	{
		LaneMiddlePerceptor laneResult = perception.getLaneMiddlePerceptor(PerceptorName.LANE_MIDDLE);
		if (laneResult != null) {
			laneMiddleSensor.update(laneResult, getGlobalTime(), thisCar.getPose(),
					getRecognizedObjects(RecognizedObjectType.MIDDLE_LANE), getMap());
		}
	}

	/**
	 * Updates the cars current pose based on odometry and visual information
	 */
	protected void localize()
	{
		// update through odometry
		ITachometer wheelSpeedLeft = getAgentModel().getTachometer(TachometerPosition.LEFT);
		ITachometer wheelSpeedRight = getAgentModel().getTachometer(TachometerPosition.RIGHT);

		IPose2D odometryPose = gyroOdometry.update(wheelSpeedLeft.getTicks(), wheelSpeedLeft.getDirection(),
				wheelSpeedRight.getTicks(), wheelSpeedRight.getDirection(),
				getAgentModel().getImuSensor().getHorizontalAngle());

		// currently we only believe in odometry
		thisCar.setPose(odometryPose);

		// do repositioning in driving direction
		IPose2D newPose =
				laneMiddleSensor.calculateSagittalRepositioning(currentSegment, getGlobalTime(), thisCar.getPose(),
						getRecognizedObjects(RecognizedObjectType.STOP_LINE_AHEAD), getAgentModel().getBaslerCamera());
		if (newPose != null) {
			reposition(newPose);
			// no further repositioning if we did this
			return;
		}

		// do lateral and angle repositioning based on lane middle detection
		newPose = laneMiddleSensor.calculateLateralRepositioning(currentSegment, getGlobalTime(), thisCar.getPose());
		if (newPose != null) {
			reposition(newPose);
		}
	}

	public void reposition(IPose2D newPose)
	{
		thisCar.setPose(newPose);
		gyroOdometry.setPose(newPose);
	}

	private void processJuryInstruction(IAudiCupPerception perception)
	{
		IJuryPerceptor juryPerceptor = perception.getJuryPerceptor();
		if (juryPerceptor != null && juryManager.update(juryPerceptor)) {
			IDriveStatus driveStatus = getAgentModel().getDriveStatus();
			if (getJuryAction() == JuryAction.GET_READY) {
				driveStatus.setStatus(CarState.READY);
			}

			if (getJuryAction() == JuryAction.START) {
				driveStatus.setStatus(CarState.RUNNING);
			}

			// If we get STOP from the jury, we should turn into a state where we can be reactivated
			if (getJuryAction() == JuryAction.STOP) {
				driveStatus.setStatus(CarState.STARTUP);
			}

			// update maneuver-ID in driveStatus to response our current state with the current maneuver
			driveStatus.setManeuverId(juryManager.getManeuverId());
			driveInstructionManager.setStartInstructionIndex(juryManager.getManeuverId());
			updateDrivePath();

			int sector = driveInstructionManager.getSectorIndex(juryManager.getManeuverId());
			IPose2D startPose = map.getCurrentStartPose(sector);
			reposition(startPose);
			resetCurrentSegment(startPose);

			hasJuryActionChanged = true;
		}
	}

	/**
	 * Updates the current segment information based on the pose of the car.
	 */
	void progressSegment()
	{
		Segment whereWeAreNow = map.getSegmentContaining(thisCar.getPose().getPosition());
		if (whereWeAreNow == null) {
			// we are not in a segment of our map
			return;
		}

		if (whereWeAreNow.getID() != currentSegment.getID()) {
			currentSegment.switchToSegment(whereWeAreNow, thisCar.getPose().getAngle());
			// do lateral repositioning based on lane middle detection
			IPose2D newPose =
					laneMiddleSensor.calculateLateralRepositioningOnSegmentChange(currentSegment, thisCar.getPose());
			if (newPose != null) {
				reposition(newPose);
			}
		}
	}

	@Override
	public IThisCar getThisCar()
	{
		return thisCar;
	}

	@Override
	protected IAudiCupAgentModel getAgentModel()
	{
		return (IAudiCupAgentModel) super.getAgentModel();
	}

	@Override
	public ILaneMiddleSensor getLaneMiddleSensor()
	{
		return laneMiddleSensor;
	}

	@Override
	public StreetMap getMap()
	{
		return map;
	}

	@Override
	public RuntimeSegment getCurrentSegment()
	{
		return currentSegment;
	}

	@Override
	public GyroOdometry getGyroOdometry()
	{
		return gyroOdometry;
	}

	@Override
	public JuryAction getJuryAction()
	{
		return juryManager.getAction();
	}

	@Override
	public JuryActionManager getJuryActionManager()
	{
		return juryManager;
	}

	@Override
	public boolean isInitialized()
	{
		return initialized;
	}

	@Override
	public DriveInstructionManager getDriveInstructionManager()
	{
		return driveInstructionManager;
	}

	public void setDriveInstructionManager(DriveInstructionManager driveInstructionManager)
	{
		this.driveInstructionManager = driveInstructionManager;
	}

	@Override
	public void updateDrivePath()
	{
		getThisCar().setPath(WaypointExtractor.extractPath(driveInstructionManager, currentSegment));
	}

	@Override
	public List<Obstacle> getRecognizedObjects()
	{
		return obstacles.stream()
				.filter(o -> o instanceof Obstacle)
				.map(o -> (Obstacle) o)
				.collect(Collectors.toList());
	}

	@Override
	public List<Obstacle> getRecognizedObjects(RecognizedObjectType type)
	{
		return getRecognizedObjects().stream().filter(o -> o.getType() == type).collect(Collectors.toList());
	}

	@Override
	public EnvironmentManager getEnvironmentManager()
	{
		return environmentManager;
	}

	@Override
	public boolean hasJuryActionChanged()
	{
		return hasJuryActionChanged;
	}

	public void setObstacles(List<IVisibleObject> obstacles)
	{
		this.obstacles = obstacles;
	}

	@Override
	public boolean isStraightStreet(int elements)
	{
		Segment segment = getCurrentSegment().getSegment();

		int i = 0;
		while (i <= elements) {
			if (segment == null || segment.getType() != SegmentType.STRAIGHT) {
				return false;
			}

			segment = getNextSegment(segment);
			i++;
		}

		return true;
	}

	@Override
	public boolean closeToCrosswalk()
	{
		Segment currentSegment = getCurrentSegment().getSegment();
		Segment nextSegment = getCurrentSegment().getIntendedOption().getSegmentAfter();

		if (nextSegment == null) {
			return false;
		}

		//		if (currentSegment.getType() == SegmentType.STRAIGHT_WITH_CROSSWALK ||
		//				nextSegment.getType() == SegmentType.STRAIGHT_WITH_CROSSWALK) {
		//			return true;
		//		}

		if (nextSegment.getType() == SegmentType.STRAIGHT_WITH_CROSSWALK) {
			return true;
		}

		return false;
	}

	@Override
	public boolean closeToCrossing()
	{
		Segment nextSegment = getCurrentSegment().getIntendedOption().getSegmentAfter();

		if (nextSegment == null) {
			return false;
		}

		if (isCrossing(nextSegment)) {
			return true;
		}

		SegmentLink nextOptions = nextSegment.getSameLaneOutOption(getCurrentDirection());

		if (nextOptions != null && nextOptions.hasStopLine()) {
			return true;
		}

		return false;
	}

	@Override
	public boolean closeToCurve()
	{
		Segment currentSegment = getCurrentSegment().getSegment();
		Segment nextSegment = getCurrentSegment().getIntendedOption().getSegmentAfter();
		Segment afterNextSegment = getNextSegment(nextSegment);

		if (nextSegment == null || afterNextSegment == null) {
			return false;
		}

		if (isCurve(currentSegment) || isCurve(nextSegment) || isCurve(afterNextSegment)) {
			return true;
		}

		return false;
	}

	private Segment getNextSegment(Segment current)
	{
		RuntimeSegment runtimeSegment = new RuntimeSegment(current, getThisCar().getPose().getAngle());
		return runtimeSegment.getIntendedOption().getSegmentAfter();
	}

	private boolean isCrossing(Segment current)
	{
		return (current.getType() == SegmentType.X_CROSSING || current.getType() == SegmentType.T_CROSSING);
	}

	private boolean isCurve(Segment current)
	{
		return (current.getType() == SegmentType.CURVE_SMALL || current.getType() == SegmentType.CURVE_BIG ||
				current.getType() == SegmentType.S_CURVE_BOTTOM || current.getType() == SegmentType.S_CURVE_TOP);
	}

	private Direction getCurrentDirection()
	{
		Angle angle = getThisCar().getPose().getAngle();
		return Direction.getDirection(angle);
	}
}
