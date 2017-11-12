/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.worldmodel.impl;

import hso.autonomy.agent.model.worldmodel.IVisibleObject;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.communication.perception.ITimerPerceptor;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.agentmodel.IBodyPart;
import hso.autonomy.agent.model.worldmodel.IFieldLine;
import hso.autonomy.agent.model.worldmodel.ILandmark;
import hso.autonomy.agent.model.worldmodel.IWorldModel;
import hso.autonomy.agent.model.worldmodel.localizer.IEnvironmentModel;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizationLine;
import hso.autonomy.agent.model.worldmodel.localizer.ILocalizer;
import hso.autonomy.agent.model.worldmodel.localizer.IReferenceLine;
import hso.autonomy.agent.model.worldmodel.localizer.IReferencePoint;
import hso.autonomy.util.geometry.Geometry;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.geometry.positionFilter.IPositionFilter;
import hso.autonomy.util.geometry.positionFilter.PositionKalmanFilter;
import hso.autonomy.util.geometry.positionFilter.PositionResetDecorator;
import hso.autonomy.util.observer.IObserver;
import hso.autonomy.util.observer.IPublishSubscribe;
import hso.autonomy.util.observer.Subject;

/**
 * Container class for all visible objects on the field, for the time and game
 * state
 */
public class WorldModel implements IWorldModel, IEnvironmentModel, Serializable
{
	/** observers of world model */
	protected final transient IPublishSubscribe<IWorldModel> observer;

	/** the localizer that is able to calculate the absolute position */
	protected final transient ILocalizer localizer;

	/** the landmark visual objects (flags and goal posts) */
	// private final Map<String, ILandmark> landmarks;
	protected final Map<String, ILandmark> landmarks;

	/** the reference points used for localization (flags and goal posts) */
	protected final transient List<IReferencePoint> referencePoints;

	/** the field line visual objects */
	protected final Map<String, IFieldLine> fieldLines;

	/** the reference lines (field lines) used for localization */
	protected final transient List<IReferenceLine> referenceLines;

	/** list of all obstacles to avoid */
	protected List<IVisibleObject> obstacles;

	/** the time on the global clock (in cycles). Not the game time */
	protected float globalTime;

	/** reference to the agent model needed for gyro localization */
	private IAgentModel agentModel;

	protected transient IPositionFilter posFilter;

	/**
	 * The transformation from the camera body system to the root body system of
	 * the current cycle.
	 */
	private transient Pose3D cameraToRootVision;

	/**
	 * Constructor
	 *
	 * @param agentModel Reference to the agent model object
	 * @param localizer the module that calculates the agent's global position
	 */
	public WorldModel(IAgentModel agentModel, ILocalizer localizer)
	{
		this.agentModel = agentModel;
		this.localizer = localizer;

		// create objects
		landmarks = new HashMap<>();
		fieldLines = new HashMap<>();
		referencePoints = new ArrayList<>();
		referenceLines = new ArrayList<>();
		obstacles = new ArrayList<>(0);
		observer = new Subject<>();
		// posFilter = new NoFilter();
		// posFilter = new PositionFilter();
		posFilter = new PositionResetDecorator(new PositionKalmanFilter(), 1.0);
	}

	/**
	 * Called once perception is finished parsing a new incoming message
	 * @param perception the object containing the result from parsing
	 */
	@Override
	public boolean update(IPerception perception)
	{
		// update global time
		ITimerPerceptor timerPerceptor = perception.getTime();
		if (timerPerceptor != null) {
			globalTime = timerPerceptor.getTime();
		}

		// camera coordinate system is updated to current pose
		updateCameraCoordinateSystem();

		// now inform observers about changes
		observer.onStateChange(this);
		return false;
	}

	protected Pose3D localize(List<ILocalizationLine> visibleFieldLines, Rotation orientationEstimation)
	{
		Pose3D localizeInfo = null;
		try {
			localizeInfo = localizer.localize(this, visibleFieldLines, orientationEstimation);
			localizer.assignReferenceLines(this, localizeInfo, visibleFieldLines, orientationEstimation);
			localizeInfo = localizer.localize(this, visibleFieldLines, orientationEstimation);
		} catch (RuntimeException e) {
		}
		return localizeInfo;
	}

	protected void updateCameraCoordinateSystem()
	{
		// Update current camera-to-root-vision transformation, if possible
		IBodyPart cameraBodyPart = agentModel.getBodyPartContainingCamera();
		if (cameraBodyPart != null) {
			// cameraToRootVision = cameraBodyPart.getPose();
			cameraToRootVision = agentModel.getCameraPose();

			// Transform local body position and orientation into global field
			// system
			Vector3D pos = cameraToRootVision.getPosition();
			cameraToRootVision.position = new Vector3D(pos.getY(), -pos.getX(), pos.getZ());
			cameraToRootVision.orientation =
					Geometry.zTransformRotation(cameraToRootVision.getOrientation(), -Math.PI / 2);
		}
	}

	protected Vector3D toRootVisionSystem(Vector3D visionVector)
	{
		if (visionVector == null) {
			return null;
		}

		if (cameraToRootVision == null) {
			// There exists no transformation, so no system conversion
			return visionVector;
		}

		return cameraToRootVision.getPosition().add(cameraToRootVision.getOrientation().applyTo(visionVector));
	}

	// --------------------------------------------------
	// General environment and game information
	@Override
	public float getGlobalTime()
	{
		return globalTime;
	}

	@Override
	public Collection<ILandmark> getLandmarks()
	{
		return Collections.unmodifiableCollection(landmarks.values());
	}

	@Override
	public ILandmark getLandmark(String name)
	{
		return landmarks.get(name);
	}
	/**
	 * @return the average error of known position and calculated position,
	 *         Double.MAX_VALUE if we do not see any flags
	 */
	@Override
	public double getLandmarkError()
	{
		double result = 0.0;
		int count = 0;
		for (String name : landmarks.keySet()) {
			Landmark landmark = (Landmark) landmarks.get(name);
			// this landmark is not visible currently
			if (landmark.isVisible()) {
				result += landmark.getPosition().subtract(landmark.getKnownPosition()).getNorm();
				count++;
			}
		}
		if (count < 1) {
			return Double.MAX_VALUE;
		}

		return result / count;
	}

	@Override
	public Collection<IFieldLine> getFieldLines()
	{
		return Collections.unmodifiableCollection(fieldLines.values());
	}

	@Override
	public IFieldLine getFieldLine(String name)
	{
		return fieldLines.get(name);
	}

	@Override
	public List<IVisibleObject> getObstacles()
	{
		return obstacles;
	}

	// --------------------------------------------------
	// EnvironmentModel Methods
	@Override
	public List<IReferenceLine> getReferenceLines()
	{
		return referenceLines;
	}

	@Override
	public List<IReferencePoint> getReferencePoints()
	{
		return referencePoints;
	}

	@Override
	public void resetPositionFilter()
	{
		posFilter.reset();
	}

	// --------------------------------------------------
	// Observer methods
	@Override
	public void attach(IObserver<IWorldModel> newObserver)
	{
		observer.attach(newObserver);
	}

	@Override
	public boolean detach(IObserver<IWorldModel> oldObserver)
	{
		return observer.detach(oldObserver);
	}

	// --------------------------------------------------
	// Basic object methods
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof WorldModel)) {
			return false;
		}
		WorldModel other = (WorldModel) o;

		return landmarks.equals(other.landmarks);
	}

	protected IAgentModel getAgentModel()
	{
		return agentModel;
	}
}
