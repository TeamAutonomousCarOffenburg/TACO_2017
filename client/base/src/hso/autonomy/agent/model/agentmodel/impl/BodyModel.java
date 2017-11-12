/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.action.IAction;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmeta.IBodyPartConfiguration;
import hso.autonomy.agent.model.agentmodel.IBodyModel;
import hso.autonomy.agent.model.agentmodel.IBodyPart;
import hso.autonomy.agent.model.agentmodel.IHingeJoint;
import hso.autonomy.agent.model.agentmodel.IJoint;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.agent.model.agentmodel.impl.ik.IAgentIKSolver;
import hso.autonomy.util.function.FunctionUtil;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.geometry.Pose6D;

/**
 * Represents the body with its body parts and joints
 * @author dorer
 */
public class BodyModel implements Serializable, IBodyModel
{
	/** link to the parent body part */
	protected final IBodyPart torso;

	/** a cache for all sensors including joints for faster access */
	private Map<String, ISensor> sensorCache;

	private boolean centerOfMassValid;

	private Vector3D centerOfMass;

	private transient final IAgentIKSolver ikSolver;

	private boolean shutOff;

	/**
	 * Constructor to create the body model from a passed meta model
	 * @param metaModel the meta model containing body part information
	 */
	protected BodyModel(IAgentMetaModel metaModel, IAgentIKSolver ikSolver)
	{
		this.ikSolver = ikSolver;
		Map<String, IBodyPart> bodyParts = createBodyParts(metaModel);
		torso = connectBodyParts(metaModel, bodyParts);
		sensorCache = collectAllSensors();
		centerOfMassValid = false;
	}

	/**
	 * Copy constructor
	 * @param source the source body model to copy
	 */
	protected BodyModel(BodyModel source)
	{
		this.ikSolver = source.ikSolver;
		torso = new BodyPart((BodyPart) source.torso, null);
		sensorCache = collectAllSensors();
	}

	private Map<String, IBodyPart> createBodyParts(IAgentMetaModel metaModel)
	{
		Map<String, IBodyPart> bodyParts = new HashMap<>();

		for (IBodyPartConfiguration config : metaModel.getBodyPartConfigurations()) {
			BodyPart newPart = new BodyPart(config);
			bodyParts.put(config.getName(), newPart);
		}
		return bodyParts;
	}

	private BodyPart connectBodyParts(IAgentMetaModel metaModel, Map<String, IBodyPart> bodyParts)
	{
		BodyPart result = null;
		for (IBodyPartConfiguration config : metaModel.getBodyPartConfigurations()) {
			BodyPart parent = (BodyPart) bodyParts.get(config.getParent());
			BodyPart currentPart = (BodyPart) bodyParts.get(config.getName());
			currentPart.setParent(parent);
			if (parent == null) {
				// we have what we call torso
				result = currentPart;
			}
		}
		return result;
	}

	private Map<String, ISensor> collectAllSensors()
	{
		Map<String, ISensor> result = new HashMap<>(50);
		torso.collectSensors(result);
		return result;
	}

	/**
	 * Creates the action content for sending to the server
	 */
	protected void generateTargetStateToAction(IAction action)
	{
		generateShutoffAction(action);

		Map<String, float[]> actions = new HashMap<>();
		generateJointActions(actions);
		for (String key : actions.keySet()) {
			action.setEffectorValues(key, actions.get(key));
		}
	}

	/**
	 * Informs the action component to create action commands from this body
	 * model
	 * @param actions the action component that creates the action protocol
	 */
	void generateJointActions(Map<String, float[]> actions)
	{
		torso.generateJointActions(actions);
	}

	/**
	 * Called to forward the beam action.
	 */
	void generateShutoffAction(IAction action)
	{
		if (shutOff) {
			action.setMaxGain(0.0f);
		}
	}

	void shutOff(boolean off)
	{
		shutOff = off;
	}

	/**
	 * Updates the joint values in the body model from perception
	 * @param perception the new perception we made
	 */
	protected void updateFromPerception(IPerception perception)
	{
		torso.updateFromPerception(perception);
		centerOfMassValid = false;
	}

	/**
	 * Updates the joint values in the body model if not connected to perception
	 */
	protected void updateNoPerception()
	{
		torso.updateNoPerception();
		centerOfMassValid = false;
	}

	/**
	 * Called to copy back joint speed values from the passed model to this
	 * model. The passed model is usually a copy that has been created earlier
	 * from this object so that the joint model matches.
	 * @param source the body model to take the values from
	 */
	public void updateJointsSpeed(IBodyModel source)
	{
		torso.updateJointsSpeed(source.getTorso());
	}

	@Override
	public ISensor getSensor(String name)
	{
		return sensorCache.get(name);
		// return torso.getSensorDeep(name);
	}

	@Override
	public IJoint getJoint(String name)
	{
		return (IJoint) getSensor(name);
	}

	@Override
	public IBodyPart getTorso()
	{
		return torso;
	}

	@Override
	public IBodyPart getBodyPart(String name)
	{
		return torso.getBodyPart(name);
	}

	@Override
	public Vector3D getCenterOfMass()
	{
		if (!centerOfMassValid) {
			centerOfMass = torso.getCenterOfMass();
		}
		return centerOfMass;
	}

	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof BodyModel)) {
			return false;
		}
		BodyModel other = (BodyModel) o;
		return torso.equals(other.torso);
	}

	@Override
	public void setHingeJointPosition(String name, double value)
	{
		IJoint joint = getJoint(name);
		((HingeJoint) joint).performAxisPosition(value);
	}

	@Override
	public void adjustHingeJointPosition(String name, double delta)
	{
		IJoint joint = getJoint(name);
		((HingeJoint) joint).adjustAxisPosition(delta);
	}

	@Override
	public void performInitialPose()
	{
		LinkedList<IBodyPart> bodyPartsToInitialize = new LinkedList<>();
		Collection<IBodyPart> children;
		IBodyPart currentBodyPart = torso;

		while (currentBodyPart != null) {
			if (currentBodyPart.getJoint() != null) {
				currentBodyPart.getJoint().performInitialPosition();
			}

			// Add all children to list of body part to initialize and poll the
			// next body part to initialize
			children = currentBodyPart.getChildren();
			if (children != null) {
				bodyPartsToInitialize.addAll(children);
			}
			currentBodyPart = bodyPartsToInitialize.pollFirst();
		}
	}

	@Override
	public void resetAllMovements()
	{
		LinkedList<IBodyPart> bodyPartsToInitialize = new LinkedList<>();
		Collection<IBodyPart> children;
		IBodyPart currentBodyPart = torso;

		while (currentBodyPart != null) {
			if (currentBodyPart.getJoint() != null) {
				currentBodyPart.getJoint().resetMovement();
			}

			// Add all children to list of body part to initialize and poll the
			// next body part to initialize
			children = currentBodyPart.getChildren();
			if (children != null) {
				bodyPartsToInitialize.addAll(children);
			}
			currentBodyPart = bodyPartsToInitialize.pollFirst();
		}
	}

	@Override
	public Vector3D[] getCorners(IBodyPart part)
	{
		Pose3D pose = part.getPose();

		Vector3D[] edges = part.getCorners();
		Vector3D[] edgesReturn = new Vector3D[edges.length];

		for (int i = 0; i < edges.length; i++) {
			edgesReturn[i] = pose.getPosition().add(pose.getOrientation().applyTo(edges[i]));
		}

		return edgesReturn;
	}

	@Override
	public boolean moveBodyPartToPose(String targetBodyName, Pose6D targetPose)
	{
		return moveBodyPartToPose(targetBodyName, targetPose.getPosition(), targetPose.getAngles());
	}

	@Override
	public boolean moveBodyPartToPose(String targetBodyName, Vector3D targetPosition, Vector3D targetAngles)
	{
		IBodyPart targetBody = getBodyPart(targetBodyName);

		if (targetBody != null) {
			return ikSolver.solve(targetBody, targetPosition, targetAngles);
		}

		return false;
	}

	/**
	 * Use this method to do inverse kinematics and calculate the joint angles,
	 * speed and acceleration of the target
	 * @param targetBodyName name of the body part for which to solve
	 * @param targetPose an array of three 6D target poses in the time difference
	 *        FunctionUtil.DELTA_T
	 * @return false if the body part is not existing or is not applicable to
	 *         multiple target position calculation.
	 */
	@Override
	public boolean moveBodyPartToPose(String targetBodyName, Pose6D[] targetPose)
	{
		if (targetPose.length != 3) {
			return false;
		}
		IBodyPart targetBody = getBodyPart(targetBodyName);
		if (targetBody == null) {
			return false;
		}
		Vector3D[] targetPosition = new Vector3D[targetPose.length];
		Vector3D[] targetAngles = new Vector3D[targetPose.length];
		for (int i = 0; i < targetPose.length; i++) {
			targetPosition[i] = targetPose[i].getPosition();
			targetAngles[i] = targetPose[i].getAngles();
		}
		double[] deltas0 = ikSolver.calculateDeltaAngles(targetBody, targetPosition[0], targetAngles[0]);
		if (deltas0 == null) {
			return false;
		}
		double[] deltas1 = ikSolver.calculateDeltaAngles(targetBody, targetPosition[1], targetAngles[1]);
		if (deltas1 == null) {
			return false;
		}
		double[] deltas2 = ikSolver.calculateDeltaAngles(targetBody, targetPosition[2], targetAngles[2]);
		if (deltas2 == null) {
			return false;
		}

		// perform the change
		List<IHingeJoint> hingeJoints = targetBody.getBackwardHingeChain();
		Collections.reverse(hingeJoints);
		for (int i = deltas1.length - 1; i >= 0; i--) {
			IHingeJoint joint = hingeJoints.get(i);
			double angle0 = joint.getAngle() + deltas0[i];
			double angle1 = joint.getAngle() + deltas1[i];
			double angle2 = joint.getAngle() + deltas2[i];
			double speedAtDesiredAngle = FunctionUtil.derivative1(angle0, angle2, 2 * FunctionUtil.DELTA_T);
			double accelerationAtDesiredAngle = FunctionUtil.derivative2(angle0, angle1, angle2, FunctionUtil.DELTA_T);
			joint.setFutureValues((float) angle1, (float) speedAtDesiredAngle, (float) accelerationAtDesiredAngle);
		}

		return true;
	}

	public List<IHingeJoint> getAllHingeJoints()
	{
		List<IHingeJoint> result = new ArrayList<>();
		torso.getAllHingeJoints(result);
		return result;
	}
}
