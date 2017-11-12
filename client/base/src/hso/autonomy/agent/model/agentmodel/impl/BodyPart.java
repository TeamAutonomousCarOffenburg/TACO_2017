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

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmeta.IBodyPartConfiguration;
import hso.autonomy.agent.model.agentmeta.ICompositeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.IHingeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.ISensorConfiguration;
import hso.autonomy.agent.model.agentmodel.IBodyPart;
import hso.autonomy.agent.model.agentmodel.IHingeJoint;
import hso.autonomy.agent.model.agentmodel.IJoint;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.util.geometry.Pose3D;

/**
 * @author stilnox
 *
 */
public class BodyPart implements IBodyPart, Serializable
{
	/** the name of this body part with which it can be retrieved */
	private final String name;

	/**
	 * reference to the body part to which this is attached, null if no parent
	 */
	private BodyPart parent;

	/** reference to the body parts that are attached to this */
	private final Map<String, BodyPart> children;

	/**
	 * the joint that is used to rotate this bodypart (redundant reference from
	 * sensors)
	 */
	private IJoint joint;

	/** Sensors attached to this body part including joint sensor if existing. */
	private Map<String, ISensor> flatSensors;

	/** Only contains parents of structured sensors. */
	private Map<String, ISensor> structuredSensors;

	/**
	 * translation from this bodypart's center to its parent's center if
	 * joint-angle == 0
	 */
	private final Vector3D translation;

	/**
	 * the position (in local coordinates) where this bodypart in connected to
	 * the parent bodypart.
	 */
	private final Vector3D anchor;

	/** the inverse matrix of anchor */
	private final Vector3D inverseAnchor;

	/**
	 * A Vector that represents the distance(-Vector) from one Joint to the next
	 */
	private Vector3D distanceToParentJoint;

	/** the mass of this body part in kg */
	private final float mass;

	/**
	 * Dimensions of the BodyPart. For now each BodyPart is represented by a Box
	 */
	private final Vector3D geometry;

	private Vector3D[] corners;

	/**
	 * Constructor for this bodypart
	 *
	 * @param name name of the bodypart
	 * @param parent reference to the parent bodypart
	 * @param joint joint that rotates this bodypart
	 * @param translation translation to the parent bodypart (center to center)
	 * @param anchor position where this bodypart rotates around (position of the
	 *        joint relative to this bodypart's center)
	 * @param mass mass of this bodypart
	 * @param geometry the dimensions of this bodypart
	 */
	public BodyPart(String name, BodyPart parent, IJoint joint, Vector3D translation, Vector3D anchor, float mass,
			Vector3D geometry)
	{
		this.name = name;
		this.joint = joint;
		flatSensors = new HashMap<>(3);
		structuredSensors = new HashMap<>(3);
		if (joint != null) {
			joint.updateSensors(flatSensors, structuredSensors);
		}
		this.translation = translation;

		this.anchor = anchor;
		this.inverseAnchor = anchor.negate();

		this.mass = mass;
		this.geometry = geometry;

		children = new HashMap<>(3);
		setParent(parent);
	}

	/**
	 * Constructor to create body part from IBodyPartConfiguration
	 * @param config the configuration
	 */
	public BodyPart(IBodyPartConfiguration config)
	{
		this(config.getName(), null, null, config.getTranslation(), config.getJointAnchor(), config.getMass(),
				config.getGeometry());

		flatSensors = new HashMap<>(3);
		structuredSensors = new HashMap<>(3);
		ISensorConfiguration jointConfig = config.getJointConfiguration();

		if (jointConfig != null) {
			IJoint newJoint = null;
			if (jointConfig instanceof IHingeJointConfiguration) {
				// If the joint configuration is a HingeJoint-Type, add a new
				// HingeJoint to the sensors list
				newJoint = new HingeJoint((IHingeJointConfiguration) jointConfig);

			} else if (jointConfig instanceof ICompositeJointConfiguration) {
				newJoint = new CompositeJoint((ICompositeJointConfiguration) jointConfig);
			} else {
				// further joint types
			}
			if (newJoint != null) {
				joint = newJoint;
				joint.updateSensors(flatSensors, structuredSensors);
			}
		}
		ISensorConfiguration sensorConfig;

		sensorConfig = config.getGyroRateConfiguration();
		if (sensorConfig != null) {
			// If the body part has a gyro-rate perceptor attached, create
			// one
			GyroRate gyroRate = new GyroRate(sensorConfig.getName(), sensorConfig.getPerceptorName());
			gyroRate.updateSensors(flatSensors, structuredSensors);
		}

		sensorConfig = config.getAccelerometerConfiguration();
		if (sensorConfig != null) {
			// If the body part has a accelerometer attached, create one
			Accelerometer accelerometer = new Accelerometer(sensorConfig.getName(), sensorConfig.getPerceptorName());
			accelerometer.updateSensors(flatSensors, structuredSensors);
		}

		sensorConfig = config.getForceResistanceConfiguration();
		if (sensorConfig != null) {
			// If the body part has a force-resistance perceptor attached,
			// create one
			ForceResistance forceResistance =
					new ForceResistance(sensorConfig.getName(), sensorConfig.getPerceptorName());
			forceResistance.updateSensors(flatSensors, structuredSensors);
		}

		sensorConfig = config.getCompassConfig();
		if (sensorConfig != null) {
			// If the body part has a compass perceptor attached,
			// create one
			Compass compass = new Compass(sensorConfig.getName(), sensorConfig.getPerceptorName());
			compass.updateSensors(flatSensors, structuredSensors);
		}
	}

	/**
	 * Copy constructor
	 * @param source the object from which to copy values from
	 * @param parent the parent of this object (!= parent of other)
	 */
	public BodyPart(BodyPart source, BodyPart parent)
	{
		// TODO kdo: check if this is really non-mutable
		// or should we just make a deep copy since it is cheap for matrices?
		// non deep copy for non changing parts
		this.name = source.name;
		this.parent = parent;
		this.translation = source.translation;
		this.anchor = source.anchor;
		this.inverseAnchor = source.inverseAnchor;
		this.mass = source.mass;
		this.geometry = source.geometry;
		setDistanceToParentJoint();

		// deep copy of the mutable parts and children
		flatSensors = new HashMap<>();
		structuredSensors = new HashMap<>();
		for (ISensor sensor : source.structuredSensors.values()) {
			ISensor newSensor = sensor.copy();
			newSensor.updateSensors(flatSensors, structuredSensors);
		}
		if (source.joint != null) {
			// we have already copied joint through sensors
			this.joint = (IJoint) structuredSensors.get(source.joint.getName());
		} else {
			joint = null;
		}

		// we have to be sure that body parts are strictly hierarchical
		children = new HashMap<>();
		for (BodyPart child : source.children.values()) {
			BodyPart copy = new BodyPart(child, this);
			addChild(copy);
		}
	}

	@Override
	public Vector3D getTranslation()
	{
		return translation;
	}

	@Override
	public Vector3D getAnchor()
	{
		return anchor;
	}

	@Override
	public String getName()
	{
		return name;
	}

	@Override
	public IBodyPart getParent()
	{
		return parent;
	}

	@Override
	public Vector3D getPosition()
	{
		if (parent != null) {
			return getPose().getPosition();
		}

		return Vector3D.ZERO;
	}

	@Override
	public Rotation getOrientation()
	{
		Rotation orientation;

		if (parent != null) {
			orientation = parent.getOrientation();
			if (joint != null) {
				orientation = orientation.applyTo(joint.getRotation());
			}
		} else {
			orientation = Rotation.IDENTITY;
		}

		return orientation;
	}

	@Override
	public Pose3D getPose()
	{
		Pose3D pose = getJointTransformation();

		pose.position = pose.getPosition().add(pose.getOrientation().applyTo(inverseAnchor));

		return pose;
	}

	@Override
	public Pose3D getPose(Vector3D localPos)
	{
		Pose3D pose = getJointTransformation();

		pose.position = pose.getPosition().add(pose.getOrientation().applyTo(inverseAnchor.add(localPos)));

		return pose;
	}

	@Override
	public Pose3D getJointTransformation()
	{
		// TODO kdo: we might want to calculate this information once
		// that would require that the object is immutable and created each time
		// we have updates on the joints. In that case it should be calculated top
		// down starting with the root parent.
		Pose3D parentPose;
		Rotation orientation;
		Vector3D jointPosition;

		if (parent != null) {
			parentPose = parent.getJointTransformation();
			orientation = parentPose.getOrientation();
			jointPosition = parentPose.getPosition().add(orientation.applyTo(distanceToParentJoint));

			if (joint != null) {
				orientation = orientation.applyTo(joint.getRotation());
			}
		} else {
			orientation = Rotation.IDENTITY;
			jointPosition = Vector3D.ZERO;
			parentPose = new Pose3D(jointPosition, orientation);
		}

		parentPose.position = jointPosition;
		parentPose.orientation = orientation;

		return parentPose;
	}

	/**
	 * Set Parent BodyPart
	 */
	public void setParent(BodyPart parent)
	{
		this.parent = parent;

		if (parent != null) {
			parent.addChild(this);
		}

		setDistanceToParentJoint();
	}

	private void setDistanceToParentJoint()
	{
		if (parent == null) {
			distanceToParentJoint = Vector3D.ZERO;
		} else {
			distanceToParentJoint = new Vector3D(-parent.anchor.getX() + translation.getX() + anchor.getX(),
					-parent.anchor.getY() + translation.getY() + anchor.getY(),
					-parent.anchor.getZ() + translation.getZ() + anchor.getZ());
		}
	}

	@Override
	public float getMass()
	{
		return mass;
	}

	@Override
	public Vector3D getGeometry()
	{
		return geometry;
	}

	/**
	 * Adds the passed body part to the map of child body parts
	 * @param bodyPart the child body part to add
	 */
	private void addChild(BodyPart bodyPart)
	{
		children.put(bodyPart.getName(), bodyPart);
	}

	/**
	 * @return the number of child body parts
	 */
	@Override
	public int getNoOfChildren()
	{
		return children.size();
	}

	/**
	 * @param name name of the child body part
	 * @return the child body part, null if not existing
	 */
	@Override
	public BodyPart getChild(String name)
	{
		return children.get(name);
	}

	@Override
	public Collection<IBodyPart> getChildren()
	{
		return Collections.<IBodyPart>unmodifiableCollection(children.values());
	}

	/**
	 * @return a map of all hinge joints in this model
	 */
	@Override
	public ISensor getSensorDeep(String sensorName)
	{
		if (flatSensors.containsKey(sensorName)) {
			return flatSensors.get(sensorName);
		}
		for (BodyPart child : children.values()) {
			ISensor result = child.getSensorDeep(sensorName);
			if (result != null) {
				return result;
			}
		}
		return null;
	}

	@Override
	public void collectSensors(Map<String, ISensor> result)
	{
		result.putAll(flatSensors);
		for (BodyPart child : children.values()) {
			child.collectSensors(result);
		}
	}

	@Override
	public <T extends ISensor> T getSensor(Class<T> type)
	{
		for (ISensor s : flatSensors.values()) {
			if (type.isInstance(s)) {
				return type.cast(s);
			}
		}

		return null;
	}

	/**
	 * Informs the action component to create action commands from this body
	 * model
	 * @param actions the action component that creates the action protocol
	 */
	@Override
	public void generateJointActions(Map<String, float[]> actions)
	{
		if (joint != null) {
			joint.generateJointAction(actions);
		}
		for (BodyPart child : children.values()) {
			child.generateJointActions(actions);
		}
	}

	/**
	 * Updates the joint values in the body model from perception
	 * @param perception the new perception we made
	 */
	@Override
	public void updateFromPerception(IPerception perception)
	{
		for (ISensor sensor : structuredSensors.values()) {
			// this also updates joint
			sensor.updateFromPerception(perception);
		}

		for (BodyPart child : children.values()) {
			child.updateFromPerception(perception);
		}
	}

	/**
	 * Updates the joint values in the body model from perception
	 */
	@Override
	public void updateNoPerception()
	{
		// this also updates joint
		structuredSensors.values().forEach(ISensor::updateNoPerception);
		children.values().forEach(BodyPart::updateNoPerception);
	}

	@Override
	public IJoint getJoint()
	{
		return joint;
	}

	@Override
	public String toString()
	{
		return name;
	}

	@Override
	public BodyPart getBodyPart(String name)
	{
		if (name.equals(this.name)) {
			return this;
		}
		for (BodyPart child : children.values()) {
			BodyPart result = child.getBodyPart(name);
			if (result != null) {
				return result;
			}
		}
		return null;
	}

	@Override
	public Vector3D getCenterOfMass()
	{
		Vector3D coM = getPosition().scalarMultiply(mass);
		for (BodyPart child : children.values()) {
			coM = coM.add(child.getCenterOfMass());
		}

		if (parent == null) {
			coM = coM.scalarMultiply(1.0f / getWholeMass());
		}
		return coM;
	}

	@Override
	public float getWholeMass()
	{
		float wholeMass = mass;
		for (BodyPart child : children.values()) {
			wholeMass += child.getWholeMass();
		}
		return wholeMass;
	}

	@Override
	public boolean equals(Object obj)
	{
		if (!(obj instanceof BodyPart)) {
			return false;
		}
		BodyPart other = (BodyPart) obj;
		if (!name.equals(other.name) || !translation.equals(other.translation) || !anchor.equals(other.anchor) ||
				!geometry.equals(other.geometry) || !children.equals(other.children) ||
				!flatSensors.equals(other.flatSensors)) {
			return false;
		}
		if (parent == null && other.parent != null) {
			// we do not check the parent for equality to avoid endless loop
			return false;
		}
		return !(joint == null && other.joint != null);
	}

	@Override
	public void updateJointsSpeed(IBodyPart part)
	{
		if (joint != null) {
			joint.updateJointPositionFromJoint(part.getJoint());
		}
		assert children.size() == part.getNoOfChildren()
			: "updating joint speeds from invalid source model: children: " +
										  children.size() + " other: " + part.getNoOfChildren() + " this part: " + this;

		for (IBodyPart child : children.values()) {
			IBodyPart otherChild = part.getChild(child.getName());
			assert otherChild != null : "updating joint speeds from invalid source model: child is null";
			child.updateJointsSpeed(otherChild);
		}
	}

	@Override
	public double getGlobalPitch(Rotation globalOrientation)
	{
		// transform local into global orientation
		double[][] orientation = getOrientation().applyTo(globalOrientation).getMatrix();
		return Math.toDegrees(Math.asin(orientation[1][2]));
	}

	@Override
	public double getGlobalRoll(Rotation globalOrientation)
	{
		// transform local into global orientation
		double[][] orientation = getOrientation().applyTo(globalOrientation).getMatrix();
		return Math.toDegrees(Math.asin(orientation[0][2]));
	}

	@Override
	public Vector3D[] getCorners()
	{
		if (corners == null) {
			Vector3D halfGeometry = geometry.scalarMultiply(0.5);

			corners = new Vector3D[8];

			corners[0] = new Vector3D(-halfGeometry.getX(), halfGeometry.getY(), -halfGeometry.getZ());
			corners[1] = new Vector3D(halfGeometry.getX(), halfGeometry.getY(), -halfGeometry.getZ());
			corners[3] = new Vector3D(-halfGeometry.getX(), -halfGeometry.getY(), -halfGeometry.getZ());
			corners[2] = new Vector3D(halfGeometry.getX(), -halfGeometry.getY(), -halfGeometry.getZ());

			corners[4] = new Vector3D(halfGeometry.getX(), halfGeometry.getY(), halfGeometry.getZ());
			corners[5] = new Vector3D(halfGeometry.getX(), -halfGeometry.getY(), halfGeometry.getZ());
			corners[7] = new Vector3D(-halfGeometry.getX(), halfGeometry.getY(), halfGeometry.getZ());
			corners[6] = new Vector3D(-halfGeometry.getX(), -halfGeometry.getY(), halfGeometry.getZ());
		}

		return corners;
	}

	@Override
	public double[][] getJacobian(Vector3D endEffectorPos, Vector3D targetAngles)
	{
		double[][] jacobian = null;

		// Transform end effector position
		Pose3D pose = getPose(endEffectorPos);
		Vector3D globalEndEffectorPos = pose.getPosition();

		// Collect all body parts up to the torso (without the torso)
		LinkedList<IBodyPart> parts = new LinkedList<>();
		IBodyPart currentBodyPart = this;

		while (currentBodyPart.getParent() != null) {
			parts.addFirst(currentBodyPart);
			currentBodyPart = currentBodyPart.getParent();
		}

		// Create the jacobian matrix
		if (targetAngles != null) {
			jacobian = new double[6][parts.size()];
		} else {
			jacobian = new double[3][parts.size()];
		}

		for (int i = 0; i < parts.size(); i++) {
			addJacobianColumn(jacobian, i, globalEndEffectorPos, targetAngles,
					((IHingeJoint) parts.get(i).getJoint()).getJointAxis(), parts.get(i).getJointTransformation());
		}

		return jacobian;
	}

	/**
	 * Add a column to the jacobian matrix.
	 *
	 * @param jacobian - jacobian matrix
	 * @param col - column to add
	 * @param targetPos - target position of the end effector (in root-body
	 *        system)
	 * @param targetAngles - target yaw-pitch-roll angels of the body part
	 * @param jointAxis - rotation axis of the joint
	 * @param jointPose - pose of the joint
	 */
	private void addJacobianColumn(double[][] jacobian, int col, Vector3D targetPos, Vector3D targetAngles,
			Vector3D jointAxis, Pose3D jointPose)
	{
		Vector3D axis = jointPose.getOrientation().applyTo(jointAxis);
		Vector3D n = targetPos.subtract(jointPose.getPosition());

		if (targetAngles != null && jacobian.length == 6) {
			jacobian[3][col] = axis.getX() / 200;
			jacobian[4][col] = axis.getY() / 200;
			jacobian[5][col] = axis.getZ() / 200;
		}

		axis = Vector3D.crossProduct(axis, n);

		jacobian[0][col] = axis.getX();
		jacobian[1][col] = axis.getY();
		jacobian[2][col] = axis.getZ();
	}

	@Override
	public void getAllHingeJoints(List<IHingeJoint> result)
	{
		for (ISensor sensor : flatSensors.values()) {
			if (sensor instanceof IHingeJoint) {
				result.add((IHingeJoint) sensor);
			}
		}

		for (BodyPart child : children.values()) {
			child.getAllHingeJoints(result);
		}
	}

	@Override
	public List<IHingeJoint> getBackwardHingeChain()
	{
		List<IHingeJoint> hingeJoints = new ArrayList<>();
		IBodyPart currentBody = this;
		while (currentBody.getJoint() != null) {
			IJoint joint = currentBody.getJoint();
			List<IJoint> allJoints = joint.getAllSubJoints();
			// since we go backward, the order of composite joints have to be
			// reversed
			Collections.reverse(allJoints);
			for (IJoint subjoint : allJoints) {
				hingeJoints.add((IHingeJoint) subjoint);
			}
			currentBody = currentBody.getParent();
		}
		return hingeJoints;
	}
}
