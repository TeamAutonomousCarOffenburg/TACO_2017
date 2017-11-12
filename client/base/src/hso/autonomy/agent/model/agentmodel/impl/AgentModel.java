/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import java.io.Serializable;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.action.IAction;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmodel.IAgentModel;
import hso.autonomy.agent.model.agentmodel.IBodyModel;
import hso.autonomy.agent.model.agentmodel.IBodyPart;
import hso.autonomy.agent.model.agentmodel.IHingeJoint;
import hso.autonomy.agent.model.agentmodel.impl.ik.IAgentIKSolver;
import hso.autonomy.util.geometry.IPose3D;
import hso.autonomy.util.geometry.Pose3D;
import hso.autonomy.util.observer.IObserver;
import hso.autonomy.util.observer.IPublishSubscribe;
import hso.autonomy.util.observer.Subject;

/**
 * Implementation of the AgentModel. Used to represent all the information the
 * agent has about itself.
 *
 * @author Stefan Glaser, Ingo Schindler
 */
public class AgentModel implements IAgentModel, Serializable
{
	/** observers of agent model */
	protected final transient IPublishSubscribe<IAgentModel> observer;

	private final IAgentMetaModel metaModel;

	/** the body part model of this robot as sensed by last perception */
	protected BodyModel bodyModelSensed;

	/** the body part model of this robot as we expect it after next sense */
	protected BodyModel bodyModelExpected;

	/** the body part model of this robot to define the future */
	protected BodyModel bodyModelFuture;

	/** the name of the camera body part containing the camera */
	protected final String nameOfCameraBodyPart;

	/** the camera offset in the camera body part system */
	protected final IPose3D cameraOffset;

	/**
	 * initializes all known Sensors like: HingeJoints, ForceResistances and
	 * GyroRates
	 */
	public AgentModel(IAgentMetaModel metaModel, IAgentIKSolver ikSolver)
	{
		this.metaModel = metaModel;

		bodyModelSensed = createBodyModel(metaModel, ikSolver);
		bodyModelExpected = bodyModelSensed;
		bodyModelFuture = createBodyModel(bodyModelSensed);
		nameOfCameraBodyPart = metaModel.getNameOfCameraBodyPart();
		cameraOffset = metaModel.getCameraOffset();
		observer = new Subject<>();
	}

	/**
	 * Factory method
	 * @param sourceModel the source from which to create the new body model
	 * @return the specific body model created
	 */
	protected BodyModel createBodyModel(BodyModel sourceModel)
	{
		return new BodyModel(sourceModel);
	}

	/**
	 * Factory method
	 * @param metaModel the agent configuration meta model
	 * @param ikSolver an inverse kinematic solver if required
	 * @return the specific body model created
	 */
	protected BodyModel createBodyModel(IAgentMetaModel metaModel, IAgentIKSolver ikSolver)
	{
		return new BodyModel(metaModel, ikSolver);
	}

	@Override
	public boolean update(IPerception perception)
	{
		boolean result = updateWithFuture(perception);

		// now inform observers about changes
		observer.onStateChange(this);
		return result;
	}

	protected boolean updateWithFuture(IPerception perception)
	{
		// update all Sensors
		bodyModelSensed.updateFromPerception(perception);
		bodyModelExpected = bodyModelFuture;
		bodyModelExpected.updateFromPerception(perception);
		bodyModelFuture = createBodyModel(bodyModelExpected);
		return perception.containsMotion();
	}

	protected boolean updateNoFuture(IPerception perception)
	{
		// we only have two bodyModels effectively
		bodyModelSensed.updateFromPerception(perception);
		bodyModelExpected = bodyModelSensed;
		bodyModelFuture.updateNoPerception();
		return perception.containsMotion();
	}

	@Override
	public void attach(IObserver<IAgentModel> newObserver)
	{
		observer.attach(newObserver);
	}

	@Override
	public boolean detach(IObserver<IAgentModel> oldObserver)
	{
		return observer.detach(oldObserver);
	}

	@Override
	public HingeJoint getHJ(String name)
	{
		return (HingeJoint) bodyModelSensed.getJoint(name);
	}

	@Override
	public HingeJoint getHJExpected(String name)
	{
		return (HingeJoint) bodyModelExpected.getJoint(name);
	}

	@Override
	public IHingeJoint getWriteableHJ(String name)
	{
		// TODO: kdo here we can now switch to a separate instance
		return (IHingeJoint) bodyModelFuture.getSensor(name);
	}

	@Override
	public IBodyPart getBodyPart(String name)
	{
		// TODO: check which model to use
		// Better would be I think to move this method to BodyModel and out of
		// here
		// so that the user decides.
		return bodyModelSensed.getBodyPart(name);
	}

	@Override
	public IBodyPart getTorso()
	{
		return bodyModelSensed.getTorso();
	}

	@Override
	public Vector3D getCenterOfMass()
	{
		// TODO: check which model to use
		// Better would be I think to move this method to BodyModel and out of
		// here
		// so that the user decides.
		return bodyModelSensed.getCenterOfMass();
	}

	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof AgentModel)) {
			return false;
		}
		AgentModel other = (AgentModel) o;
		return bodyModelSensed.equals(other.bodyModelSensed);
	}

	@Override
	public int hashCode()
	{
		return bodyModelSensed.hashCode();
	}

	@Override
	public void shutOff(boolean off)
	{
		bodyModelFuture.shutOff(off);
	}

	@Override
	public ForceResistance getForceResistance(String name)
	{
		// TODO: check which model to use
		// Better would be I think to move this method to BodyModel and out of
		// here
		// so that the user decides.
		return (ForceResistance) bodyModelSensed.getSensor(name);
	}

	@Override
	public GyroRate getGyroRate(String name)
	{
		// TODO: check which model to use
		// Better would be I think to move this method to BodyModel and out of
		// here
		// so that the user decides.
		return (GyroRate) bodyModelSensed.getSensor(name);
	}

	@Override
	public Accelerometer getAccelerometer(String name)
	{
		// TODO: check which model to use
		// Better would be I think to move this method to BodyModel and out of
		// here
		// so that the user decides.
		return (Accelerometer) bodyModelSensed.getSensor(name);
	}

	@Override
	public String toString()
	{
		return bodyModelSensed.toString();
	}

	@Override
	public void reflectTargetStateToAction(IAction action)
	{
		bodyModelFuture.generateTargetStateToAction(action);
	}

	@Override
	public void updateJointsSpeed(IBodyModel source)
	{
		bodyModelFuture.updateJointsSpeed(source);
	}

	@Override
	public IBodyModel getFutureBodyModel()
	{
		return bodyModelFuture;
	}

	@Override
	public IBodyPart getBodyPartContainingCamera()
	{
		if (nameOfCameraBodyPart != null) {
			return getBodyPart(nameOfCameraBodyPart);
		}

		return null;
	}

	@Override
	public IPose3D getCameraOffset()
	{
		return this.cameraOffset;
	}

	@Override
	public Pose3D getCameraPose()
	{
		return getBodyPartContainingCamera().getPose().applyTo(this.cameraOffset);
	}

	@Override
	public List<IHingeJoint> getAllHingeJoints()
	{
		return bodyModelSensed.getAllHingeJoints();
	}

	@Override
	public double getMaxMotorTemperature()
	{
		List<IHingeJoint> joints = getAllHingeJoints();
		double max = 0;
		for (IHingeJoint joint : joints) {
			float temperature = joint.getMotor().getTemperature();
			if (temperature > max) {
				max = temperature;
			}
		}
		return max;
	}

	@Override
	public double getAverageMotorTemperature()
	{
		List<IHingeJoint> joints = getAllHingeJoints();
		if (joints.isEmpty()) {
			return 0;
		}

		double sum = 0;
		for (IHingeJoint joint : joints) {
			sum += joint.getMotor().getTemperature();
		}
		return sum / joints.size();
	}

	@Override
	public double getMaxCoilTemperature()
	{
		List<IHingeJoint> joints = getAllHingeJoints();
		double max = 0;
		for (IHingeJoint joint : joints) {
			float temperatureCoil = joint.getMotor().getCalculatedTemperatureCoil();
			if (temperatureCoil > max) {
				max = temperatureCoil;
			}
		}
		return max;
	}

	@Override
	public double getAverageCoilTemperature()
	{
		List<IHingeJoint> joints = getAllHingeJoints();
		if (joints.isEmpty()) {
			return 0;
		}

		double sum = 0;
		for (IHingeJoint joint : joints) {
			sum += joint.getMotor().getCalculatedTemperatureCoil();
		}
		return sum / joints.size();
	}

	@Override
	public void setStiffness(float stiffness)
	{
		List<IHingeJoint> hinges = bodyModelFuture.getAllHingeJoints();
		for (IHingeJoint hinge : hinges) {
			hinge.getMotor().setStiffness(stiffness);
		}
	}

	protected IAgentMetaModel getMetaModel()
	{
		return metaModel;
	}
}
