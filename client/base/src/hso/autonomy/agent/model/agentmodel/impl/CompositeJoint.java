/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import hso.autonomy.agent.communication.perception.ICompositeJointPerceptor;
import hso.autonomy.agent.communication.perception.IHingeJointPerceptor;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmeta.ICompositeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.IHingeJointConfiguration;
import hso.autonomy.agent.model.agentmeta.IJointToMotorMapper;
import hso.autonomy.agent.model.agentmodel.ICompositeJoint;
import hso.autonomy.agent.model.agentmodel.IJoint;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.util.function.FunctionUtil;

/**
 * "CompositeJoint" sensor implementation
 *
 * @author Stefan Glaser, Klaus Dorer
 */
public class CompositeJoint extends Joint implements ICompositeJoint
{
	/** first hinge joint */
	private HingeJoint[] joints;

	private final transient IJointToMotorMapper mapper;

	/**
	 * Instantiates a new CompositeJoint sensor
	 *
	 * @param jointConfig - the joint meta model
	 */
	public CompositeJoint(ICompositeJointConfiguration jointConfig)
	{
		super(jointConfig.getName(), jointConfig.getPerceptorName());

		IHingeJointConfiguration[] configs = jointConfig.getHjConfigurations();
		joints = new HingeJoint[configs.length];
		for (int i = 0; i < configs.length; i++) {
			IHingeJointConfiguration config = configs[i];
			this.joints[i] = new HingeJoint(config);
		}
		this.mapper = jointConfig.getJointToMotorMapper();
	}

	/**
	 * Copy constructor
	 * @param source to copy from
	 */
	private CompositeJoint(CompositeJoint source)
	{
		super(source);

		joints = new HingeJoint[source.joints.length];
		for (int i = 0; i < joints.length; i++) {
			this.joints[i] = new HingeJoint(source.joints[i]);
		}
		this.mapper = source.mapper;
	}

	@Override
	public float getAngle(int i)
	{
		return joints[i].getAngle();
	}

	@Override
	public float getMinAngle(int i)
	{
		return joints[i].getMinAngle();
	}

	@Override
	public float getMaxAngle(int i)
	{
		return joints[i].getMaxAngle();
	}

	@Override
	public float getNextAxisSpeed(int i)
	{
		return joints[i].getNextAxisSpeed();
	}

	@Override
	public void performAxisSpeed(int i, float speed)
	{
		joints[i].performAxisSpeed(speed);
	}

	@Override
	public void performAxisPosition(int i, float position)
	{
		joints[i].performAxisPosition(position);
	}

	@Override
	public void performAxisPosition(int i, float position, float maxSpeed)
	{
		joints[i].performAxisPosition(position, maxSpeed);
	}

	@Override
	public void performInitialPosition()
	{
		for (HingeJoint joint : joints) {
			joint.performInitialPosition();
		}
	}

	@Override
	public void resetMovement()
	{
		for (HingeJoint joint : joints) {
			joint.resetMovement();
		}
	}

	@Override
	public Rotation getRotation()
	{
		Rotation result = joints[0].getRotation();
		for (int i = 1; i < joints.length; i++) {
			result = joints[i].getRotation().applyTo(result);
		}
		return result;
	}

	@Override
	public ISensor copy()
	{
		return new CompositeJoint(this);
	}

	@SuppressWarnings("unused")
	@Override
	public void generateJointAction(Map<String, float[]> actions)
	{
		if (true) {
			functionMapping(actions);
		} else {
			simpleMapping(actions);
		}
	}

	/**
	 * Does a simple joint angles to motor angles mapping not considering speed
	 * or acceleration
	 */
	private void simpleMapping(Map<String, float[]> actions)
	{
		double[] jointAngles = new double[joints.length];
		for (int i = 0; i < joints.length; i++) {
			jointAngles[i] = joints[i].getAngle();
		}

		double[] motorAngles = jointAngles;
		if (mapper != null) {
			motorAngles = mapper.jointToMotorAngle(jointAngles);
		}

		float[] values = new float[joints.length * 5];
		for (int i = 0; i < joints.length; i++) {
			int j = i * 5;
			values[j] = joints[i].getMotor().getNextSpeed((float) motorAngles[i]);
			values[j + 1] = (float) motorAngles[i];
			values[j + 2] = 0; // TODO KDO speed
			values[j + 3] = 0; // acceleration

			values[j + 4] = joints[i].getMotor().getGain();
		}

		// if (getName().equals(ISweatyRealConstants.NeckYawCombined)) {
		// System.out.println(jointAngles[0] + "; " + motorAngles[0]);
		// }

		actions.put(getName(), values);
	}

	private void functionMapping(Map<String, float[]> actions)
	{
		double[][] jointAngles = new double[3][joints.length];
		double dt = FunctionUtil.DELTA_T;
		double dt2 = dt * dt;
		for (int i = 0; i < joints.length; i++) {
			double fx = joints[i].getAngle();
			double f1x = joints[i].getSpeedAtDesiredAngle();
			double f2x = joints[i].getAccelerationAtDesiredAngle();
			// recalculation of function values at +/-dt support points from speed
			// and
			// acceleration
			jointAngles[0][i] = fx - dt * f1x + 0.5 * dt2 * f2x;
			jointAngles[1][i] = fx;
			jointAngles[2][i] = fx + dt * f1x + 0.5 * dt2 * f2x;

			// if (getName().equals(ISweatyRealConstants.LAnkleCombined)) {
			// System.out.print("; " + fx + "; " + f1x + "; " + f2x + "; ");
			// System.out.print(jointAngles[0][i] + "; " + jointAngles[1][i]
			// + "; " + jointAngles[2][i] + "; ");
			// }
		}

		double[][] motorAngles = jointAngles;
		if (mapper != null) {
			motorAngles[0] = mapper.jointToMotorAngle(jointAngles[0]);
			motorAngles[1] = mapper.jointToMotorAngle(jointAngles[1]);
			motorAngles[2] = mapper.jointToMotorAngle(jointAngles[2]);
		}

		float[] values = new float[joints.length * 5];
		for (int i = 0; i < joints.length; i++) {
			int j = i * 5;
			float fx = (float) motorAngles[1][i];
			float f1x = (float) FunctionUtil.derivative1(motorAngles[0][i], motorAngles[2][i], 2 * dt);
			float f2x = (float) FunctionUtil.derivative2(motorAngles[0][i], motorAngles[1][i], motorAngles[2][i], dt);
			values[j] = joints[i].getMotor().getNextSpeed(fx);
			values[j + 1] = fx;
			values[j + 2] = f1x;
			values[j + 3] = f2x;
			values[j + 4] = joints[i].getMotor().getGain();
			// if (getName().equals(ISweatyRealConstants.NeckYawCombined)) {
			// System.out.println("Weakness: " + values[j + 4]);
			// }
			// if (getName().equals(ISweatyRealConstants.LAnkleCombined)) {
			// System.out.print("; " + fx + "; " + f1x + "; " + f2x + "; ");
			// }
		}
		// if (getName().equals(ISweatyRealConstants.LAnkleCombined)) {
		// System.out.println();
		// }

		actions.put(getName(), values);
	}

	@Override
	public void updateFromPerception(IPerception perception)
	{
		ICompositeJointPerceptor ujPerceptor = perception.getCompositeJointPerceptor(getPerceptorName());

		if (ujPerceptor == null) {
			return;
		}

		IHingeJointPerceptor[] jointPerceptors = ujPerceptor.getJoints();
		double[] motorAngles = new double[joints.length];
		for (int i = 0; i < joints.length; i++) {
			motorAngles[i] = jointPerceptors[i].getAxis();
		}

		// map motor to joint angles
		double[] jointAngles = motorAngles;
		if (mapper != null) {
			jointAngles = mapper.motorToJointAngle(motorAngles);
		}

		// if (getName().equalsIgnoreCase(ISweatyRealConstants.LHipCombined)) {
		// System.out.println("Left Hip: " + Arrays.toString(jointAngles));
		// }

		for (int i = 0; i < joints.length; i++) {
			// apply the possible change of the motor mapper
			jointPerceptors[i].setAxis((float) jointAngles[i]);
			joints[i].updateFromPerception(jointPerceptors[i]);
		}
	}

	@Override
	public void updateNoPerception()
	{
		for (HingeJoint joint : joints) {
			joint.updateNoPerception();
		}
	}

	@Override
	public void updateJointPositionFromJoint(IJoint joint)
	{
		CompositeJoint cj = (CompositeJoint) joint;
		for (int i = 0; i < joints.length; i++) {
			joints[i].updateJointPositionFromJoint(cj.joints[i]);
		}
	}

	@Override
	public List<IJoint> getAllSubJoints()
	{
		List<IJoint> result = new ArrayList<>();
		Collections.addAll(result, joints);
		return result;
	}

	@Override
	public void updateSensors(Map<String, ISensor> flatSensors, Map<String, ISensor> structuredSensors)
	{
		for (HingeJoint joint : joints) {
			flatSensors.put(joint.getName(), joint);
		}
		structuredSensors.put(getName(), this);
	}

	public void setFutureValues(int i, float desiredAngle, float speedAtDesiredAngle, float accelerationAtDesiredAngle)
	{
		joints[i].setFutureValues(desiredAngle, speedAtDesiredAngle, accelerationAtDesiredAngle);
	}
}
