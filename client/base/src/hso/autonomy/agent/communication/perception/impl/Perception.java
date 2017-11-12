/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.communication.perception.impl;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import hso.autonomy.agent.communication.perception.IAccelerometerPerceptor;
import hso.autonomy.agent.communication.perception.ICommandPerceptor;
import hso.autonomy.agent.communication.perception.ICompassPerceptor;
import hso.autonomy.agent.communication.perception.ICompositeJointPerceptor;
import hso.autonomy.agent.communication.perception.IFlagPerceptor;
import hso.autonomy.agent.communication.perception.IForceResistancePerceptor;
import hso.autonomy.agent.communication.perception.IGlobalPosePerceptor;
import hso.autonomy.agent.communication.perception.IGyroPerceptor;
import hso.autonomy.agent.communication.perception.IHingeJointPerceptor;
import hso.autonomy.agent.communication.perception.ILinePerceptor;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.communication.perception.IPerceptor;
import hso.autonomy.agent.communication.perception.IReferencePointPerceptor;
import hso.autonomy.agent.communication.perception.ITimerPerceptor;
import hso.autonomy.agent.communication.perception.IVisibleObjectPerceptor;

/**
 * Represents all data the agent is able to perceive from its environment.
 * Should be updated in every simulation cycle.
 *
 * @author Klaus Dorer, Simon Raffeiner, Stefan Glaser
 */
public class Perception implements IPerception
{
	// Map for all named perceptors
	protected Map<String, IPerceptor> perceptors;

	// Flag - if the current perception contains vision information
	private boolean containsVision;

	// Flag - if the current perception contains motor information
	private boolean containsMotion;

	public Perception()
	{
		perceptors = new HashMap<>();
	}

	@Override
	public ICompositeJointPerceptor getCompositeJointPerceptor(String name)
	{
		return (ICompositeJointPerceptor) perceptors.get(name);
	}

	@Override
	public IHingeJointPerceptor getHingeJointPerceptor(String name)
	{
		return (IHingeJointPerceptor) perceptors.get(name);
	}

	@Override
	public IAccelerometerPerceptor getAccelerationPerceptor(String name)
	{
		return (IAccelerometerPerceptor) perceptors.get(name);
	}

	@Override
	public ICompassPerceptor getCompassPerceptor(String name)
	{
		return (ICompassPerceptor) perceptors.get(name);
	}

	@Override
	public IGyroPerceptor getGyroRatePerceptor(String name)
	{
		return (IGyroPerceptor) perceptors.get(name);
	}

	@Override
	public IForceResistancePerceptor getForceResistancePerceptor(String name)
	{
		return (IForceResistancePerceptor) perceptors.get(name);
	}

	@Override
	public IVisibleObjectPerceptor getVisibleObject(String name)
	{
		return (IVisibleObjectPerceptor) perceptors.get(name);
	}

	@Override
	public IGlobalPosePerceptor getGlobalPose()
	{
		return (IGlobalPosePerceptor) perceptors.get("globalPose");
	}

	@Override
	public ITimerPerceptor getTime()
	{
		return (ITimerPerceptor) perceptors.get("time");
	}

	@Override
	public List<IFlagPerceptor> getFlags()
	{
		return perceptors.values()
				.stream()
				.filter(perceptor -> perceptor instanceof IFlagPerceptor)
				.map(perceptor -> (IFlagPerceptor) perceptor)
				.collect(Collectors.toList());
	}

	@Override
	public ICommandPerceptor getCommandPerceptor()
	{
		return (ICommandPerceptor) perceptors.get("command");
	}

	@Override
	public List<ILinePerceptor> getVisibleLines()
	{
		return perceptors.values()
				.stream()
				.filter(perceptor -> perceptor instanceof ILinePerceptor)
				.map(perceptor -> (ILinePerceptor) perceptor)
				.collect(Collectors.toList());
	}

	@Override
	public List<IReferencePointPerceptor> getReferencePointPerceptor()
	{
		return perceptors.values()
				.stream()
				.filter(perceptor -> perceptor instanceof IReferencePointPerceptor)
				.map(perceptor -> (IReferencePointPerceptor) perceptor)
				.collect(Collectors.toList());
	}

	@Override
	public boolean containsVision()
	{
		return containsVision;
	}

	@Override
	public boolean containsMotion()
	{
		return containsMotion;
	}

	@Override
	public void updatePerceptors(Map<String, IPerceptor> perceptors)
	{
		if (perceptors == null || perceptors.isEmpty()) {
			// nothing to do, might happen at disconnection
			return;
		}

		this.perceptors = perceptors;

		// Clear list of visible objects
		containsVision = false;
		containsMotion = false;

		// Process
		perceptors.values().forEach(this ::processInputPerceptor);
	}

	private void processInputPerceptor(IPerceptor perceptor)
	{
		// Handle sensor perceptors
		if (perceptor instanceof IHingeJointPerceptor || perceptor instanceof ICompositeJointPerceptor) {
			containsMotion = true;
		}

		if (perceptor instanceof IVisibleObjectPerceptor) {
			containsVision = true;
		}
	}

	@Override
	public String toString()
	{
		return perceptors.toString();
	}
}