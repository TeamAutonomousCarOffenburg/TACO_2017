/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IForceResistancePerceptor;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.IForceResistance;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.util.misc.FuzzyCompare;

/**
 * Implementation of a "ForceResistance" sensor
 *
 * @author Klaus Dorer
 */
public class ForceResistance extends Sensor implements IForceResistance
{
	/** point where the force acts */
	private Vector3D forceOrigin;

	/** the force itself */
	private Vector3D force;

	/**
	 * counts the number of consecutive update cycles we had force on this sensor
	 */
	private int forceCount;

	/**
	 * counts the number of consecutive update cycles we had no force on this
	 * sensor (not including the current cycle)
	 */
	private int noForceBeforeCount;

	/**
	 * Instantiates a new ForceResistance sensor
	 *
	 * @param name Sensor name
	 */
	public ForceResistance(String name, String perceptorName)
	{
		super(name, perceptorName);
		forceOrigin = Vector3D.ZERO;
		force = Vector3D.ZERO;
	}

	/**
	 * Copy constructor
	 * @param source the object to copy from
	 */
	private ForceResistance(ForceResistance source)
	{
		super(source.name, source.perceptorName);
		forceOrigin = source.forceOrigin;
		force = source.force;
	}

	@Override
	public Vector3D getForceOrigin()
	{
		return forceOrigin;
	}

	/**
	 * Set the force origin
	 *
	 * @param forceOrigin Force origin point
	 */
	void setForceOrigin(Vector3D forceOrigin)
	{
		this.forceOrigin = forceOrigin;
	}

	@Override
	public Vector3D getForce()
	{
		return force;
	}

	/**
	 * Set the force vector
	 *
	 * @param force Force vector
	 */
	void setForce(Vector3D force)
	{
		if (force.getNorm() >= 0.01) {
			forceCount++;
		} else {
			if (forceCount > 0) {
				noForceBeforeCount = 0;
			}
			forceCount = 0;
			noForceBeforeCount++;
		}
		this.force = force;
	}

	/**
	 * Updates this ForceResistances from perception
	 * @param perception the result from server message parsing
	 */
	@Override
	public void updateFromPerception(IPerception perception)
	{
		IForceResistancePerceptor frPerceptor = perception.getForceResistancePerceptor(getPerceptorName());

		if (frPerceptor == null) {
			setForce(Vector3D.ZERO);
			return;
		}

		// Fetch new Force Values
		setForce(frPerceptor.getForce());
		setForceOrigin(frPerceptor.getForceOrigin());
	}

	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof ForceResistance)) {
			return false;
		}
		ForceResistance other = (ForceResistance) o;
		if (!super.equals(other)) {
			return false;
		}
		if (!FuzzyCompare.eq(force, other.force, 0.00001)) {
			return false;
		}
		return FuzzyCompare.eq(forceOrigin, other.forceOrigin, 0.00001f);
	}

	@Override
	public ISensor copy()
	{
		return new ForceResistance(this);
	}

	/**
	 * @return the number of consecutive update cycles we had force on this
	 *         sensor
	 */
	public int getForceCount()
	{
		return forceCount;
	}

	/**
	 * @return the number of consecutive update cycles we had no force on this
	 *         sensor (not including the current cycle)
	 */
	public int getNoForceBeforeCount()
	{
		return noForceBeforeCount;
	}
}
