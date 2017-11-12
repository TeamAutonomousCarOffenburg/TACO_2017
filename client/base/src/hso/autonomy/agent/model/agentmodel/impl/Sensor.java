/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel.impl;

import java.io.Serializable;
import java.util.Map;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmodel.ISensor;

/**
 * Base class for all agent sensors
 *
 * @author Klaus Dorer
 */
public abstract class Sensor implements Serializable, ISensor
{
	/** name of the Sensor */
	final String name;

	/** the name of the corresponding perceptor */
	final String perceptorName;

	/**
	 * Instantiate a new sensor
	 *
	 * @param name Sensor name
	 */
	public Sensor(String name, String perceptorName)
	{
		this.name = name;
		this.perceptorName = perceptorName;
	}

	/**
	 * Copy constructor
	 * @param source to copy from
	 */
	Sensor(Sensor source)
	{
		this.name = source.name;
		this.perceptorName = source.perceptorName;
	}

	/**
	 * Retrieve the sensor name string
	 *
	 * @return Sensor name
	 */
	@Override
	public String getName()
	{
		return name;
	}

	/**
	 * Get perceptor name
	 *
	 * @return Perceptor name
	 */
	public String getPerceptorName()
	{
		return perceptorName;
	}

	/**
	 * Updates the joint values in the body model from perception
	 * @param perception the new perception we made
	 */
	@Override
	public abstract void updateFromPerception(IPerception perception);

	@Override
	public boolean equals(Object obj)
	{
		if (!(obj instanceof Sensor)) {
			return false;
		}
		return name.equals(((Sensor) obj).name);
	}

	@Override
	public String toString()
	{
		return name;
	}

	/**
	 * @return a deep copy of this sensor
	 */
	@Override
	public abstract ISensor copy();

	@Override
	public int hashCode()
	{
		return name.hashCode() + super.hashCode();
	}

	@Override
	public void updateSensors(Map<String, ISensor> flatSensors, Map<String, ISensor> structuredSensors)
	{
		flatSensors.put(getName(), this);
		structuredSensors.put(getName(), this);
	}

	@Override
	public void updateNoPerception()
	{
	}
}
