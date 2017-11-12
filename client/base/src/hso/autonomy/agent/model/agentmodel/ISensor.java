/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

import java.util.Map;

import hso.autonomy.agent.communication.perception.IPerception;

/**
 *
 * @author dorer
 */
public interface ISensor {
	/**
	 * Retrieve the sensor name string
	 *
	 * @return Sensor name
	 */
	String getName();

	/**
	 * Updates the joint values in the body model from perception
	 * @param perception the new perception we made
	 */
	void updateFromPerception(IPerception perception);

	/**
	 * Updates the joint values in the body model with no perception
	 */
	void updateNoPerception();

	/**
	 * @return a deep copy of this sensor
	 */
	ISensor copy();

	/**
	 * Adds this sensor the appropriate list depending if it is structured or not
	 */
	void updateSensors(Map<String, ISensor> flatSensors, Map<String, ISensor> structuredSensors);
}