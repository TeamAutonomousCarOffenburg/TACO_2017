package taco.agent.model.agentmodel;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import taco.agent.model.agentmeta.impl.DistanceSensorConfiguration;

public interface IUltrasonic extends IAudiCupSensor {
	/**
	 * @return the distance in m
	 */
	double getDistance();

	/**
	 * @return the position of an object sensed in car coordinate system
	 */
	Vector3D getObjectPosition();

	/**
	 * @param threshold the distance in m
	 */
	boolean isCloserThan(double threshold);

	DistanceSensorConfiguration getConfig();
}