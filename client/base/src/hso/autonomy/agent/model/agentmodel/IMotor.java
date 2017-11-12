/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmodel;

/**
 * Represents a Motor of a joint
 */
public interface IMotor {
	float getPerceivedAngle();

	void setPerceivedAngle(float angle);

	/**
	 * @param angle the new desired angle
	 * @return the speed to get to the desired angle
	 */
	float getNextSpeed(float angle);

	float getMaxSpeed();

	float getSpeed();

	float getLoad();

	float getVoltage();

	float getTemperature();

	float getCalculatedTemperature();

	float getCalculatedTemperatureCoil();

	byte getError();

	void setStiffness(float stiffness);

	float getStiffness();

	float getGain();

	void setGain(float gain);
}