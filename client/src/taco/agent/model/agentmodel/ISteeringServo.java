package taco.agent.model.agentmodel;

import hso.autonomy.util.geometry.Angle;

public interface ISteeringServo extends IAudiCupActuator {
	/**
	 * Sets the steering angle of the car
	 * @param angle the angle [-30 (right), 30 (left)]
	 */
	void steer(Angle angle);

	/**
	 * Resets the steering angle back to 0.
	 */
	void reset();

	/**
	 * @return the desired steering angle
	 */
	Angle getDesiredAngle();
}