package taco.agent.model.agentmodel;

public interface IAudiCupMotor extends IAudiCupActuator {
	double LOW_SPEED = 25;

	double DEFAULT_SPEED = 35;

	double HIGH_SPEED = 50;

	/**
	 * Sets the motor speed to <code>DEFAULT_SPEED</code>
	 */
	void driveForward();

	/**
	 * Sets the motor speed to <code>-DEFAULT_SPEED</code>
	 */
	void driveBackward();

	/**
	 * Sets the motor speed
	 * @param speed the target speed [-100 (backwards), 100 (forwards)],
	 */
	void drive(double speed);

	/**
	 * Sets the motor speed to zero
	 */
	void stop();

	/**
	 * @return the target speed [-100 (backwards), 100 (forwards)]
	 */
	double getTargetSpeed();

	boolean isBraking();
}