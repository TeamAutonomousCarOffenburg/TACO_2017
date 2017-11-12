package taco.agent.model.agentmodel;

public interface ITachometer extends IAudiCupSensor {
	/**
	 * @return current speed in km/h
	 */
	double getSpeed();

	/**
	 * @return how many ticks we got from last measurement
	 */
	long getTicks();

	/**
	 * @return forward (>0) or backward (<0)
	 */
	int getDirection();
}