package taco.agent.communication.perception;

public interface IWheelTickPerceptor extends IAudiCupPerceptor {
	long getTicks();

	int getDirection();
}