package taco.agent.communication.perception;

import hso.autonomy.agent.communication.perception.IPerceptor;

public interface IAudiCupPerceptor extends IPerceptor {
	/** when the measurement has been done */
	long getTimeStamp();
}