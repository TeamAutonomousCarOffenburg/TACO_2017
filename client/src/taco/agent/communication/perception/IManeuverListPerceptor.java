package taco.agent.communication.perception;

import java.util.List;

import hso.autonomy.agent.communication.perception.IPerceptor;
import taco.agent.communication.perception.impl.ManeuverPerceptor;

public interface IManeuverListPerceptor extends IPerceptor {
	/**
	 * Retrieve the list of maneuver including its sector.
	 * @return the list of maneuver
	 */
	List<ManeuverPerceptor> getManeuver();
}
