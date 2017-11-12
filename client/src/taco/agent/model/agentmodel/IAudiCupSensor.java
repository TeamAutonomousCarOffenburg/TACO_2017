package taco.agent.model.agentmodel;

import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.util.geometry.Pose3D;

public interface IAudiCupSensor extends ISensor {
	/**
	 * @return true if this sensor is finished initializing
	 */
	boolean isInitialized();

	Pose3D getPose();
}