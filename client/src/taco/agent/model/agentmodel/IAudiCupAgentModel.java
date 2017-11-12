package taco.agent.model.agentmodel;

import hso.autonomy.agent.model.agentmodel.IAgentModel;
import taco.agent.model.agentmeta.impl.CarMetaModel;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.agentmodel.impl.enums.TachometerPosition;
import taco.agent.model.agentmodel.impl.enums.UltrasonicPosition;

public interface IAudiCupAgentModel extends IAgentModel {
	CarMetaModel getCarMetaModel();

	/**
	 * @return true if this model has run through the initial setup process
	 */
	boolean isInitialized();

	IImuSensor getImuSensor();

	ITachometer getTachometer(TachometerPosition position);

	IAudiCupMotor getMotor();

	ISteeringServo getSteering();

	IUltrasonic getUltrasonic(UltrasonicPosition position);

	ILight getLight(LightName name);

	IDriveStatus getDriveStatus();

	ICameraSensor getBaslerCamera();

	ICameraSensor getSignDetectionCamera();

	IPositionActuator getCarPositionActuator();

	IObstacleActuator getObstaclePositionActuator();

	IParkingSpaceActuator getParkingSpaceActuator();

	IRoadSignActuator getRoadSignActuator();

	/**
	 * @return the length of the back axle
	 */
	double getCarWidth();

	/**
	 * @return the length of the car from back axle to front us sensor (in m)
	 */
	double getCarLength();
}
