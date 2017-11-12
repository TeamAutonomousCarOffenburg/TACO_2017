package taco.agent.communication.action;

public interface EffectorName {
	String MAIN_MOTOR = "MainMotor";
	String STEERING_SERVO = "SteeringServo";

	String HEAD_LIGHTS = "HeadLights";
	String BACK_LIGHTS = "BackLights";
	String BRAKE_LIGHTS = "BrakeLights";
	String WARN_LIGHTS = "WarnLights";
	String INDICATOR_LIGHTS_LEFT = "IndicatorLightsLeft";
	String INDICATOR_LIGHTS_RIGHT = "IndicatorLightsRight";

	String DRIVE_STATUS = "DriveStatus";
	String JURY_COMMAND = "JuryCommand";
	String MANEUVER_LIST = "ManeuverList";
	String PARKING_SPACE = "ParkingSpace";
	String ROAD_SIGN = "RoadSign";
	String POSITION = "Position";
	String OBSTACLE = "Obstacle";
}
