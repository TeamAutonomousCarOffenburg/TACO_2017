package taco.agent.communication.perception;

public interface PerceptorName {
	String LEFT_WHEEL_SPEED = "WH_WheelSpeed_Sensor_Left";
	String RIGHT_WHEEL_SPEED = "WH_WheelSpeed_Sensor_Right";

	String US_FRONT_LEFT = "US_Front_Left";
	String US_FRONT_CENTER_LEFT = "US_Front_Center_Left";
	String US_FRONT_CENTER = "US_Front_Center";
	String US_FRONT_CENTER_RIGHT = "US_Front_Center_Right";
	String US_FRONT_RIGHT = "US_Front_Right";

	String US_SIDE_LEFT = "US_Side_Left";
	String US_SIDE_RIGHT = "US_Side_Right";

	String US_REAR_LEFT = "US_Rear_Left";
	String US_REAR_CENTER = "US_Rear_Center";
	String US_REAR_RIGHT = "US_Rear_Right";

	String CAR_IMU = "CarIMU";

	String VOLT_POWER = "VOLT_power_circuit";
	String VOLT_MEASUREMENT = "VOLT_measurement_circuit";

	String STEERING_SERVO = "SteeringServo";

	String DRIVE_STATUS = "DriveStatus";
	String JURY_COMMAND = "JuryCommand";
	String MANEUVER_LIST = "ManeuverList";
	String ENVIRONMENT_CONFIGURATION = "EnvironmentConfiguration";
	String LANE_MIDDLE = "laneMiddle";
	String FLOOR_NORMAL = "FloorNormal";
	String SIGNS = "Signs";
	String VISION = "Vision";
	String TIME = "time";

	String BASLER_CAMERA = "Basler";
	String XTION_CAMERA = "Xtion_RGB";
}
