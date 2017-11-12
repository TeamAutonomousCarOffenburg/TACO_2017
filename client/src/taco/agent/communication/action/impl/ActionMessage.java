package taco.agent.communication.action.impl;

import java.util.Map;

import com.google.gson.annotations.SerializedName;

import hso.autonomy.agent.communication.action.IEffector;

import static taco.agent.communication.action.EffectorName.*;

public class ActionMessage
{
	@SerializedName(HEAD_LIGHTS)
	private boolean headLights;

	@SerializedName(BACK_LIGHTS)
	private boolean backLights;

	@SerializedName(BRAKE_LIGHTS)
	private boolean brakeLights;

	@SerializedName(WARN_LIGHTS)
	private boolean warnLights;

	@SerializedName(INDICATOR_LIGHTS_LEFT)
	private boolean indicatorLightsLeft;

	@SerializedName(INDICATOR_LIGHTS_RIGHT)
	private boolean indicatorLightsRight;

	@SerializedName(MAIN_MOTOR)
	private double mainMotor;

	@SerializedName(STEERING_SERVO)
	private double steeringServo;

	@SerializedName(DRIVE_STATUS)
	private DriveStatusEffector driveStatus;

	@SerializedName(POSITION)
	private PositionEffector position;

	@SerializedName(OBSTACLE)
	private ObstacleEffector obstacles;

	@SerializedName(ROAD_SIGN)
	private RoadSignEffector roadsigns;

	@SerializedName(PARKING_SPACE)
	private ParkingSpaceEffector parkingSpaces;

	public ActionMessage(Map<String, IEffector> effectors)
	{
		for (String key : effectors.keySet()) {
			IEffector effector = effectors.get(key);
			switch (key) {
			case HEAD_LIGHTS:
				headLights = ((BooleanEffector) effector).getValue();
				break;
			case BACK_LIGHTS:
				backLights = ((BooleanEffector) effector).getValue();
				break;
			case BRAKE_LIGHTS:
				brakeLights = ((BooleanEffector) effector).getValue();
				break;
			case WARN_LIGHTS:
				warnLights = ((BooleanEffector) effector).getValue();
				break;
			case INDICATOR_LIGHTS_LEFT:
				indicatorLightsLeft = ((BooleanEffector) effector).getValue();
				break;
			case INDICATOR_LIGHTS_RIGHT:
				indicatorLightsRight = ((BooleanEffector) effector).getValue();
				break;
			case MAIN_MOTOR:
				mainMotor = ((DoubleEffector) effector).getValue();
				break;
			case STEERING_SERVO:
				steeringServo = ((DoubleEffector) effector).getValue();
				break;
			case DRIVE_STATUS:
				driveStatus = (DriveStatusEffector) effector;
				break;
			case POSITION:
				position = (PositionEffector) effector;
				break;
			case OBSTACLE:
				obstacles = (ObstacleEffector) effector;
				break;
			case PARKING_SPACE:
				parkingSpaces = (ParkingSpaceEffector) effector;
				break;
			case ROAD_SIGN:
				roadsigns = (RoadSignEffector) effector;
				break;
			}
		}
	}
}
