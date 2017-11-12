package taco.agent.communication.perception.impl;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import com.google.gson.annotations.SerializedName;

import hso.autonomy.agent.communication.perception.IPerceptor;
import hso.autonomy.agent.communication.perception.impl.TimePerceptor;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.misc.GsonUtil;

import static taco.agent.communication.perception.PerceptorName.*;

public class PerceptionMessage
{
	public TimePerceptor time;

	@SerializedName(VOLT_MEASUREMENT)
	public DoublePerceptor measurementCircuit;

	@SerializedName(VOLT_POWER)
	public DoublePerceptor powerCircuit;

	@SerializedName(US_FRONT_LEFT)
	public DoublePerceptor ultrasonicFrontLeft;

	@SerializedName(US_FRONT_CENTER_LEFT)
	public DoublePerceptor ultrasonicFrontCenterLeft;

	@SerializedName(US_FRONT_CENTER)
	public DoublePerceptor ultrasonicFrontCenter;

	@SerializedName(US_FRONT_CENTER_RIGHT)
	public DoublePerceptor ultrasonicFrontCenterRight;

	@SerializedName(US_FRONT_RIGHT)
	public DoublePerceptor ultrasonicFrontRight;

	@SerializedName(US_SIDE_LEFT)
	public DoublePerceptor ultrasonicSideLeft;

	@SerializedName(US_SIDE_RIGHT)
	public DoublePerceptor ultrasonicSideRight;

	@SerializedName(US_REAR_LEFT)
	public DoublePerceptor ultrasonicRearLeft;

	@SerializedName(US_REAR_CENTER)
	public DoublePerceptor ultrasonicRearCenter;

	@SerializedName(US_REAR_RIGHT)
	public DoublePerceptor ultrasonicRearRight;

	@SerializedName(CAR_IMU)
	public ImuPerceptor imu;

	@SerializedName(RIGHT_WHEEL_SPEED)
	public WheelTickPerceptor wheelSpeedRight;

	@SerializedName(LEFT_WHEEL_SPEED)
	public WheelTickPerceptor wheelSpeedLeft;

	@SerializedName(STEERING_SERVO)
	public DoublePerceptor steeringServo;

	@SerializedName(MANEUVER_LIST)
	public ManeuverListPerceptor maneuvers;

	@SerializedName(DRIVE_STATUS)
	public ValuePerceptor<String[]> driveStatus;

	@SerializedName(JURY_COMMAND)
	public JuryPerceptor juryCommand;

	@SerializedName(LANE_MIDDLE)
	public LaneMiddlePerceptor laneMiddle;

	@SerializedName(FLOOR_NORMAL)
	public FloorNormalPerceptor floorNormal;

	@SerializedName(SIGNS)
	public SignPerceptor signs;

	@SerializedName(VISION)
	public VisionPerceptor vision;

	@SerializedName(ENVIRONMENT_CONFIGURATION)
	public EnvironmentConfigPerceptor environmentConfig;

	public Map<String, IPerceptor> toPerceptorMap()
	{
		HashMap<String, IPerceptor> map = new HashMap<>();

		BiConsumer<String, AudiCupPerceptor> add = (key, value) ->
		{
			if (value != null) {
				value.setName(key);
				map.put(key, value);
			}
		};

		map.put(TIME, time);

		add.accept(VOLT_MEASUREMENT, measurementCircuit);
		add.accept(VOLT_POWER, powerCircuit);

		add.accept(US_FRONT_LEFT, ultrasonicFrontLeft);
		add.accept(US_FRONT_CENTER_LEFT, ultrasonicFrontCenterLeft);
		add.accept(US_FRONT_CENTER, ultrasonicFrontCenter);
		add.accept(US_FRONT_CENTER_RIGHT, ultrasonicFrontCenterRight);
		add.accept(US_FRONT_RIGHT, ultrasonicFrontRight);

		add.accept(US_SIDE_LEFT, ultrasonicSideLeft);
		add.accept(US_SIDE_RIGHT, ultrasonicSideRight);

		add.accept(US_REAR_LEFT, ultrasonicRearLeft);
		add.accept(US_REAR_CENTER, ultrasonicRearCenter);
		add.accept(US_REAR_RIGHT, ultrasonicRearRight);

		add.accept(CAR_IMU, imu);

		add.accept(RIGHT_WHEEL_SPEED, wheelSpeedRight);
		add.accept(LEFT_WHEEL_SPEED, wheelSpeedLeft);

		add.accept(STEERING_SERVO, steeringServo);

		add.accept(MANEUVER_LIST, maneuvers);
		add.accept(DRIVE_STATUS, driveStatus);
		add.accept(ENVIRONMENT_CONFIGURATION, environmentConfig);

		add.accept(LANE_MIDDLE, laneMiddle);
		add.accept(JURY_COMMAND, juryCommand);

		add.accept(FLOOR_NORMAL, floorNormal);
		add.accept(SIGNS, signs);
		add.accept(VISION, vision);

		return map;
	}
}
