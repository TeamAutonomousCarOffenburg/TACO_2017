package taco.agent.model.agentmeta.impl;

import hso.autonomy.agent.model.agentmeta.impl.SensorConfiguration;

/**
 * The deserialized CarMetaModel.json file.
 */
public class CarMetaModel
{
	public static final String DEFAULT_DIRECTORY = "../config";

	private float floorHeight;

	private String steeringServoName;

	private String mainMotorName;

	private AxleConfiguration frontAxle;

	private RearAxleConfiguration rearAxle;

	private MountedSensorConfiguration[] imuConfigs;

	private DistanceSensorConfiguration[] ultrasonicConfigs;

	private MountedSensorConfiguration[] rotationConfigs;

	private SensorConfiguration[] voltageConfigs;

	private CameraConfiguration[] cameraConfigs;

	private CameraConfiguration[] depthCameraConfigs;

	private SensorConfiguration[] motorConfigs;

	private SensorConfiguration[] lightConfigs;

	private SensorConfiguration[] statusConfigs;

	private ServoDriveConfiguration[] servoDriveConfigs;

	public float getFloorHeight()
	{
		return floorHeight;
	}

	public String getSteeringServoName()
	{
		return steeringServoName;
	}

	public String getMainMotorName()
	{
		return mainMotorName;
	}

	public AxleConfiguration getFrontAxle()
	{
		return frontAxle;
	}

	public RearAxleConfiguration getRearAxle()
	{
		return rearAxle;
	}

	public MountedSensorConfiguration[] getImuConfigs()
	{
		return imuConfigs;
	}

	public DistanceSensorConfiguration[] getUltrasonicConfigs()
	{
		return ultrasonicConfigs;
	}

	public MountedSensorConfiguration[] getRotationConfigs()
	{
		return rotationConfigs;
	}

	public SensorConfiguration[] getVoltageConfigs()
	{
		return voltageConfigs;
	}

	public CameraConfiguration[] getCameraConfigs()
	{
		return cameraConfigs;
	}

	public CameraConfiguration[] getDepthCameraConfigs()
	{
		return depthCameraConfigs;
	}

	public SensorConfiguration[] getMotorConfigs()
	{
		return motorConfigs;
	}

	public SensorConfiguration[] getLightConfigs()
	{
		return lightConfigs;
	}

	public SensorConfiguration[] getStatusConfigs()
	{
		return statusConfigs;
	}

	public ServoDriveConfiguration[] getServoDriveConfigs()
	{
		return servoDriveConfigs;
	}

	public double getAxleSpacing()
	{
		return getFrontAxle().getPosition().getX() - getRearAxle().getPosition().getX();
	}
}
