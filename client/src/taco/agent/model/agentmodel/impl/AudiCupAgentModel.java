package taco.agent.model.agentmodel.impl;

import java.util.HashMap;
import java.util.Map;

import hso.autonomy.agent.communication.action.IAction;
import hso.autonomy.agent.communication.action.IEffector;
import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.agent.model.agentmeta.IAgentMetaModel;
import hso.autonomy.agent.model.agentmeta.impl.SensorConfiguration;
import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.agent.model.agentmodel.impl.AgentModel;
import hso.autonomy.agent.model.agentmodel.impl.ik.IAgentIKSolver;
import taco.agent.communication.action.EffectorName;
import taco.agent.communication.action.IAudiCupAction;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.communication.perception.PerceptorName;
import taco.agent.model.agentmeta.impl.AudiCupAgentMetaModel;
import taco.agent.model.agentmeta.impl.CameraConfiguration;
import taco.agent.model.agentmeta.impl.CarMetaModel;
import taco.agent.model.agentmeta.impl.DistanceSensorConfiguration;
import taco.agent.model.agentmeta.impl.MountedSensorConfiguration;
import taco.agent.model.agentmodel.*;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.agentmodel.impl.enums.TachometerPosition;
import taco.agent.model.agentmodel.impl.enums.UltrasonicPosition;

public class AudiCupAgentModel extends AgentModel implements IAudiCupAgentModel
{
	/** map of all sensors of the car */
	private Map<String, IAudiCupSensor> sensors;

	/** map of all actuators of the car */
	private Map<String, IAudiCupActuator> actuators;

	private boolean initialized;

	public AudiCupAgentModel(IAgentMetaModel metaModel, IAgentIKSolver ikSolver)
	{
		super(metaModel, ikSolver);
		sensors = new HashMap<>();
		actuators = new HashMap<>();

		CarMetaModel meta = ((AudiCupAgentMetaModel) metaModel).getCarMetaModel();
		// imu
		for (MountedSensorConfiguration config : meta.getImuConfigs()) {
			sensors.put(config.getName(), new ImuSensor(config.getName(), config.getPose()));
		}

		// wheel ticks
		float wheelDiameter = meta.getFrontAxle().getWheelDiameter();
		for (MountedSensorConfiguration config : meta.getRotationConfigs()) {
			sensors.put(config.getName(), new Tachometer(config.getName(), config.getPose(), wheelDiameter));
		}

		// ultrasonic sensors
		for (DistanceSensorConfiguration config : meta.getUltrasonicConfigs()) {
			sensors.put(config.getName(), new Ultrasonic(config));
		}

		// camera
		for (CameraConfiguration config : meta.getCameraConfigs()) {
			sensors.put(config.getName(), new CameraSensor(config));
		}

		// motor
		for (SensorConfiguration config : meta.getMotorConfigs()) {
			actuators.put(config.getName(), new AudiCupMotor(config.getName()));
		}

		// steering
		for (SensorConfiguration config : meta.getServoDriveConfigs()) {
			actuators.put(config.getName(), new SteeringServo(config.getName()));
		}

		// lights
		for (SensorConfiguration config : meta.getLightConfigs()) {
			actuators.put(config.getName(), new Light(config.getName()));
		}

		// drive status
		for (SensorConfiguration config : meta.getStatusConfigs()) {
			actuators.put(config.getName(), new DriveStatus(config.getName()));
		}

		actuators.put(EffectorName.POSITION, new PositionActuator(EffectorName.POSITION));
		actuators.put(EffectorName.OBSTACLE, new ObstacleActuator(EffectorName.OBSTACLE));
		actuators.put(EffectorName.ROAD_SIGN, new RoadSignActuator(EffectorName.ROAD_SIGN));
		actuators.put(EffectorName.PARKING_SPACE, new ParkingSpaceActuator(EffectorName.PARKING_SPACE));

		initialized = false;
	}

	@Override
	public CarMetaModel getCarMetaModel()
	{
		return ((AudiCupAgentMetaModel) getMetaModel()).getCarMetaModel();
	}

	private boolean checkInitialization()
	{
		for (IAudiCupSensor sensor : sensors.values()) {
			if (!sensor.isInitialized()) {
				return false;
			}
		}

		for (IAudiCupActuator actuator : actuators.values()) {
			if (!actuator.isInitialized()) {
				return false;
			}
		}

		return true;
	}

	@Override
	public IImuSensor getImuSensor()
	{
		return (IImuSensor) sensors.get(PerceptorName.CAR_IMU);
	}

	@Override
	public ITachometer getTachometer(TachometerPosition position)
	{
		return (ITachometer) sensors.get(position.perceptorName);
	}

	@Override
	public IUltrasonic getUltrasonic(UltrasonicPosition position)
	{
		return (IUltrasonic) sensors.get(position.perceptorName);
	}

	@Override
	public IAudiCupMotor getMotor()
	{
		return (IAudiCupMotor) actuators.get(EffectorName.MAIN_MOTOR);
	}

	@Override
	public ISteeringServo getSteering()
	{
		return (ISteeringServo) actuators.get(EffectorName.STEERING_SERVO);
	}

	@Override
	public ILight getLight(LightName name)
	{
		return (ILight) actuators.get(name.effectorName);
	}

	@Override
	public IDriveStatus getDriveStatus()
	{
		return (IDriveStatus) actuators.get(EffectorName.DRIVE_STATUS);
	}

	@Override
	public ICameraSensor getBaslerCamera()
	{
		return (ICameraSensor) sensors.get(PerceptorName.BASLER_CAMERA);
	}

	@Override
	public ICameraSensor getSignDetectionCamera()
	{
		if (getCarMetaModel().getCameraConfigs().length == 2) {
			return (ICameraSensor) sensors.get(PerceptorName.BASLER_CAMERA);
		}
		return (ICameraSensor) sensors.get(PerceptorName.XTION_CAMERA);
	}

	@Override
	public IPositionActuator getCarPositionActuator()
	{
		return (IPositionActuator) actuators.get(EffectorName.POSITION);
	}

	@Override
	public IObstacleActuator getObstaclePositionActuator()
	{
		return (IObstacleActuator) actuators.get(EffectorName.OBSTACLE);
	}

	@Override
	public IParkingSpaceActuator getParkingSpaceActuator()
	{
		return (IParkingSpaceActuator) actuators.get(EffectorName.PARKING_SPACE);
	}

	@Override
	public IRoadSignActuator getRoadSignActuator()
	{
		return (IRoadSignActuator) actuators.get(EffectorName.ROAD_SIGN);
	}

	@Override
	public boolean update(IPerception perception)
	{
		super.update(perception);

		checkBrakeAndBackLights();

		// Update all sensors
		for (ISensor sensor : sensors.values()) {
			sensor.updateFromPerception(perception);
		}

		// Update all actuators
		IAudiCupPerception audiCupPerception = (IAudiCupPerception) perception;
		for (IAudiCupActuator actuator : actuators.values()) {
			actuator.update(audiCupPerception);
		}

		if (!initialized) {
			initialized = checkInitialization();
		}

		return true;
	}

	private void checkBrakeAndBackLights()
	{
		if (getMotor().isBraking()) {
			getLight(LightName.BRAKE).turnOn();
		} else {
			getLight(LightName.BRAKE).turnOff();
		}

		if (getMotor().getTargetSpeed() < 0) {
			getLight(LightName.BACK).turnOn();
		} else {
			getLight(LightName.BACK).turnOff();
		}
	}

	@Override
	public void reflectTargetStateToAction(IAction action)
	{
		IAudiCupAction audiCupAction = (IAudiCupAction) action;
		Map<String, IEffector> effectors = new HashMap<>();
		for (IAudiCupActuator actuator : actuators.values()) {
			actuator.createAction(audiCupAction, effectors);
		}

		audiCupAction.setEffectors(effectors);
	}

	@Override
	public boolean isInitialized()
	{
		return initialized;
	}

	@Override
	public double getCarWidth()
	{
		return getCarMetaModel().getRearAxle().getLength();
	}

	@Override
	public double getCarLength()
	{
		return getUltrasonic(UltrasonicPosition.FRONT_CENTER).getPose().getX();
	}
}
