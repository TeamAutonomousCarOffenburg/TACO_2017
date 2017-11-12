package taco.agent.model.agentmodel.impl;

import hso.autonomy.agent.model.agentmodel.ISensor;
import hso.autonomy.agent.model.agentmodel.impl.Sensor;
import hso.autonomy.util.geometry.Pose3D;
import taco.agent.model.agentmodel.IAudiCupSensor;

public abstract class AudiCupSensor extends Sensor implements IAudiCupSensor
{
	/** The time of the last measurement. */
	protected long lastMeasurementTime;

	/** The position of the sensor. */
	protected Pose3D pose;

	public AudiCupSensor(String name, Pose3D pose)
	{
		super(name, name);
		lastMeasurementTime = 0;
		this.pose = pose;
	}

	@Override
	public ISensor copy()
	{
		// we do not copy sensors here
		return null;
	}

	@Override
	public boolean isInitialized()
	{
		return true;
	}

	@Override
	public Pose3D getPose()
	{
		return pose;
	}
}
