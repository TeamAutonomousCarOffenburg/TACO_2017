package taco.agent.model.agentmodel.impl;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import hso.autonomy.agent.communication.perception.IPerception;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.communication.perception.IDoublePerceptor;
import taco.agent.model.agentmeta.impl.DistanceSensorConfiguration;
import taco.agent.model.agentmodel.IUltrasonic;

/**
 * Ultrasonic distance sensor.
 */
public class Ultrasonic extends AudiCupSensor implements IUltrasonic
{
	private final DistanceSensorConfiguration config;

	/** the measured distance in m */
	private double distance;

	public Ultrasonic(DistanceSensorConfiguration config)
	{
		super(config.getName(), config.getPose());
		this.config = config;

		distance = config.getMaxDistance();
	}

	@Override
	public void updateFromPerception(IPerception perception)
	{
		IDoublePerceptor perceptor = ((IAudiCupPerception) perception).getDoublePerceptor(getPerceptorName());
		if (perceptor == null) {
			return;
		}

		// here we could add some filtering
		distance = perceptor.getValue();
	}

	@Override
	public double getDistance()
	{
		return distance;
	}

	@Override
	public Vector3D getObjectPosition()
	{
		return config.getPose().applyTo(new Vector3D(distance, 0, 0));
	}

	@Override
	public boolean isCloserThan(double threshold)
	{
		return this.distance < threshold;
	}

	@Override
	public DistanceSensorConfiguration getConfig()
	{
		return config;
	}
}
