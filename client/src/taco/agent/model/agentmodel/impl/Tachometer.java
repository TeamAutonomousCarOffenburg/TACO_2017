package taco.agent.model.agentmodel.impl;

import hso.autonomy.agent.communication.perception.IPerception;
import hso.autonomy.util.geometry.Pose3D;
import taco.agent.communication.perception.IAudiCupPerception;
import taco.agent.communication.perception.IWheelTickPerceptor;
import taco.agent.model.agentmodel.ITachometer;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

/**
 * Sensor for the speed of a wheel measured by ticks.
 */
public class Tachometer extends AudiCupSensor implements ITachometer
{
	public static final double CORRECTION_FACTOR = 0.95;

	private static final int TICKS_PER_REVOLUTION = 60;

	private static final double METERS_PER_SECOND_TO_KMH = 3.6;

	/** over how many second to average the speed */
	private static final int SPEED_AVERAGING_CYCLES = 10;

	private final float wheelDiameter;

	/** number of ticks we started with */
	private Long initialTicks;

	private List<Double> ticksPerSecondHistory;

	private double speed;

	/** how many ticks we got from last measurement */
	private long lastTicks;

	/** forward (>0) or backward (<0) */
	private int direction;

	public Tachometer(String name, Pose3D pose, float wheelDiameter)
	{
		super(name, pose);
		this.wheelDiameter = wheelDiameter;
		ticksPerSecondHistory = new ArrayList<>();
		lastTicks = 0;
		direction = 1;
	}

	@Override
	public void updateFromPerception(IPerception perception)
	{
		IWheelTickPerceptor tickPerceptor = ((IAudiCupPerception) perception).getWheelTickPerceptor(getPerceptorName());

		if (tickPerceptor == null) {
			return;
		}

		long newTicks = tickPerceptor.getTicks();
		if (initialTicks == null) {
			initialTicks = newTicks;
		}

		newTicks -= initialTicks;
		long newMeasurementTime = tickPerceptor.getTimeStamp();
		direction = tickPerceptor.getDirection();

		updateSpeed(newTicks, newMeasurementTime);

		// remember measurements
		lastTicks = newTicks;
		lastMeasurementTime = newMeasurementTime;
	}

	private void updateSpeed(long newTicks, long newMeasurementTime)
	{
		// lastMeasurementTime was initialized with 0
		// -> don't calculate ticksPerSecond if that timestamp is still 0
		if (lastMeasurementTime == 0 || lastMeasurementTime >= newMeasurementTime) {
			return;
		}

		// ticksPerSecond is tickDifference (positive or negative) of last and current measurements,
		// multiplied with timeDifference of last and current measurements, multiplied with 1000000 to get from
		// nanoseconds to seconds
		long tickDifference = (newTicks - lastTicks) * direction;
		double timeDifference = 1000000.0 / (newMeasurementTime - lastMeasurementTime);
		ticksPerSecondHistory.add(tickDifference * timeDifference);

		if (ticksPerSecondHistory.size() > SPEED_AVERAGING_CYCLES) {
			ticksPerSecondHistory.remove(0);
		}
		OptionalDouble averageTicksPerSecond = ticksPerSecondHistory.stream().mapToDouble(value -> value).average();
		if (averageTicksPerSecond.isPresent()) {
			speed = ticksToMeters(averageTicksPerSecond.getAsDouble(), wheelDiameter) * METERS_PER_SECOND_TO_KMH;
		}
	}

	@Override
	public double getSpeed()
	{
		return speed;
	}

	@Override
	public long getTicks()
	{
		return lastTicks;
	}

	@Override
	public int getDirection()
	{
		return direction;
	}

	public static double ticksToMeters(double ticks, float wheelDiameter)
	{
		return (ticks / TICKS_PER_REVOLUTION) * wheelDiameter * Math.PI * CORRECTION_FACTOR;
	}

	public static double metersToTicks(double meters, float wheelDiameter)
	{
		return (TICKS_PER_REVOLUTION * meters) / wheelDiameter / Math.PI / CORRECTION_FACTOR;
	}
}
