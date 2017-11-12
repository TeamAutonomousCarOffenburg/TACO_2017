package taco.agent.model.worldmodel.odometry;

import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.model.agentmodel.impl.Tachometer;
import taco.util.drive.DriveGeometry;

/**
 * Odometry based on gyro measurements for the angle and wheel ticks for the length on a circle
 */
public class GyroOdometry extends Odometry
{
	private final float wheelDiameter;

	private long previousTicksLeft;

	private long previousTicksRight;

	private Angle previousAngle;

	public GyroOdometry(float wheelDiameter, IPose2D startPose)
	{
		super(startPose);
		this.wheelDiameter = wheelDiameter;
		previousTicksLeft = 0;
		previousTicksRight = 0;
		previousAngle = Angle.ZERO;
	}

	public void init(long leftTicks, long rightTicks)
	{
		super.init();
		previousTicksLeft = leftTicks;
		previousTicksRight = rightTicks;
		previousAngle = Angle.ZERO;
	}

	public IPose2D update(long leftTicks, int leftDir, long rightTicks, int rightDir, Angle zAngle)
	{
		double newWheelTicks = ((double) (leftTicks - previousTicksLeft) * leftDir +
									   (double) (rightTicks - previousTicksRight) * rightDir) /
							   2.0;

		Angle deltaAngle = zAngle.subtract(previousAngle);
		double distance = Tachometer.ticksToMeters(newWheelTicks, wheelDiameter);
		pose = pose.applyTo(DriveGeometry.calculateArcPose(distance, deltaAngle));

		previousTicksLeft = leftTicks;
		previousTicksRight = rightTicks;
		previousAngle = zAngle;

		return pose;
	}

	/**
	 * @return the total distance the car has driven so far
	 */
	public double getDrivenDistance()
	{
		return Tachometer.ticksToMeters((previousTicksLeft + previousTicksRight) / 2, wheelDiameter);
	}
}
