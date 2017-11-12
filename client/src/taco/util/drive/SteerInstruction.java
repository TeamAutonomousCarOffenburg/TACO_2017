package taco.util.drive;

import hso.autonomy.util.geometry.Angle;

public class SteerInstruction
{
	public final Angle steeringAngle;

	public final boolean driveForward;

	public SteerInstruction()
	{
		this(Angle.ZERO, true);
	}

	public SteerInstruction(Angle steeringAngle, boolean driveForward)
	{
		this.steeringAngle = steeringAngle;
		this.driveForward = driveForward;
	}
}
