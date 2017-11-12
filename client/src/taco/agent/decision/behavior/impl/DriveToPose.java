package taco.agent.decision.behavior.impl;

import java.awt.Color;

import org.apache.commons.math3.geometry.euclidean.twod.SubLine;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.Angle;
import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupBehavior;
import taco.agent.model.agentmodel.IAudiCupMotor;
import taco.util.drive.DriveGeometry;
import taco.util.drive.Line2D;
import taco.util.drive.SteerInstruction;

public class DriveToPose extends AudiCupBehavior
{
	private IPose2D goalPose;

	private IPose2D nextPose;

	private double speed;

	private boolean allowBackwards;

	private boolean forceBackwards;

	public DriveToPose(IThoughtModel thoughtModel)
	{
		super(IBehaviorConstants.DRIVE_TO_POSE, thoughtModel);
	}

	public void setTargetPose(IPose2D nextPose, IPose2D poseAfterNext)
	{
		setTargetPose(nextPose, poseAfterNext, IAudiCupMotor.DEFAULT_SPEED, false, false);
	}

	public void setTargetPose(IPose2D nextPose, IPose2D poseAfterNext, double speed)
	{
		setTargetPose(nextPose, poseAfterNext, speed, false, false);
	}

	public void setTargetPose(IPose2D nextPose, IPose2D poseAfterNext, double speed, boolean allowBackwards)
	{
		setTargetPose(nextPose, poseAfterNext, speed, allowBackwards, false);
	}

	public void setTargetPose(
			IPose2D nextPose, IPose2D poseAfterNext, double speed, boolean allowBackwards, boolean forceBackwards)
	{
		this.goalPose = nextPose;
		this.nextPose = poseAfterNext;
		this.speed = speed;
		this.allowBackwards = allowBackwards;
		this.forceBackwards = forceBackwards;
	}

	@Override
	public void init()
	{
		super.init();
		goalPose = null;
		nextPose = null;
		speed = 0;
		allowBackwards = false;
	}

	@Override
	public void perform()
	{
		IPose2D carPose = getWorldModel().getThisCar().getPose();
		logDebugInfo(goalPose, nextPose, carPose);

		SteerInstruction steerInstruction =
				getDriveGeometry().getNextInstruction(carPose, goalPose, nextPose, allowBackwards, forceBackwards);
		double effectiveSpeed = speed;
		if (!steerInstruction.driveForward) {
			effectiveSpeed *= -1;
		}

		getAgentModel().getSteering().steer(steerInstruction.steeringAngle);
		getAgentModel().getMotor().drive(effectiveSpeed);
	}

	public void resetPreviousInstruction()
	{
		getDriveGeometry().resetPreviousInstruction();
	}

	public void resetPreviousInstruction(Angle angle, boolean driveForwards)
	{
		getDriveGeometry().resetPreviousInstruction(angle, driveForwards);
	}

	private void logDebugInfo(IPose2D nextPose, IPose2D poseAfterNext, IPose2D carPose)
	{
		Line2D line = getDriveGeometry().getLine(carPose, nextPose, allowBackwards);
		if (line != null) {
			getThoughtModel().getDrawings().draw(
					BEHAVIOR_DRAWING, Color.BLACK, new SubLine(line.getStart(), line.getEnd(), 0.0001));
		} else {
			getThoughtModel().getDrawings().draw(BEHAVIOR_DRAWING, Color.BLACK,
					getDriveGeometry().getCircle(
							carPose.applyTo(new Pose2D(getAgentModel().getCarMetaModel().getAxleSpacing(), 0)),
							nextPose, carPose, false));
		}

		getThoughtModel().log("nextWaypoint", poseAfterNext);
	}

	protected DriveGeometry getDriveGeometry()
	{
		return getThoughtModel().getDriveGeometry();
	}
}
