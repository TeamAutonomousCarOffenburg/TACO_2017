package taco.agent.model.worldmodel.impl;

import hso.autonomy.util.geometry.IPose2D;
import hso.autonomy.util.geometry.Pose2D;
import taco.agent.model.worldmodel.IThisCar;

public class ThisCar extends Car implements IThisCar
{
	/** the global pose of this object */
	private IPose2D pose;

	private DrivePath drivePath;

	public ThisCar(IPose2D startPose)
	{
		super("TacoCar", 0.02f);
		setPose(startPose);
		drivePath = new DrivePath();
	}

	@Override
	public IPose2D getPose()
	{
		return pose;
	}

	@Override
	public void setPose(IPose2D newPose)
	{
		pose = newPose;
		position = newPose.getPosition();
	}

	@Override
	public void setPath(DrivePath path)
	{
		this.drivePath = path;
	}

	@Override
	public DrivePath getPath()
	{
		return drivePath;
	}
}
