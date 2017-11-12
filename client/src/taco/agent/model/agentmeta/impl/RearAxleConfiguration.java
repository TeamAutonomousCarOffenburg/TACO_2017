package taco.agent.model.agentmeta.impl;

public class RearAxleConfiguration extends AxleConfiguration
{
	// TODO: why is this part of the meta model?
	private float wheelAngle;

	private String leftWheelTacho;

	private String rightWheelTacho;

	public RearAxleConfiguration(String name, String perceptorName)
	{
		super(name, perceptorName);
	}

	public float getWheelAngle()
	{
		return wheelAngle;
	}

	public String getLeftWheelTacho()
	{
		return leftWheelTacho;
	}

	public String getRightWheelTacho()
	{
		return rightWheelTacho;
	}
}
