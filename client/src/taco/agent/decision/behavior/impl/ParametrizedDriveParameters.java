package taco.agent.decision.behavior.impl;

import hso.autonomy.util.geometry.Angle;
import kdo.util.parameter.EnumParameterList;
import kdo.util.parameter.ParameterList;
import taco.agent.model.agentmodel.IAudiCupMotor;

public class ParametrizedDriveParameters extends ParameterList
{
	/** how many cycles there are between instructions */
	public static final int INTERVAL = 20;

	public static final int CYCLES = 1000;

	public enum Param { SPEED, STEERING }

	public ParametrizedDriveParameters()
	{
		super();
		double[] p = {35.000, 6.330, 35.000, 28.814, 35.000, 29.487, 35.000, -13.129, 35.000, -26.235, 35.000, -21.109,
				35.000, -7.564, 35.000, 23.155, 35.000, 14.831, 35.000, 25.799, 35.000, 14.541, 35.000, -0.350, 35.000,
				-28.570, 35.000, -10.988, 35.000, -12.854, 35.000, -22.894, 35.000, 22.861, 35.000, 19.961, 35.000,
				25.987, 35.000, 26.806, 35.000, 15.277, 35.000, -22.007, 35.000, -29.887, 35.000, -13.699, 35.000,
				-26.270, 35.000, -0.805, 35.000, 29.238, 35.000, 29.623, 35.000, 5.219, 35.000, -3.398, 35.000, 23.307,
				35.000, 4.468, 35.000, -18.786, 35.000, -15.556, 35.000, -25.984, 35.000, -19.782, 35.000, -28.604,
				35.000, -26.936, 35.000, -29.125, 35.000, -27.263, 35.000, -22.440, 35.000, -29.860, 35.000, -24.166,
				35.000, -21.864, 35.000, -27.388, 35.000, -23.303, 35.000, 11.115, 35.000, -22.261, 35.000, 5.575,
				35.000, -27.715};
		setParams(p);
	}

	protected void setValues()
	{
		double speed = IAudiCupMotor.DEFAULT_SPEED;
		for (int i = 0; i < CYCLES / INTERVAL; i++) {
			add(speed, 0);
		}
	}

	private void add(double speed, float angle)
	{
		int index = parameters.size() / 2;
		put(Param.SPEED.name() + index, (float) speed);
		put(Param.STEERING.name() + index, angle);
	}

	private void setParams(double[] p)
	{
		parameters.clear();
		for (int i = 0; i < p.length; i += 2) {
			put(Param.SPEED.name() + (i / 2), (float) p[i]);
			put(Param.STEERING.name() + (i / 2), (float) p[i + 1]);
		}
	}

	public float speed(int cycle)
	{
		return get(Param.SPEED.name() + cycle);
	}

	public Angle steering(int cycle)
	{
		return Angle.deg(get(Param.STEERING.name() + cycle));
	}
}
