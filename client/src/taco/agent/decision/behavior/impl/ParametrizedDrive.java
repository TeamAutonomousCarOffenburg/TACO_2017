package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import kdo.util.parameter.ParameterMap;
import taco.agent.decision.behavior.IBehaviorConstants;
import taco.agent.decision.behavior.base.AudiCupBehavior;

public class ParametrizedDrive extends AudiCupBehavior
{
	private final ParametrizedDriveParameters params;

	private int cycles;

	public ParametrizedDrive(IThoughtModel thoughtModel, ParameterMap params)
	{
		super(IBehaviorConstants.PARAMETRIZED_DRIVE, thoughtModel);
		this.params = (ParametrizedDriveParameters) params.get(IBehaviorConstants.PARAMETRIZED_DRIVE);
	}

	@Override
	public void init()
	{
		super.init();
		cycles = 0;
	}

	@Override
	public void perform()
	{
		cycles++;

		if (cycles % ParametrizedDriveParameters.INTERVAL == 0) {
			int index = cycles / ParametrizedDriveParameters.INTERVAL;
			getAgentModel().getMotor().drive(params.speed(index));
			getAgentModel().getSteering().steer(params.steering(index));
		}
	}
}
