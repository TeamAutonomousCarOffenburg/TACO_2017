package taco.agent.decision.behavior.impl;

import hso.autonomy.agent.model.thoughtmodel.IThoughtModel;
import hso.autonomy.util.geometry.IPose2D;
import taco.agent.decision.behavior.base.AudiCupBehavior;
import taco.agent.model.agentmodel.IAudiCupAgentModel;
import taco.agent.model.agentmodel.impl.enums.LightName;
import taco.agent.model.worldmodel.ParkingSpaceState;
import taco.agent.model.worldmodel.impl.ParkingSpace;

public abstract class PullOutBase extends AudiCupBehavior
{
	protected enum Phase { STARTING, FORWARD, TURN, FINISHED }

	protected Phase phase;

	protected IPose2D startPose;

	protected ParkingSpace closestParkingSpace;

	public PullOutBase(String name, IThoughtModel thoughtModel)
	{
		super(name, thoughtModel);
	}

	@Override
	public boolean isFinished()
	{
		return phase == Phase.FINISHED;
	}

	@Override
	public void init()
	{
		super.init();
		phase = Phase.STARTING;
		startPose = null;
		closestParkingSpace = null;
	}

	@Override
	public void perform()
	{
		IPose2D currentCarPose = getWorldModel().getThisCar().getPose();
		IAudiCupAgentModel agentModel = getAgentModel();

		switch (phase) {
		case STARTING:
			starting(currentCarPose, agentModel);
			closestParkingSpace = getWorldModel().getEnvironmentManager().getClosestParkingSpace(currentCarPose);
			break;

		case FORWARD:
			forward(currentCarPose, agentModel, 0.25);
			break;

		case TURN:
			turn(currentCarPose, agentModel);

			if (closestParkingSpace != null) {
				getWorldModel().getEnvironmentManager().updateParkingSpace(
						closestParkingSpace.getID(), ParkingSpaceState.FREE, getWorldModel().getGlobalTime());
			}
			break;
		}
	}

	protected void starting(IPose2D currentCarPose, IAudiCupAgentModel agentModel)
	{
		startPose = currentCarPose;
		agentModel.getLight(LightName.WARN).turnOff();
		phase = Phase.FORWARD;
	}

	protected void forward(IPose2D currentCarPose, IAudiCupAgentModel agentModel, double length)
	{
		double distance = currentCarPose.getDistanceTo(startPose);
		if (distance > length) {
			phase = Phase.TURN;
		}
		agentModel.getSteering().reset();
		agentModel.getMotor().driveForward();
	}

	protected abstract void turn(IPose2D currentCarPose, IAudiCupAgentModel agentModel);
}
