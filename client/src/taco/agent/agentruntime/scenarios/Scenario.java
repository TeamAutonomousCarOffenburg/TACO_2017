package taco.agent.agentruntime.scenarios;

import java.util.function.Supplier;

import taco.agent.model.worldmodel.DriveInstruction;

public enum Scenario {
	NONE(NoneScenario::new),
	DRIVE_STRAIGHT(DriveStraightScenario::new),
	DRIVE_SLALOM(DriveSlalomScenario::new),
	STOP_AT_HIGH_SPEED(StopAtHighSpeedScenario::new),
	DRIVE_WAYPOINTS(DriveWaypointsScenario::new),
	DRIVE_WAYPOINTS_2(DriveWaypointsScenario2::new),
	DRIVE_WAYPOINTS_AI(DriveWaypointsAIScenario::new),
	DRIVE_WAYPOINTS_LONG(DriveWaypointsLongScenario::new),
	CROSS_PARKING(CrossParkingScenario::new),
	OVERTAKE_STRAIGHT(OvertakeStraightScenario::new),
	PULL_OUT_LEFT(() -> new PullOutScenario(DriveInstruction.PULL_OUT_LEFT)),
	PULL_OUT_RIGHT(() -> new PullOutScenario(DriveInstruction.PULL_OUT_RIGHT)),
	RIGHT_OF_WAY(RightOfWayScenario::new),
	PARAMETRIZED_DRIVE(ParametrizedDriveScenario::new),
	FOLLOW_OUTER_LANE(() -> new FollowLaneScenario(1)),
	FOLLOW_INNER_LANE(() -> new FollowLaneScenario(3)),
	FOLLOW_LANE_LEARNING(() -> new FollowLaneLearningScenario(5)),
	AADC_2015(AADC2015Scenario::new),
	AADC_2017_QUALI(AADC2017QualiScenario::new),
	AADC_2017_FINAL(AADC2017FinalScenario::new);

	public static final Scenario DEFAULT = DRIVE_WAYPOINTS;

	private final Supplier<IScenario> constructor;

	Scenario(Supplier<IScenario> constructor)
	{
		this.constructor = constructor;
	}

	public IScenario construct()
	{
		IScenario scenario = constructor.get();
		scenario.setName(name()); // ugly hack...
		return scenario;
	}
}
