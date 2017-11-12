package taco.util.rightofway;

public enum RightOfWayAction {
	DRIVE,

	STOP_THEN_DRIVE,

	/** Do not drive (wait some time / e.g. give way) */
	WAIT,

	/** Not permitted (e.g. invalid drive destination) */
	NOT_PERMITTED
}
