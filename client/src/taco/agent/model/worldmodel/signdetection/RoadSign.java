package taco.agent.model.worldmodel.signdetection;

import hso.autonomy.util.geometry.IPose2D;
import taco.agent.model.worldmodel.objectdetection.DetectedObject;
import taco.util.SignType;

public class RoadSign extends DetectedObject
{
	/** allowed time to detect a sign when it's inside the visible area*/
	private static final int TTL = 2;

	private SignType sign;

	private float lastUpdateTime;

	/** true if roadsign should be visible at the current cycle*/
	private boolean visible;

	/** true if we really have detected the roadsign*/
	private boolean seen;

	public RoadSign(SignType type, IPose2D pose)
	{
		super(type.toString(), pose);
		sign = type;
		lastUpdateTime = 0;
		visible = false;
		seen = false;
	}

	public SignType getSignType()
	{
		// set sign to unknown if we detect something as a sign we could not define
		if (sign == null) {
			sign = SignType.UNKNOWN;
		}
		return sign;
	}

	public float getLastUpdateTime()
	{
		return lastUpdateTime;
	}

	public boolean isVisible()
	{
		return visible;
	}

	public boolean hasSeen()
	{
		return seen;
	}

	@Override
	public void update(IPose2D pose)
	{
		//		this.pose = Pose2D.average(pose, pose, seenCount, 1);
	}

	public void update(boolean visible, boolean seen, float updateTime)
	{
		// only update time if state hsa changed
		if (visible != this.visible || this.seen != seen) {
			this.lastUpdateTime = updateTime;
		}
		this.seen = seen;
		this.visible = visible;
	}

	/**
	 * Checks if this sign has been removed from the map (e.g. by the jury).
	 * Condition for this is when the sign is longer than the TTL inside the visible area but it still
	 * was not detected
	 */
	public boolean hasBeenRemoved(float globalTime)
	{
		return isVisible() && !hasSeen() && (globalTime - getLastUpdateTime() > TTL);
	}

	@Override
	public String toString()
	{
		return "\nsign=" + sign + ", pose=" + pose + " , isVisible= " + visible + " , hasSeen= " + seen +
				" , lastUpdateTime= " + lastUpdateTime;
	}
}
