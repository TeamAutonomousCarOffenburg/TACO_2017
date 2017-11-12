package taco.util.drive;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.IPose2D;

public interface IBehaviorGeometry {
	IPose2D getClosestPose(Vector2D virtualPoint);
	IPose2D getClosestPose(Vector3D virtualPoint);
}
