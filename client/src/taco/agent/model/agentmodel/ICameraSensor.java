package taco.agent.model.agentmodel;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import hso.autonomy.util.geometry.Area2D;

/**
 * Representation of a camera.
 */
public interface ICameraSensor extends IAudiCupSensor {
	Vector3D pixelToCar(Vector2D position);

	Vector2D carToPixel(Vector2D position);

	Vector2D carToPixel(Vector3D position);

	Area2D.Float pixelToCar(Area2D.Int area);
}