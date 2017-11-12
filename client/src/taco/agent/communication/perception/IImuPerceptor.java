package taco.agent.communication.perception;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public interface IImuPerceptor extends IAudiCupPerceptor {
	Rotation getGyro();

	Vector3D getAcceleration();
}