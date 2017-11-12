package taco.agent.communication.perception;

import hso.autonomy.agent.communication.perception.IPerceptor;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public interface IFloorNormalPerceptor extends IPerceptor {
	Vector3D getValue();
}
