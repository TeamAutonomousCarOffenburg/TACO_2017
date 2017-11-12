package taco.agent.communication.perception;

import java.util.List;

public interface IVisionPerceptor {
	List<RecognizedObject> getRecognizedObjects();
}
