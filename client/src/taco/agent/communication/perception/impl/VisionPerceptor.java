package taco.agent.communication.perception.impl;

import taco.agent.communication.perception.IVisionPerceptor;
import taco.agent.communication.perception.PerceptorName;
import taco.agent.communication.perception.RecognizedObject;

import java.util.List;

public class VisionPerceptor extends AudiCupPerceptor implements IVisionPerceptor
{
	private List<RecognizedObject> recognizedObjects;

	public VisionPerceptor(long timestamp, List<RecognizedObject> recognizedObjects)
	{
		super(PerceptorName.VISION, timestamp);
		this.recognizedObjects = recognizedObjects;
	}

	public List<RecognizedObject> getRecognizedObjects()
	{
		return recognizedObjects;
	}
}
