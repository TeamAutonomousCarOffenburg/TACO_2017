package taco.agent.model.worldmodel.driveinstruction;

import taco.agent.communication.perception.IJuryPerceptor;
import taco.agent.communication.perception.JuryAction;

public class JuryActionManager
{
	private int maneuverID;

	private int oldManeuverID;

	private JuryAction action;

	public JuryActionManager()
	{
		oldManeuverID = 0;
		maneuverID = 0;
		action = JuryAction.STOP;
	}

	public boolean update(IJuryPerceptor juryPerceptor)
	{
		boolean changed = juryPerceptor.getManeuverId() != maneuverID || juryPerceptor.getAction() != action;

		maneuverID = juryPerceptor.getManeuverId();
		action = juryPerceptor.getAction();

		return changed;
	}

	public int getManeuverId()
	{
		return maneuverID;
	}

	public JuryAction getAction()
	{
		return action;
	}

	public void setAction(JuryAction action)
	{
		this.action = action;
	}

	public boolean hasManeuverIDChanged()
	{
		return oldManeuverID != maneuverID;
	}
}
