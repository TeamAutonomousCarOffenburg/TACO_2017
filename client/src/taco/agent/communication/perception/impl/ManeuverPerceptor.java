package taco.agent.communication.perception.impl;

public class ManeuverPerceptor
{
	private String driveInstruction;

	private int sector;

	private int maneuverId;

	public ManeuverPerceptor(String driveInstruction, int sector, int maneuverId)
	{
		this.driveInstruction = driveInstruction;
		this.sector = sector;
		this.maneuverId = maneuverId;
	}

	public String getDriveInstruction()
	{
		return driveInstruction;
	}

	public int getSector()
	{
		return sector;
	}

	public int getManeuverId()
	{
		return maneuverId;
	}

	@Override
	public String toString()
	{
		return "\nManeuver-ID: " + maneuverId + " , Sector: " + sector + " , Instruction: " + driveInstruction;
	}
}
