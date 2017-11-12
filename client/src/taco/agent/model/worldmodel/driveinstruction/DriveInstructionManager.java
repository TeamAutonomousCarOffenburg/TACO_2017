package taco.agent.model.worldmodel.driveinstruction;

import taco.agent.communication.perception.impl.ManeuverPerceptor;
import taco.agent.model.worldmodel.DriveInstruction;
import taco.agent.model.worldmodel.impl.Maneuver;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class DriveInstructionManager
{
	public static class Builder
	{
		private List<Maneuver> instructions = new ArrayList<>();

		public Builder add(DriveInstruction instruction, int instructionSubID, int sector)
		{
			instructions.add(new Maneuver(instruction, instructionSubID, sector, instructions.size()));
			return this;
		}

		public Builder add(DriveInstruction instruction, int sector)
		{
			instructions.add(new Maneuver(instruction, sector, instructions.size()));
			return this;
		}

		public Builder add(DriveInstruction instruction)
		{
			instructions.add(new Maneuver(instruction, 0, instructions.size()));
			return this;
		}

		public DriveInstructionManager build()
		{
			return new DriveInstructionManager(instructions);
		}
	}

	private static final Maneuver DUMMY_MANEUVER = new Maneuver(DriveInstruction.STRAIGHT, 0, 0);

	/** The current instruction index. */
	private int currentInstructionIndex;

	/** The start instruction index (by jury module) */
	private int startInstructionIndex;

	/** The list of drive instructions. */
	private List<Maneuver> instructions;

	private DriveInstructionManager(List<Maneuver> instructions)
	{
		this.instructions = instructions;
		currentInstructionIndex = 0;
		startInstructionIndex = 0;
	}

	public int getCurrentInstructionIndex()
	{
		return currentInstructionIndex;
	}

	public int getNumberOfInstructions()
	{
		return instructions.size();
	}

	public DriveInstruction getInstruction(int index)
	{
		return getManeuver(index).getDriveInstruction();
	}

	public DriveInstruction getPreviousInstruction()
	{
		return getInstruction(currentInstructionIndex - 1);
	}

	public DriveInstruction getCurrentInstruction()
	{
		return getInstruction(currentInstructionIndex);
	}

	public Maneuver getManeuver(int index)
	{
		if (index < 0 || index >= instructions.size()) {
			return DUMMY_MANEUVER;
		}

		return instructions.get(index);
	}

	public List<Maneuver> getManeuvers()
	{
		return Collections.unmodifiableList(instructions);
	}

	public Maneuver getPreviousManeuver()
	{
		return getManeuver(currentInstructionIndex - 1);
	}

	public Maneuver getCurrentManeuver()
	{
		return getManeuver(currentInstructionIndex);
	}

	/**
	 * @return true if the last instruction was performed
	 */
	public boolean progressInstructionIndex()
	{
		currentInstructionIndex++;
		return currentInstructionIndex >= getNumberOfInstructions();
	}

	public void setStartInstructionIndex(int startIndex)
	{
		startInstructionIndex = startIndex;
		currentInstructionIndex = startIndex;
	}

	public void setDriveInstructions(List<ManeuverPerceptor> driveInstructions)
	{
		instructions = new ArrayList<>();
		for (ManeuverPerceptor current : driveInstructions) {
			instructions.add(new Maneuver(current));
		}
		currentInstructionIndex = 0;
		startInstructionIndex = 0;
	}

	public int getSectorIndex(int instructionIndex)
	{
		if (instructions.isEmpty() || instructionIndex < 0 || instructionIndex >= instructions.size()) {
			System.out.format("getSectorIndex(): index %d is out of bounds (got %d instructions)\n", instructionIndex,
					instructions.size());
			return 0;
		}

		return instructions.get(instructionIndex).getSector();
	}

	public int getCurrentSectorIndex()
	{
		return getSectorIndex(currentInstructionIndex);
	}
}
