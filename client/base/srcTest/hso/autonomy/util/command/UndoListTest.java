/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */

package hso.autonomy.util.command;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

/**
 *
 * @author Klaus Dorer
 */
public class UndoListTest
{
	private UndoList testee;

	private TestCommand command1;

	private ICommand command2;

	private ICommand command3;

	@Before
	public void setUp()
	{
		testee = new UndoList(2);
		command1 = new TestCommand("command1");
		command2 = new TestCommand("command2");
		command3 = new TestCommand("command3");
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.command.UndoList#addCommand(hso.autonomy.util.command.ICommand)}
	 * .
	 */
	@Test
	public void testAddCommand()
	{
		assertEquals(0, testee.size());
		assertEquals(-1, testee.getUndoIndex());

		testee.addCommand(command1);
		assertEquals(1, testee.size());
		assertEquals(0, testee.getUndoIndex());

		testee.addCommand(command2);
		assertEquals(2, testee.size());

		testee.addCommand(command3);
		assertEquals(2, testee.size());
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.command.UndoList#addCommand(hso.autonomy.util.command.ICommand)}
	 * .
	 */
	@Test
	public void testAddMergeableCommand()
	{
		command1.setMergeable(true);
		testee.addCommand(command1);
		assertEquals(1, testee.size());

		testee.addCommand(command2);
		assertEquals(1, testee.size());
		assertEquals(0, testee.getUndoIndex());
		assertEquals(command1, testee.getUndoCommand());
	}

	/**
	 * Test method for {@link hso.autonomy.util.command.UndoList#addCommand(ICommand)}
	 */
	@Test
	public void testAddCommandRemovesLaterInList()
	{
		testee.addCommand(command1);
		testee.addCommand(command2);
		assertEquals(command2, testee.getUndoCommand());
		assertEquals(command1, testee.getUndoCommand());

		testee.addCommand(command3);
		assertEquals(1, testee.size());
		assertEquals(0, testee.getUndoIndex());
	}

	/**
	 * Test method for {@link hso.autonomy.util.command.UndoList#getUndoCommand()}.
	 */
	@Test
	public void testGetUndoCommand()
	{
		assertEquals(null, testee.getUndoCommand());
		assertEquals(null, testee.getUndoCommand());

		testee.addCommand(command1);
		assertEquals(command1, testee.getUndoCommand());

		testee.addCommand(command1);
		testee.addCommand(command2);
		assertEquals(command2, testee.getUndoCommand());
		assertEquals(command1, testee.getUndoCommand());

		testee.addCommand(command1);
		testee.addCommand(command2);
		testee.addCommand(command3);
		assertEquals(command3, testee.getUndoCommand());
		assertEquals(command2, testee.getUndoCommand());
		assertEquals(null, testee.getUndoCommand());
	}

	/**
	 * Test method for {@link hso.autonomy.util.command.UndoList#getRedoCommand()}.
	 */
	@Test
	public void testGetRedoCommand()
	{
		assertEquals(null, testee.getRedoCommand());
		assertEquals(null, testee.getRedoCommand());

		testee.addCommand(command1);
		testee.addCommand(command2);
		assertEquals(null, testee.getRedoCommand());
		assertEquals(command2, testee.getUndoCommand());
		assertEquals(command2, testee.getRedoCommand());

		assertEquals(command2, testee.getUndoCommand());
		assertEquals(command1, testee.getUndoCommand());

		ICommand redo = testee.getRedoCommand();
		assertEquals(command1, redo);
		assertEquals("command1", redo.getName());
		assertEquals(true, redo.perform());

		redo = testee.getRedoCommand();
		assertEquals(command2, redo);
		assertEquals("command2", redo.getName());
		assertEquals(true, redo.perform());

		assertEquals(null, testee.getRedoCommand());
	}
}
