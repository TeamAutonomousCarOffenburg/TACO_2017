/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.observer;

import static org.junit.Assert.assertFalse;
import static org.mockito.Mockito.mock;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

import hso.autonomy.util.observer.IObserver;
import hso.autonomy.util.observer.IPublishSubscribe;
import hso.autonomy.util.observer.Subject;

/**
 * Tests for Observer pattern Subject class
 */
public class SubjectTest
{
	private IPublishSubscribe<String> testee;

	private IObserver<String> observerMock1;

	private IObserver<String> observerMock2;

	@SuppressWarnings("unchecked")
	@Before
	public void setUp()
	{
		testee = new Subject<>();
		observerMock1 = mock(IObserver.class);
		observerMock2 = mock(IObserver.class);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.observer.Subject#attach(hso.autonomy.util.observer.IObserver)}.
	 */
	@Test
	public void testAttach()
	{
		String testString = "test";
		observerMock1.update(testString);
		testee.attach(observerMock1);
		testee.onStateChange(testString);
		Mockito.verify(observerMock1, Mockito.times(2)).update(testString);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.observer.Subject#attach(hso.autonomy.util.observer.IObserver)}.
	 */
	@Test
	public void testAttachNoDuplicates()
	{
		String testString = "test";
		observerMock1.update(testString);
		testee.attach(observerMock1);
		testee.attach(observerMock1);
		testee.onStateChange(testString);
		Mockito.verify(observerMock1, Mockito.times(2)).update(testString);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.observer.Subject#attach(hso.autonomy.util.observer.IObserver)}.
	 */
	@Test
	public void testAttachTwo()
	{
		String testString = "test";
		observerMock1.update(testString);
		observerMock2.update(testString);
		testee.attach(observerMock1);
		testee.attach(observerMock2);
		testee.onStateChange(testString);
		Mockito.verify(observerMock1, Mockito.times(2)).update(testString);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.observer.Subject#detach(hso.autonomy.util.observer.IObserver)}.
	 */
	@Test
	public void testDetach()
	{
		String testString = "test";
		observerMock1.update(testString);

		testee.attach(observerMock1);
		testee.attach(observerMock2);
		testee.detach(observerMock2);
		testee.onStateChange(testString);

		testee.detach(observerMock1);
		testee.onStateChange(testString);

		assertFalse(testee.detach(observerMock1));
		Mockito.verify(observerMock1, Mockito.times(2)).update(testString);
	}

	/**
	 * Test method for
	 * {@link hso.autonomy.util.observer.Subject#detach(hso.autonomy.util.observer.IObserver)}.
	 */
	@Test
	public void testDetachAll()
	{
		String testString = "test";
		observerMock1.update(testString);
		observerMock2.update(testString);

		testee.attach(observerMock1);
		testee.attach(observerMock2);
		testee.onStateChange(testString);

		testee.detachAll();
		testee.onStateChange(testString);

		assertFalse(testee.detach(observerMock1));
		assertFalse(testee.detach(observerMock2));
		Mockito.verify(observerMock1, Mockito.times(2)).update(testString);
		Mockito.verify(observerMock2, Mockito.times(2)).update(testString);
	}
}
