/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.util.timing;

/**
 *
 * @author kdorer
 */
public class AlarmTimer implements Runnable
{
	private long timeout;

	private boolean wokeUp;

	private ITriggerReceiver sleeper;

	private String name;

	public AlarmTimer(String name, ITriggerReceiver sleeper, long timeout)
	{
		this.name = name;
		this.sleeper = sleeper;
		this.timeout = timeout;
		wokeUp = false;
		Thread t = new Thread(this, name);
		t.setDaemon(true);
		t.start();
	}

	public synchronized void stopAlarm()
	{
		wokeUp = true;
		notify();
	}

	@Override
	public synchronized void run()
	{
		final long alarmTime = System.currentTimeMillis() + timeout;
		long currentTime;
		while (!wokeUp && alarmTime > (currentTime = System.currentTimeMillis())) {
			try {
				wait(alarmTime - currentTime);
			} catch (InterruptedException e) {
			}
		}
		if (!wokeUp) {
			// inform the caller
			sleeper.trigger(AlarmTimer.this.name);
		}
	}

	@Override
	public String toString()
	{
		return "Alarm: " + name + " timeout: " + timeout;
	}
}