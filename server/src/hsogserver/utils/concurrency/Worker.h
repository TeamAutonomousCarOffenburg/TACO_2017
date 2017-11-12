#pragma once

#include "Runnable.h"

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace taco
{
/**
 *
 * abstract class adds implementation of easy wait/notify mechanism
 *
 * \author Peter Walden
 */

class Worker : public Runnable
{
  public:
	Worker()
	{
		_isNotified = false;
	}

	/**
	 * \brief notify worker
	 */
	void notify()
	{
		_isNotified = true;
		_cv.notify_one();
	}

  protected:
	/**
	 * \brief wait for notification with timeout
	 * if notification appears before wait is called, wait for next notification
	 * timout is 200 ms
	 *
	 * \return bool: true if notified, false if timed out
	 */
	bool wait()
	{
		_isNotified = false;
		return waitImpl();
	}

	/**
	 * \brief wait for notification with timeout
	 * if notification appears before wait is called, do not wait and go on
	 * timout is 200 ms
	 *
	 * \return bool: true if notified, false if timed out
	 */
	bool wait2()
	{
		return waitImpl();
	}

  private:
	std::atomic<bool> _isNotified;
	std::mutex _mutex;
	std::condition_variable _cv;

	bool waitImpl()
	{
		std::unique_lock<std::mutex> lck(_mutex);
		_cv.wait_for(lck, std::chrono::milliseconds(200));
		bool returnValue = _isNotified;
		_isNotified = false;
		return returnValue;
	}
};
}
