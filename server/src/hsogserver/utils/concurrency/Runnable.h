#pragma once

#include <atomic>
#include <iostream>
#include <string>
#include <thread>

namespace taco
{
/*
 *
 * Abstract runnable for threads
 */

class Runnable
{
  public:
	Runnable() : m_stop(), m_thread()
	{
	}
	virtual ~Runnable()
	{
		try {
			stop();
		} catch (...) {
		}
	}
	Runnable(Runnable const &) = delete;
	Runnable &operator=(Runnable const &) = delete;
	void stop()
	{
		m_stop = true;
		if (m_thread.joinable()) {
			m_thread.join();
		}
	}

	void start()
	{
		start("unnamed");
	}
	void start(std::string threadName)
	{
		m_stop = false;
		m_thread = std::thread(&Runnable::run, this);
#ifdef __linux__
		// name limit
		auto handle = m_thread.native_handle();
		threadName.resize(15);
		pthread_setname_np(handle, threadName.c_str());
#endif
	}

  protected:
	virtual void run() = 0;
	std::atomic<bool> m_stop;

  private:
	std::thread m_thread;
};
}
