#pragma once

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>

/**
 * Fifo
 * Queue blocks reading thread if empty. If full oldest data will be overridden while pushing
 *
 * \author Peter Walden
 *
 */

template <class T, std::size_t N> class BlockingQueue
{
  public:
	typedef boost::shared_ptr<BlockingQueue<T, N>> Ptr;

	typedef T valueType;
	BlockingQueue() : _queue(N)
	{
		_canceled = false;
	}

	bool full()
	{
		std::lock_guard<std::mutex> guard(_mutex);
		return _queue.full();
	}

	bool empty()
	{
		std::lock_guard<std::mutex> guard(_mutex);
		return _queue.empty();
	}

	long size()
	{
		std::lock_guard<std::mutex> guard(_mutex);
		return _queue.size();
	}

	T at(int i)
	{
		std::lock_guard<std::mutex> guard(_mutex);
		return _queue.at(i);
	}

	/**
	 * set queue state to active and empty queue
	 *
	 */
	void clear()
	{
		std::lock_guard<std::mutex> guard(_mutex);
		_queue.clear();
		_canceled = false;
	}

	/**
	 * release all waiting threads and set queue to inactive
	 *
	 */
	void cancel()
	{
		_cv.notify_all();
		std::lock_guard<std::mutex> guard(_mutex);
		_canceled = true;
		_queue.push_back(valueType());
	}

	/**
	 * non blocking push. If full oldest element will be overwritten
	 *
	 */
	void push(valueType data)
	{
		std::lock_guard<std::mutex> guard(_mutex);
		if (!_canceled) {
			_queue.push_back(data);
			_cv.notify_one();
		}
	}

	/**
	 * blocks if queue is empty
	 *\return oldest element or nullptr in case of cancel
	 */
	valueType pop()
	{
		if (_canceled)
			return nullptr;

		std::unique_lock<std::mutex> lk(_mutex);
		_cv.wait(lk, [=] { return (!_queue.empty()); });
		if (!_canceled) {
			valueType result = _queue.front();
			_queue.pop_front();

			return result;
		}

		else
			return nullptr;
	}

	/**
	 * non blocking
	 * \return vector with elements. Empty vector if queue is empty
	 */
	std::vector<valueType> popAll()
	{
		std::vector<valueType> result;
		if (!_canceled) {
			std::lock_guard<std::mutex> guard(_mutex);

			while (!_queue.empty()) {
				result.push_back(_queue.front());
				_queue.pop_front();
			}
		}
		return result;
	}

  private:
	boost::circular_buffer<valueType> _queue;
	std::mutex _mutex;
	std::condition_variable _cv;
	std::atomic<bool> _canceled;
};
