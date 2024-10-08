// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#ifndef THROTTLE_H
#define THROTTLE_H

#include <atomic>
#include <condition_variable>

namespace mgodpl::visualization {
	/**
	 * This class encapsulates the logic for throttling the execution of an algorithm being visualized,
	 * for instance by calling the wait_and_advance() method before each step of the algorithm in
	 * the callback hooks of the algorithm being executed.
	 *
	 * This way, an algorithm need not be explicitly written with visualization and/or stepwise execution in mind.
	 */
	class Throttle {
		/// The condition variable used to signal the algorithm thread that it's allowed to advance.
		std::condition_variable cv;
		/// The mutex used to lock the condition variable.
		std::mutex cv_mutex;
		/// The number of steps allowed to advance.
		std::atomic_int64_t steps_allowed = 0;

	public:
		/**
		 * @brief Wait until the algorithm is allowed to advance.
		 *
		 * This method waits until the number of steps allowed is greater than zero, decrementing it when it's allowed to advance, then returns.
		 */
		void wait_and_advance(int wait_steps);

		/**
		 * @brief Allow the algorithm to advance.
		 *
		 * This method increments the number of steps allowed, then notifies the algorithm thread that it's allowed to advance.
		 */
		void allow_advance();
	};
}
#endif //THROTTLE_H
