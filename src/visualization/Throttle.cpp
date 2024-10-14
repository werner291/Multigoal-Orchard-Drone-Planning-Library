// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/19/24.
//

#include "Throttle.h"

void mgodpl::visualization::Throttle::wait_and_advance(int wait_steps) {
	// Lock the mutex.
	std::unique_lock<std::mutex> lock(cv_mutex);

	while (wait_steps-- > 0) {
		// Wait until we're allowed to advance.
		cv.wait(lock, [&]() { return steps_allowed > 0; });

		// Decrement the steps allowed.
		--steps_allowed;
	}
}

void mgodpl::visualization::Throttle::allow_advance() {
	// Lock the mutex.
	std::unique_lock<std::mutex> lock(cv_mutex);

	// Increment the steps allowed.
	++steps_allowed;

	// Notify the algorithm thread that it's allowed to advance.
	cv.notify_one();
}
