// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-8-24.
//

#ifndef MGODPL_RUNQUEUE_H
#define MGODPL_RUNQUEUE_H

#include <mutex>
#include <vector>
#include <functional>

namespace mgodpl {
	class SimpleVtkViewer;
}

namespace mgodpl::visualization {

	/**
	 * A class that maintains a queue of functions to be executed in a separate thread.
	 */
	class RunQueue {
	private:
		// Mutex for thread-safe operations on the queue
		std::mutex mutex;
		// Queue of functions to be executed
		std::vector<std::function<void(SimpleVtkViewer & viewer)>> queue;

	public:
		/**
		 * Enqueue a function to be executed.
		 * @param f Function to be added to the queue.
		 */
		void enqueue(std::function<void(SimpleVtkViewer & viewer)> f);

		/**
		 * Execute all functions in the queue and then clear the queue.
		 */
		void run_all(SimpleVtkViewer &viewer);
	};
}

#endif //MGODPL_RUNQUEUE_H
