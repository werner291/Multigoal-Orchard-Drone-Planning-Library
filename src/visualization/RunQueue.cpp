// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-8-24.
//

#include "RunQueue.h"

void mgodpl::visualization::RunQueue::enqueue(std::function<void()> f) {
	std::lock_guard<std::mutex> lock(mutex); // Lock the mutex to ensure thread safety
	queue.push_back(f); // Add the function to the queue
}

void mgodpl::visualization::RunQueue::run_all() {
	std::lock_guard<std::mutex> lock(mutex); // Lock the mutex to ensure thread safety
	for (const auto &f: queue) { // For each function in the queue
		f(); // Execute the function
	}
	queue.clear(); // Clear the queue
}
