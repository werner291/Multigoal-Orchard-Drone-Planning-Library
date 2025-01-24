module;

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <atomic>
#include <functional>
#include <optional>

#include "Throttle.h"
#include "RunQueue.h"

export module visualization.ThrottledRunQueue;

export namespace mgodpl::visualization {
	struct ThrottledRunQueue {
		visualization::Throttle throttle;
		visualization::RunQueue run_queue;

		template<typename R>
		R run_main(const std::function<R(SimpleVtkViewer &)> &f) {
			std::mutex mutex;
			std::optional<R> result;
			run_queue.enqueue([&](SimpleVtkViewer &viewer) {
				std::lock_guard lock(mutex);
				result = f(viewer);
			});

			while (true) {
				std::lock_guard lock(mutex);
				if (result.has_value()) {
					return std::move(result.value());
				}
			}
		}


		void run_main_void(const std::function<void(SimpleVtkViewer &)> &f) {
			std::atomic_bool has_run = false;
			run_queue.enqueue([&](mgodpl::SimpleVtkViewer &viewer) {
				f(viewer);
				has_run = true;
			});

			while (!has_run) {
				throttle.wait_and_advance(1);
			}
		}

		void wait(int n) {
			throttle.wait_and_advance(n);
		}
	};
}


