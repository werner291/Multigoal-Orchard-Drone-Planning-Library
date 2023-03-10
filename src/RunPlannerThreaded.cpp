//
// Created by werner on 9-3-23.
//

#include "RunPlannerThreaded.h"

void RunPlannerThreaded::start() {
	thread_ = std::thread(&RunPlannerThreaded::run, this);
}

RunPlannerThreaded::RunPlannerThreaded(DynamicGoalVisitationEvaluation eval, bool run_to_completion) : eval(std::move(eval)) {
	if (run_to_completion) {
		thread_segments_requested = SIZE_MAX;
	}
}

void RunPlannerThreaded::request_next() {
	{
		std::unique_lock<std::mutex> lock(mq_mutex);
		thread_segments_requested++;
	}
	message_queue_cv.notify_one();
}

std::optional<RunPlannerThreaded::PlannerUpdate> RunPlannerThreaded::poll_trajectory_update() {
	std::unique_lock<std::mutex> lock(traj_mutex);

	if (trajectory_queue.empty()) {
		return std::nullopt;
	} else {
		auto traj = trajectory_queue.front();
		trajectory_queue.pop();
		return traj;
	}
}

void RunPlannerThreaded::run() {

	while (!is_done) {
		// Wait for at least one to be requested.
		if (thread_segments_requested == 0) {
			std::unique_lock<std::mutex> lock(mq_mutex);
			message_queue_cv.wait(lock);
		}

		if (thread_segments_requested == 0) {
			continue;
		}

		// Compute the next trajectory.
		auto traj = eval.computeNextTrajectory();

		if (traj) {

			std::unique_lock<std::mutex> lock(traj_mutex);

			trajectory_queue.emplace(PlannerUpdate{*traj, eval.getDiscoveryStatus()});

			trajectory_queue_cv.notify_one();

		} else {
			is_done = true;
		}

		// Decrease the number of requested segments.
		{
			std::unique_lock<std::mutex> lock(mq_mutex);
			thread_segments_requested--;
		}
	}
}

RunPlannerThreaded::~RunPlannerThreaded() {
	if (thread_.joinable()) {
		finish();
	}
}

void RunPlannerThreaded::finish() {
	is_done = true;
	thread_segments_requested = 0;
	message_queue_cv.notify_all();
	thread_.join();
}

size_t RunPlannerThreaded::n_segments_requested() const {
	std::unique_lock<std::mutex> lock(mq_mutex);

	return thread_segments_requested;
}
