//
// Created by werner on 9-3-23.
//

#ifndef NEW_PLANNERS_RUNPLANNERTHREADED_H
#define NEW_PLANNERS_RUNPLANNERTHREADED_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include "DynamicGoalVisitationEvaluation.h"

/**
 * @brief A threaded class for running a planner and computing trajectories on the fly.
 *
 * This class allows a planner to be run on a separate thread while trajectories are
 * computed and made available on a separate thread-safe queue.
 */
class RunPlannerThreaded {

	mutable std::mutex mq_mutex; ///< Mutex for protecting access to thread_segments_requested.
	size_t thread_segments_requested = 0; ///< The number of segments that have been requested but not yet computed.
	std::condition_variable message_queue_cv; ///< Condition variable for waiting until segments are requested.

public:
	struct PlannerUpdate {
		robot_trajectory::RobotTrajectory traj;
		std::vector<utilities::DiscoveryStatus> status;
	};

private:
	std::mutex traj_mutex; ///< Mutex for protecting access to the trajectory queue.
	std::queue<PlannerUpdate> trajectory_queue; ///< The queue of computed trajectories.
	std::condition_variable trajectory_queue_cv; ///< Condition variable for waiting until a trajectory is available.

	bool is_done = false; ///< Whether the thread should exit.

	DynamicGoalVisitationEvaluation eval; ///< The planner evaluation object.

	std::thread thread_; ///< The thread on which the planner is run.

	/**
	 * @brief The main thread function for running the planner and computing trajectories.
	 *
	 * This function runs in a separate thread and is responsible for requesting trajectory
	 * segments from the planner, computing them, and adding them to the trajectory queue.
	 */
	void run();

public:

	/**
	 * @brief Starts the planner thread.
	 *
	 * This function creates a new thread and calls the run() function on it.
	 */
	void start();

	/**
	 * @brief Constructs a new RunPlannerThreaded object.
	 *
	 * @param eval The planner evaluation object.
	 * @param run_to_completion Whether to run the planner until completion.
	 */
	explicit RunPlannerThreaded(DynamicGoalVisitationEvaluation eval, bool run_to_completion);

	/**
	 * @brief Requests the next trajectory segment to be computed.
	 *
	 * This function increments thread_segments_requested and notifies the message_queue_cv
	 * condition variable to wake up the planner thread.
	 */
	void request_next();

	/**
	 * @brief Polls the next available trajectory from the queue.
	 *
	 * This function pops the next trajectory off the queue and returns it as an optional.
	 * If the queue is empty, it returns std::nullopt.
	 *
	 * @return The next available trajectory, or std::nullopt if the queue is empty.
	 */
	std::optional<PlannerUpdate> poll_trajectory_update();

	~RunPlannerThreaded();

	void finish();

	size_t n_segments_requested() const;

};

#endif //NEW_PLANNERS_RUNPLANNERTHREADED_H
