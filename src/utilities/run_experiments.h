
#ifndef NEW_PLANNERS_RUN_EXPERIMENTS_H
#define NEW_PLANNERS_RUN_EXPERIMENTS_H

#include <vector>
#include <json/json.h>
#include <functional>
#include <random>
#include <iostream>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include "json_utils.h"
#include "JsonChunkedLogger.h"

/**
 * Run a series of experiments in parallel, and put the results in a JSON file.
 *
 * Will write the results a file progressively; in case of a crash,
 * the results will be available, and the experiment can be resumed.
 *
 * Experiments are shuffled in a deterministic manner, erasing possible issues with
 * the order of experiments and sligtly differing system load when they are run, to
 * make the results more reproducible and fairer.
 *
 * @tparam Parameters 	The type of the experiment parameters.
 * @param parameters 	A vector of experiment parameters to run.
 * @param experiment 	A function that takes an experiment parameter and returns a JSON value as the result of the experiment.
 * @param results_file 	The name/path of the file to save the results to.
 * @param chunk_size 	The number of experiments to complete in between file writes.
 * @param parallelism 	The number of parallel threads to use.
 * @param shuffle_seed 	A seed for the random number generator used to shuffle the experiment parameters.
 */
template<typename Parameters>
void runExperimentsParallelRecoverable(const std::vector<Parameters>& parameters,
									   const std::function<Json::Value(const Parameters &)> &experiment,
									   const std::string& results_file,
									   size_t chunk_size = 8,
									   size_t parallelism = 8,
									   int shuffle_seed = 0) {

	std::vector<const Parameters*> parameters_pointers;
	for (auto& p : parameters) {
		parameters_pointers.push_back(&p);
	}

	// Shuffle the experiment parameters in a deterministic way using a fixed seed.
	std::shuffle(parameters_pointers.begin(), parameters_pointers.end(), std::default_random_engine(shuffle_seed));

	// Initialize the results logger, and the mutex to access it.
	std::mutex records_mutex;
	JsonChunkedLogger logger(chunk_size, results_file);

	// Create a thread pool with the specified number of threads.
	boost::asio::thread_pool pool(parallelism);

	// Run over all the experiments (parameters) and either run them or skip them if they have already been run.
	for (size_t experiment_i = 0; experiment_i < parameters_pointers.size(); experiment_i += 1) {

		Json::Value params_json = toJSON(*parameters_pointers[experiment_i]);

		std::lock_guard<std::mutex> lock(records_mutex);

		// Check if the experiment has already been run.
		if (logger.hasExperimentBeenDone((int) experiment_i, params_json)) {
			std::cout << "Skipping experiment " << experiment_i << " because it has already been run." << std::endl;
			continue;
		} else {

			// Add the experiment to the thread pool.
			boost::asio::post(pool, [experiment_i=experiment_i,&parameters_pointers,&records_mutex,&experiment, &logger]() {

				try {
					// Run the experiment once it's our turn.
					std::cout << "Running experiment " << experiment_i << std::endl;

					auto start_time = std::chrono::high_resolution_clock::now();
					Json::Value result = experiment(*parameters_pointers[experiment_i]);
					auto duration = std::chrono::high_resolution_clock::now() - start_time;
					int duration_millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

					result["total_experiment_runtime"] = duration_millis;

					std::cout << "Finished experiment " << (experiment_i + 1) << "/" << parameters_pointers.size()
							  << " in " << duration_millis << "ms." << std::endl;

					// Save the results to the JSON object.
					// This is protected by a mutex to prevent concurrent access, since the thread pool may run multiple experiments in parallel.
					{
						std::lock_guard<std::mutex> lock(records_mutex);
						logger.storeExperimentResult(experiment_i, toJSON(*parameters_pointers[experiment_i]), result);
					}
				} catch (std::exception &e) {
					std::cout << "Exception while running experiment " << experiment_i << ": " << e.what() << std::endl;
					{
						Json::Value result = "error: " + std::string(e.what());

						std::lock_guard<std::mutex> lock(records_mutex);
						logger.storeExperimentResult(experiment_i, toJSON(*parameters_pointers[experiment_i]), result);
					}
				}
			});
		}

	}

	// Wait for all the experiments to finish.
	pool.join();

	logger.saveResults();

	std::cout << "All experiments completed." << std::endl;

}

template<typename Planner, typename Problem>
using PlannerProblemPair = std::pair<const std::pair<Json::Value, Planner>*, const std::pair<Json::Value, Problem>*>;

template<typename Planner, typename Problem>
Json::Value toJSON(const PlannerProblemPair<Planner, Problem>& pair) {
	Json::Value result;
	result["planner"] = pair.first->first;
	result["problem"] = pair.second->first;
	return result;
}

/**
 * @brief Run a series of experiments in parallel and save the results in a JSON file.
 *
 * This function will progressively write the results to a file, so in case of a crash,
 * the results will be available, and the experiment can be resumed.
 * Experiments are shuffled in a deterministic manner, erasing possible issues with
 * the order of experiments and slightly differing system load when they are run, to
 * make the results more reproducible and fairer.
 *
 * @tparam Planner The planner type.
 * @tparam Problem The problem type.
 * @param parameters A vector of planner pairs, where each pair consists of a JSON value and a planner instance.
 * @param problems A vector of problem pairs, where each pair consists of a JSON value and a problem instance.
 * @param run_experiment A function that takes a planner and a problem and returns a JSON value as the result of the experiment.
 * @param results_file The name/path of the file to save the results to.
 * @param chunk_size The number of experiments to complete in between file writes.
 * @param parallelism The number of parallel threads to use.
 * @param shuffle_seed A seed for the random number generator used to shuffle the experiment parameters.
 */
template<typename Planner, typename Problem>
void runPlannersOnProblemsParallelRecoverable(const std::vector<std::pair<Json::Value, Planner>>& parameters,
											  const std::vector<std::pair<Json::Value, Problem>>& problems,
											  const std::function<Json::Value(const Planner &, const Problem&)> &run_experiment,
											  const std::string& results_file,
											  size_t chunk_size = 8,
											  size_t parallelism = 8,
											  int shuffle_seed = 0) {

	std::cout << "Running " << parameters.size() << " x " << problems.size() << " experiments." << std::endl;

	std::vector<PlannerProblemPair<Planner, Problem>> parameters_pointers;

	for (const auto& p : parameters) {
		for (const auto& prob : problems) {
			parameters_pointers.push_back(std::make_pair(&p, &prob));
		}
	}

	auto run_for_pair = [&run_experiment](const PlannerProblemPair<Planner, Problem>& pair) {
		return run_experiment(pair.first->second, pair.second->second);
	};

	return runExperimentsParallelRecoverable<PlannerProblemPair<Planner, Problem>>(
			parameters_pointers,
			run_for_pair,
			results_file,
			chunk_size,
			parallelism,
			shuffle_seed);
}


#endif //NEW_PLANNERS_RUN_EXPERIMENTS_H
