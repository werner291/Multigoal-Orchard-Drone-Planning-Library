
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

			// If we get to this point, the experiment has not been run yet.
			std::cout << "Submitting experiment " << experiment_i << " to the thread pool." << std::endl;

			// Add the experiment to the thread pool.
			boost::asio::post(pool, [experiment_i=experiment_i,&parameters_pointers,&records_mutex,&experiment, &logger]() {
				// Run the experiment once it's our turn.
				std::cout << "Running experiment " << experiment_i << std::endl;

				auto start_time = std::chrono::high_resolution_clock::now();
				Json::Value result = experiment(*parameters_pointers[experiment_i]);
				auto duration = std::chrono::high_resolution_clock::now() - start_time;
				int duration_millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

				result["total_experiment_runtime"] = duration_millis;

				std::cout << "Finished experiment " << experiment_i << "/" << parameters_pointers.size() << " in " << duration_millis << "ms." << std::endl;

				// Save the results to the JSON object.
				// This is protected by a mutex to prevent concurrent access, since the thread pool may run multiple experiments in parallel.
				{
					std::lock_guard<std::mutex> lock(records_mutex);
					logger.storeExperimentResult(experiment_i, toJSON(*parameters_pointers[experiment_i]), result);
				}
			});
		}

	}

	// Wait for all the experiments to finish.
	pool.join();

	logger.saveResults();

	std::cout << "All experiments completed." << std::endl;

}


#endif //NEW_PLANNERS_RUN_EXPERIMENTS_H
