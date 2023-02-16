
#include "run_experiments.h"
#include "json_utils.h"
#include "JsonChunkedLogger.h"

#include <random>
#include <iostream>

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>


template<typename Parameters>
void runExperimentsParallelRecoverable(std::vector<Parameters> parameters,
									   const std::function<Json::Value(const Parameters &)> &experiment,
									   const std::string &results_file,
									   size_t chunk_size,
									   size_t parallelism,
									   int shuffle_seed) {

	// Shuffle the experiment parameters in a deterministic way using a fixed seed.
	std::shuffle(parameters.begin(), parameters.end(), std::default_random_engine(shuffle_seed));

	// Initialize the results logger, and the mutex to access it.
	std::mutex records_mutex;
	JsonChunkedLogger logger(chunk_size, results_file);

	// Create a thread pool with the specified number of threads.
	boost::asio::thread_pool pool(parallelism);

	// Run over all the experiments (parameters) and either run them or skip them if they have already been run.
	for (size_t experiment_i = 0; experiment_i < parameters.size(); experiment_i += 1) {

		Json::Value params_json = toJson(parameters[experiment_i]);

		std::lock_guard<std::mutex> lock(records_mutex);

		// Check if the experiment has already been run.
		if (logger.hasExperimentBeenDone((int) experiment_i, params_json)) {
				std::cout << "Skipping experiment " << experiment_i << " because it has already been run." << std::endl;
				continue;
		} else {

			// If we get to this point, the experiment has not been run yet.
			std::cout << "Submitting experiment " << experiment_i << " to the thread pool." << std::endl;

			// Add the experiment to the thread pool.
			boost::asio::post(pool, [&]() {
				// Run the experiment once it's our turn.
				std::cout << "Running experiment " << experiment_i << std::endl;

				auto start_time = std::chrono::high_resolution_clock::now();
				Json::Value result = experiment(parameters[experiment_i]);
				auto duration = std::chrono::high_resolution_clock::now() - start_time;
				int duration_millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

				result["total_experiment_runtime"] = duration_millis;

				std::cout << "Finished experiment " << experiment_i << "/" << parameters.size() << " in " << duration_millis << "ms." << std::endl;

				// Save the results to the JSON object.
				// This is protected by a mutex to prevent concurrent access, since the thread pool may run multiple experiments in parallel.
				{
					std::lock_guard<std::mutex> lock(records_mutex);
					logger.storeExperimentResult(experiment_i, params_json, result);
				}
			});
		}

	}

	// Wait for all the experiments to finish.
	pool.join();

	logger.saveResults();

	std::cout << "All experiments completed." << std::endl;

}