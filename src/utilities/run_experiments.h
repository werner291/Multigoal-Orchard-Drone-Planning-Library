
#ifndef NEW_PLANNERS_RUN_EXPERIMENTS_H
#define NEW_PLANNERS_RUN_EXPERIMENTS_H

#include <vector>
#include <json/json.h>
#include <functional>

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
void runExperimentsParallelRecoverable(std::vector<Parameters> parameters,
									   const std::function<Json::Value(const Parameters &)> &experiment,
									   const std::string& results_file,
									   size_t chunk_size = 8,
									   size_t parallelism = 8,
									   int shuffle_seed = 0);


#endif //NEW_PLANNERS_RUN_EXPERIMENTS_H
