
#ifndef NEW_PLANNERS_JSONCHUNKEDLOGGER_H
#define NEW_PLANNERS_JSONCHUNKEDLOGGER_H

#include <json/json.h>
#include <fstream>
#include <iostream>

/**
 * @brief A logger that stores experiment results in a JSON file.
 *
 * Experiments can be added to this object one-by-one, and the results are stored in a JSON file. The file is stored
 * in chunks, so that the results are not lost in case of a crash. The file is updated after a specified number of
 * completions to reduce file writes.
 *
 * Also, this class can be used to check if an experiment has already been run, and skip it if it has,
 * in case the experiment crashed or was otherwise interrupted before it could finish.
 *
 * This class can be used to store experiment results as JSON objects. It stores the results in a file,
 * which is updated after a specified number of completions to reduce memory usage. The results file
 * path is specified in the constructor.
 *
 * Note: this class is not thread-safe and should be accessed via a mutex if multiple experiments are run in parallel.
 */
class JsonChunkedLogger {

	Json::Value records; //< The JSON object that stores the results.
	size_t chunk_size; //< The number of experiments to complete before updating the results file.
	size_t completions = 0; //< The number of experiments that have been completed so far during this run.
	const std::string& results_file_path; //< The path to the results file.

public:
	/**
	 * @brief Construct a new Json Chunked Logger object.
	 *
	 * @param chunk_size      The number of experiments to complete before updating the results file.
	 * @param resultsFilePath The path to the results file.
	 *
	 * If a file already exists at the specified path, it will be loaded into memory,
	 * such that it can be used to check if an experiment has already been run,
	 * and completed with new results instead of starting from scratch.
	 */
	JsonChunkedLogger(size_t chunk_size, const std::string &resultsFilePath)
			: chunk_size(chunk_size), results_file_path(resultsFilePath) {
	}

	/**
	 * Check if an experiment has already been run.
	 *
	 * Throws a runtime error if the experiment has been run, but the parameters do not match,
	 * indcating that the results may have been corrupted.
	 *
	 * @param experiment_i  The index of the experiment.
	 * @param parameters 	The parameters of the experiment.
	 * @return 				True if the experiment has already been run, false otherwise.
	 */
	bool hasExperimentBeenDone(int experiment_i, const Json::Value& parameters) {
		Json::Value &record = records[experiment_i];

		if (record.isObject()) {
			if (record["parameters"] == parameters) {
				// The experiment has already been run, so skip it.
				return true;
			} else {
				// The experiment has supposedly been run, but the parameters do not match.
				// Results may have been corrupted, so throw an error.
				throw std::runtime_error("Experiment parameters do not match with cached experiment.");
			}
		}

		return false;
	}

	/**
	 * @brief Save the results to the results file.
	 */
	void saveResults() const {
		std::cout << "Saving results to file." << std::endl;
		std::ofstream file("results.json");
		file << records;
	}

	/**
	 * @brief Store the result of an experiment, and save the results to the results file if the chunk size has been reached.
	 *
	 * @param experiment_i 		The index of the experiment.
	 * @param parameters 		The parameters of the experiment.
	 * @param result 			The result of the experiment.
	 */
	void storeExperimentResult(int experiment_i, const Json::Value& parameters, const Json::Value& result) {
		Json::Value &record = records[experiment_i];
		record["parameters"] = parameters;
		record["result"] = result;

		completions += 1;
		if (completions % chunk_size == 0) {
			saveResults();
		}
	}

};

#endif //NEW_PLANNERS_JSONCHUNKEDLOGGER_H
