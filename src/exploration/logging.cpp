// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 29-11-22.
//

#include "logging.h"

#include <sstream>
#include <ctime>

void write_log_csv_line(std::ofstream &log_file, const ExperimentLogPoint &log_point) {

	log_file << log_point.time << ",";

	for (size_t i = 0; i < log_point.per_fruit_scan_proportion.size(); i++) {
		log_file << log_point.per_fruit_scan_proportion[i];
		if (i != log_point.per_fruit_scan_proportion.size() - 1) {
			log_file << ",";
		}
	}

	log_file << std::endl << std::flush;

}

std::ofstream open_new_logfile(size_t num_targets) {

	std::stringstream log_file_name;
	log_file_name << "analysis/data/log_" << std::time(nullptr) << ".csv";

	std::ofstream log_file(log_file_name.str());

	log_file << "time" << std::endl;

	for (size_t i = 0; i < num_targets; i++) {
		log_file << ", fruit_" << i << "_pct";
	}

	return std::move(log_file);

}
