// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_LOGGING_H
#define NEW_PLANNERS_LOGGING_H

#include <vector>
#include <fstream>

struct ExperimentLogPoint {

	double time;

	std::vector<double> per_fruit_scan_proportion;

};

std::ofstream open_new_logfile(size_t num_targets);

void write_log_csv_line(std::ofstream &log_file, const ExperimentLogPoint &log_point);

#endif //NEW_PLANNERS_LOGGING_H
