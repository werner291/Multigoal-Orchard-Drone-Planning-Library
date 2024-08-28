// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/28/24.
//

#include "RobotPathFn.h"

namespace mgodpl {
	RobotPathFn slice(RobotPathFn path_function, double start, double end) {
		return [path_function, start, end](double t) { return path_function(start + t * (end - start)); };
	}
	RobotPathFn concat(std::initializer_list<RobotPathFn> path_functions) {
		std::vector<RobotPathFn> path_functions_vector(path_functions);
		return [path_functions_vector](double t) {
			const double segment_length = 1.0 / path_functions_vector.size();
			const size_t segment_index = std::min((size_t) (t / segment_length), path_functions_vector.size() - 1);
			const double segment_t = (t - segment_index * segment_length) / segment_length;
			return path_functions_vector[segment_index](segment_t);
		};
	}
} // namespace mgodpl
