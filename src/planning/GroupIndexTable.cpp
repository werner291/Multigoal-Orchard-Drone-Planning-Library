// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/19/24.
//

#include "GroupIndexTable.h"

namespace mgodpl {
	GroupIndexTable::GroupIndexTable(const std::vector<size_t> &counts): index_table(counts.size()) {
		// Keep a counter for the global index.
		size_t global_index = 0;

		// Run a nested loop to fill the index table.
		for (size_t i = 0; i < counts.size(); ++i) {
			index_table[i].reserve(counts[i]);
			for (size_t j = 0; j < counts[i]; ++j) {
				index_table[i].push_back(global_index++);
			}
		}

		total_samples = global_index;
	}
} // mgodpl
