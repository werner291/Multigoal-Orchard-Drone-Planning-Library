
#pragma once

#include "IncrementalTSPMethods.h"

namespace mgodpl {
	namespace tsp_utils {

		std::vector<NewOrderingEntry> insert_last(size_t, std::function<double(const NewOrderingEntry &,const NewOrderingEntry &)>, std::function<double(const NewOrderingEntry &)>);

		std::vector<NewOrderingEntry> insert_first(size_t, std::function<double(const NewOrderingEntry &,const NewOrderingEntry &)>, std::function<double(const NewOrderingEntry &)>);

		std::vector<NewOrderingEntry> insert_second(size_t, std::function<double(const NewOrderingEntry &,const NewOrderingEntry &)>, std::function<double(const NewOrderingEntry &)>);

		std::vector<NewOrderingEntry> insert_least_costly(size_t, std::function<double(const NewOrderingEntry &,const NewOrderingEntry &)>, std::function<double(const NewOrderingEntry &)>);

		std::vector<NewOrderingEntry> insert_random(size_t, std::function<double(const NewOrderingEntry &,const NewOrderingEntry &)>, std::function<double(const NewOrderingEntry &)>);

	}
}
