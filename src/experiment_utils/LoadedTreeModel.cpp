// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#include "LoadedTreeModel.h"

namespace mgodpl {
	namespace experiments {
		std::shared_ptr<const LoadedTreeModel> TreeModelCache::obtain_by_name(const std::string &name) {
			std::lock_guard<std::mutex> lock(mutex);
			auto it = cache.find(name);
			if (it != cache.end()) {
				return it->second;
			}
			auto model = std::make_shared<LoadedTreeModel>(LoadedTreeModel::from_name(name));
			cache.insert({name, model});
			return model;
		}
	} // mgodpl
} // experiments