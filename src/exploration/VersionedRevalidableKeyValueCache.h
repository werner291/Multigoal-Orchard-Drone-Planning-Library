// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_VERSIONEDREVALIDABLEKEYVALUECACHE_H
#define NEW_PLANNERS_VERSIONEDREVALIDABLEKEYVALUECACHE_H

#include "CandidatePathGenerator.h"
#include "../utilities/mesh_utils.h"
#include "../utilities/trajectory_primitives.h"
#include "DynamicMeshHullAlgorithm.h"
#include <ompl/util/RandomNumbers.h>

/**
 * A cache that stores the results of a function call, and only re-runs the function if the input has changed based on come versioning scheme.
 * Additionally, the cache can be re-validated with a revalidation function that is run on the cached value, to avoid expensive full recomputation.
 *
 * @tparam Key 			The type of the key used to identify the input to the function
 * @tparam Value 		The type of the value returned by the function
 * @tparam Version 		The type of the versioning information
 */
template<typename Key, typename Value, typename Version, typename Hash = std::hash<Key>>
class VersionedRevalidableKeyValueCache {

	/// The cache of values
	std::unordered_map<Key, std::pair<Value, Version>, Hash> cache;

public:

	using NewValueFn = std::function<Value(const Key &)>;
	using RevalidateFn = std::function<bool(const Key &, const Value &)>;

	/**
	 * Get the value associated with the given key, or compute it if it is not in the cache or if the version has changed.
	 *
	 * If recomputed, the value is stored in the cache.
	 *
	 * @param key 			The key to look up
	 * @param version 		The version; if this is different from the version of the cached value, the function will be run
	 * @param compute 		The function to compute the value if it is not in the cache or if the version has changed.
	 * 						The second argument is an optional containing the old cached value, if any.
	 * @return 				The value, either from the cache or computed by the function
	 */
	const Value &get_revalidate_or_compute(const Key &key,
										   const Version &version,
										   const RevalidateFn &revalidate,
										   const NewValueFn &compute) {

		// Look up the key in the cache
		auto it = cache.find(key);

		// If the key is not in the cache, or the version has changed, compute the value
		if (it == cache.end() || it->second.second != version || !revalidate(key, it->second.first)) {
			it = cache.insert({key, {compute(key), version}}).first;
		}

		// Return the value
		return it->second.first;

	}

	/**
	 * @brief Explicitly delete a key from the cache; it will be recomputed the next time it is requested.
	 */
	void invalidate(const Key &key) {
		cache.erase(key);
	}

};

#endif //NEW_PLANNERS_VERSIONEDREVALIDABLEKEYVALUECACHE_H
