// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_PATH_TRAITS_H
#define NEW_PLANNERS_PATH_TRAITS_H

/**
 * @brief A traits class for paths that defines operations that can be performed on them.
 *
 * @tparam Path 	The type of the path.
 */
template<typename Path>
struct path_traits {

	/**
	 * @brief The type of state used in the path.
	 */
	using state_type = typename Path::state_type;

	/**
	 * @brief Concatenates two paths into one path.
	 *
	 * @param path1 	The first path.
	 * @param path2 	The second path.
	 *
	 * @return The concatenated path.
	 */
	static Path concatenate_path(const Path &path1, const Path &path2);

	/**
	 * @brief Returns the initial state of the path.
	 *
	 * @param path 	The path.
	 *
	 * @return The initial state of the path.
	 */
	static state_type initial_state(const Path &path);

	/**
	 * @brief Returns the final state of the path.
	 *
	 * @param path 	The path.
	 *
	 * @return The final state of the path.
	 */
	static state_type final_state(const Path &path);

};

#endif //NEW_PLANNERS_PATH_TRAITS_H
