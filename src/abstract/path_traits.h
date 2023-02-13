
#ifndef NEW_PLANNERS_PATH_TRAITS_H
#define NEW_PLANNERS_PATH_TRAITS_H

#include <vector>

/**
 * @brief A traits class for paths that defines operations that can be performed on them.
 *
 * @tparam PathType 	The type of the path.
 */
template <typename PathType>
struct path_traits {

	/**
	 * @brief The type of state used in the path.
	 */
	using state_type = typename PathType::state_type;

	/**
	 * @brief Concatenates two paths into one path.
	 *
	 * @param path1 	The first path.
	 * @param path2 	The second path.
	 *
	 * @return The concatenated path.
	 */
	static PathType concatenate_path(const PathType &path1, const PathType &path2);

	/**
	 * @brief Returns the initial state of the path.
	 *
	 * @param path 	The path.
	 *
	 * @return The initial state of the path.
	 */
	static state_type initial_state(const PathType &path);

	/**
	 * @brief Returns the final state of the path.
	 *
	 * @param path 	The path.
	 *
	 * @return The final state of the path.
	 */
	static state_type final_state(const PathType &path);

};

#endif //NEW_PLANNERS_PATH_TRAITS_H
