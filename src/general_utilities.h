
#ifndef NEW_PLANNERS_GENERAL_UTILITIES_CPP
#define NEW_PLANNERS_GENERAL_UTILITIES_CPP

#include <vector>
#include <ompl/base/ScopedState.h>
#include <variant>
#include <boost/range/adaptors.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <Eigen/Geometry>
#include <shape_msgs/Mesh.h>

template<typename T>
std::vector<size_t> index_vector(const std::vector<T> &v) {

    std::vector<size_t> ids(v.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        ids[i] = i;
    }

    return ids;

}

Eigen::Vector4d any_perpendicular_of_two(const Eigen::Vector4d& a, const Eigen::Vector4d& b);


/**
 * Take the generalized vector cross product of three Vector4d's. That is, a vector perpendicular to all three.
 */
Eigen::Vector4d cross_three(const Eigen::Vector4d& a, const Eigen::Vector4d& b, const Eigen::Vector4d& c);

/**
 *
 * Given two quaternions `qa` and `qb` and a positive scalar `max_distance`, produce a third quaternion `qs` such that:
 *
 *  `acos(qa.dot(qs)) + acos(qb.dot(qs)) <= max_distance`
 *
 * Note: `qa` and `qb` must be unit quaternions, such that `acos(qa.dot(qb)) <= max_distance`.
 */
Eigen::Quaterniond sampleInformedQuaternion(const Eigen::Quaterniond& qa,
                                            const Eigen::Quaterniond& qb,
                                            const double max_distance);

/**
 * The slerp distance between two quaternions, defined as acos(|qs1⋅qs2|), in radians.
 */
double quat_dist(const Eigen::Quaterniond& qs1, const Eigen::Quaterniond& qs2);

template<typename V> void truncate(std::vector<V>& v, size_t n) {
    if (v.size() > n) {v.resize(n);}
}

/**
 * Recursively generate all permutations of all non-empty subsets of the provided vector of items,
 * while providing a bit of machinery to compute a left-fold of every permutation as well.
 *
 * @tparam T                The type of item in the vector of items to be permuted.
 * @tparam V                The output of the left-fold operation.
 * @param visitable         The vector of items to be permuted.
 * @param empty_value       The value of the left-fold of an empty sequence.
 * @param consider_cb       A callback, provided with the begin/end (non-inclusive) of the permuted version
 *                          of `visitable`, and the left-fold of all elements in the provided range,
 *                          EXCLUDING the last element.
 *
 *                          To return: the fold value of the full range.
 *
 * @param elements_fixed    The length of the prefix of `visitable` to keep fixed; set to zero (default option)
 *                          to generate all permutations of all non-empty subsets.
 */
template<typename T, typename V>
void generate_combinations
        (std::vector<T> visitable,
         const V empty_value,
         const std::function<V(std::vector<size_t>::const_iterator first,
                               std::vector<size_t>::const_iterator last,
                               const V &)> &consider_cb,
         size_t elements_fixed = 0) {

    // Iterate over every element beyond the range of fixed elements.
    // This intentionally includes the first element in that range.
    for (size_t swap_with = elements_fixed; swap_with < visitable.size(); ++swap_with) {
        // Swap the first element in the variable range with the pointed-to element.
        // Both indices may be euqal, in which case the swap is a no-op.
        std::swap(visitable[elements_fixed], visitable[swap_with]);

        // Call the callback with the fixed range extended by 1.
        V value = consider_cb(visitable.begin(), visitable.begin() + elements_fixed + 1, empty_value);

        // Recurse, also with the extended fixed range.
        generate_combinations(
                visitable, value, consider_cb, elements_fixed + 1
        );

        // Undo the swap to ensure predictable behavior to bring te vector back to what it was.
        std::swap(visitable[elements_fixed], visitable[swap_with]);
    }
}

/**
 * Discovers the connected components of the mesh.
 * Output is a vector of vectors of vertex indices.
 * Each vector is a connected components.
 */
std::vector<std::vector<size_t>> connected_vertex_components(const shape_msgs::Mesh& mesh);

/**
 * Perform a convex decomposition of the given mesh using the HACD library.
 *
 * @param concavity Lower values give more accurate results, at the cost of higher number of parts.
 */
std::vector<shape_msgs::Mesh> convex_decomposition(const shape_msgs::Mesh &mesh, const double concavity = 2.0);

template<typename V>
std::vector<V> vectorByOrdering(const std::vector<V> &original, const std::vector<size_t> &ordering) {
    std::vector<V> result;
    result.reserve(ordering.size());
    for (const size_t i : ordering) result.push_back(original[i]);
    return result;
}

#endif //NEW_PLANNERS_GENERAL_UTILITIES_CPP
