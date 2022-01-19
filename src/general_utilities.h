
#ifndef NEW_PLANNERS_GENERAL_UTLITIES_CPP
#define NEW_PLANNERS_GENERAL_UTLITIES_CPP

#include <vector>

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

#endif //NEW_PLANNERS_GENERAL_UTLITIES_CPP
