
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
 * \brief Take the generalized vector cross product of three Vector4d's. That is, a vector perpendicular to all three.
 */
Eigen::Vector4d cross_three(const Eigen::Vector4d& a, const Eigen::Vector4d& b, const Eigen::Vector4d& c);

#endif //NEW_PLANNERS_GENERAL_UTLITIES_CPP
