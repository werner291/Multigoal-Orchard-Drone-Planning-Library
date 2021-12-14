
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

#endif //NEW_PLANNERS_GENERAL_UTLITIES_CPP
