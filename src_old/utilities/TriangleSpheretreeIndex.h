//
// Created by werner on 10-8-22.
//

#ifndef NEW_PLANNERS_TRIANGLESPHERETREEINDEX_H
#define NEW_PLANNERS_TRIANGLESPHERETREEINDEX_H


#include <Eigen/Core>

class TriangleSpatialIndex {

	void insert(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, size_t triangle_key);

};


#endif //NEW_PLANNERS_TRIANGLESPHERETREEINDEX_H
