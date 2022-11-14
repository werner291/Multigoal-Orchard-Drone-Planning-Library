
#ifndef NEW_PLANNERS_STREAMINGMESHHULLALGORITHM_H
#define NEW_PLANNERS_STREAMINGMESHHULLALGORITHM_H

#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>

class StreamingMeshHullAlgorithm {

public:
	virtual bool addPoint(const Eigen::Vector3d &point) = 0;

	[[nodiscard]] virtual shape_msgs::msg::Mesh toMesh() const = 0;

};

#endif //NEW_PLANNERS_STREAMINGMESHHULLALGORITHM_H
