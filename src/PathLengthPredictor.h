//
// Created by werner on 25-3-23.
//

#ifndef NEW_PLANNERS_PATHLENGTHPREDICTOR_H
#define NEW_PLANNERS_PATHLENGTHPREDICTOR_H

#include <geometric_shapes/bodies.h>

#include <utility>
#include <json/value.h>
#include "procedural_tree_generation.h"
#include "planning_scene_diff_message.h"
#include "utilities/enclosing_sphere.h"
#include "shell_space/CuttingPlaneConvexHullShell.h"
#include "shell_space/CGALMeshShell.h"
#include "shell_space/DendriticConvexHullShell.h"

/**
 * @class PathLengthPredictor
 * @brief A virtual base class for predicting the path length between two points.
 */
class PathLengthPredictor {
public:
	/// Destructor
	virtual ~PathLengthPredictor() = default;

	/**
	 * @brief Predict the path length between two points.
	 * @param point1 The first Apple object.
	 * @param point2 The second Apple object.
	 * @return The predicted path length as a double.
	 */
	[[nodiscard]] virtual double predict_path_length(const Apple &point1, const Apple &point2) = 0;
};

/**
 * @class EuclideanDistancePredictor
 * @brief A class to predict the Euclidean distance between two points.
 */
class EuclideanDistancePredictor : public PathLengthPredictor {
public:
	/// Default constructor
	EuclideanDistancePredictor() = default;

	/// @inheritdoc
	[[nodiscard]] double predict_path_length(const Apple &point1, const Apple &point2) override;
};

/**
 * @class GreatCircleDistancePredictor
 * @brief A class to predict the great-circle distance between two points.
 */
class GreatCircleDistancePredictor : public PathLengthPredictor {
	bodies::BoundingSphere enclosing_sphere;
public:
	/**
	 * @brief Get the enclosing sphere.
	 * @return A const reference to the enclosing sphere.
	 */
	[[nodiscard]] const bodies::BoundingSphere &getEnclosingSphere() const;

	bool include_approaches;
public:
	/**
	 * @brief Construct the GreatCircleDistancePredictor with a given enclosing sphere.
	 * @param enclosing_sphere The enclosing sphere.
	 */
	explicit GreatCircleDistancePredictor(bodies::BoundingSphere enclosing_sphere, bool includeApproaches);

	/// @inheritdoc
	double predict_path_length(const Apple &point1, const Apple &point2) override;

	/**
	 * @brief Create a GreatCircleDistancePredictor around the leaves of the given scene_info.
	 * @param scene_info The AppleTreePlanningScene object.
	 * @return A GreatCircleDistancePredictor object.
	 */
	[[maybe_unused]] static GreatCircleDistancePredictor
	mec_around_leaves(const AppleTreePlanningScene &scene_info, bool b);
};

/**
 * @class CuttingPlaneConvexHullDistancePredictor
 * @brief A class to predict the path length between two points using the cutting plane convex hull method.
 */
class CuttingPlaneConvexHullDistancePredictor : public PathLengthPredictor {
	CuttingPlaneConvexHullShell enclosing_shell;
public:
	/**
	 * @brief Construct the CuttingPlaneConvexHullDistancePredictor with a given enclosing shell.
	 * @param enclosingShell The enclosing shell.
	 */
	explicit CuttingPlaneConvexHullDistancePredictor(const CuttingPlaneConvexHullShell &enclosingShell,
													 bool includeApproaches);

	bool include_approaches;
public:
	/// @inheritdoc
	double predict_path_length(const Apple &point1, const Apple &point2) override;

	/**
	 * @brief Create a CuttingPlaneConvexHullDistancePredictor around the leaves of the given scene_info.
	 * @param scene_info The AppleTreePlanningScene object.
	 * @return A CuttingPlaneConvexHullDistancePredictor object.
	 */
	static CuttingPlaneConvexHullDistancePredictor around_leaves(const AppleTreePlanningScene &scene_info, bool include_approaches);
};

/**
 * @class CGALConvexHullDistancePredictor
 * @brief A class to predict the path length between two points using the CGAL convex hull method.
 */
class CGALConvexHullDistancePredictor : public PathLengthPredictor {
	CGALMeshShell enclosing_shell;

public:

    explicit CGALConvexHullDistancePredictor(const AppleTreePlanningScene &scene_info, bool includeApproaches);

	/// @inheritdoc
	double predict_path_length(const Apple &point1, const Apple &point2) override;

	bool include_approaches;
};

/**
 * @class DendriticConvexHullDistancePredictor
 * @brief A class to predict the path length between two points using the dendritic convex hull method.
 */
class DendriticConvexHullDistancePredictor : public PathLengthPredictor {

	dendritic_convex_hull::Delaunay dt;

	std::vector<std::shared_ptr<dendritic_convex_hull::DendriteNode>> dendrites;

	// The CGAL mesh (a halfedge datastructure) for topology-aware shortest-paths computation.
	Triangle_mesh tmesh;

	/// An AABB-tree for quick lookup of the on_which_mesh point on the mesh (including facet information)
	CGAL::AABB_tree<AABBTraits> tree {};

public:
	/**
	 * @brief Construct the DendriticConvexHullDistancePredictor with a given mesh.
	 * @param mesh The shape_msgs::msg::Mesh object representing the geometry.
	 */
	explicit DendriticConvexHullDistancePredictor(const shape_msgs::msg::Mesh &mesh);

	explicit DendriticConvexHullDistancePredictor(const AppleTreePlanningScene &scene_info);

	/// @inheritdoc
	double predict_path_length(const Apple &point1, const Apple &point2) override;

};

class HelicalDistancePredictor : public PathLengthPredictor {

	Eigen::Vector2d center;
public:
	HelicalDistancePredictor(const Eigen::Vector2d &center, double radius, bool includeApproaches);

private:
	double radius;

public:
	double predict_path_length(const Apple &point1, const Apple &point2) override;

	bool include_approaches;

	static HelicalDistancePredictor around_leaves(const AppleTreePlanningScene &scene, bool b);
};

std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(EuclideanDistancePredictor predictor);
std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(GreatCircleDistancePredictor predictor);
std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(HelicalDistancePredictor predictor);
std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(CuttingPlaneConvexHullDistancePredictor predictor);
std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(std::shared_ptr<DendriticConvexHullDistancePredictor> predictor);
std::pair<Json::Value, std::shared_ptr<PathLengthPredictor>> pairWithJson(std::shared_ptr<CGALConvexHullDistancePredictor> predictor);

#endif //NEW_PLANNERS_PATHLENGTHPREDICTOR_H
