#ifndef NEW_PLANNERS_JSON_UTILS_H
#define NEW_PLANNERS_JSON_UTILS_H

#include "../procedural_tree_generation.h"
#include "../LeavesCollisionChecker.h"
#include "../ompl_custom.h"
#include "experiment_utils.h"

/**
 * Convert a LeafCollisions object to a JSON object
 *
 * @param leaf_collisions  LeafCollisions object
 * @return JSON object
 */
Json::Value toJSON(const LeafCollisions &leaf_collisions);

/**
 * Convert an Eigen::Vector3d to a JSON array
 *
 * @param v  Eigen::Vector3d
 * @return JSON array
 */
Json::Value toJSON(const Eigen::Vector3d &v);

/**
 * Convert a JSON array to an Eigen::Vector3d
 *
 * @param json  JSON array
 * @return Eigen::Vector3d
 */
Eigen::Vector3d fromJsonVector3d(const Json::Value &json);

/**
 * Convert an Eigen::Quaterniond to a JSON array
 *
 * @param q  Eigen::Quaterniond
 * @return JSON array
 */
Json::Value toJSON(const Eigen::Quaterniond &q);

/**
 * Convert a JSON array to an Eigen::Quaterniond
 *
 * @param json  JSON array
 * @return Eigen::Quaterniond
 */
Eigen::Quaterniond fromJsonQuaternion3d(const Json::Value &json);

/**
 * Convert an Eigen::Isometry3d to a JSON object
 *
 * @param isom  Eigen::Isometry3d
 * @return JSON object
 */
Json::Value toJSON(const Eigen::Isometry3d &isom);

/**
 * Convert a JSON object to an Eigen::Isometry3d
 *
 * @param json  JSON object
 * @return Eigen::Isometry3d
 */
Eigen::Isometry3d fromJsonIsometry3d(const Json::Value &json);

/**
 * Convert a TreeSceneData object to a JSON object
 *
 * @param tree_scene  TreeSceneData object
 * @return JSON object
 */
Json::Value toJSON(const TreeSceneData &tree_scene);

/**
 * Convert a JSON object to a TreeSceneData object
 *
 * @param json  JSON object
 * @return TreeSceneData object
 */
std::optional<TreeSceneData> treeSceneFromJson(const Json::Value &json);

/**
 * Load a JSON object from a gzip-compressed file
 *
 * @param path  Path to the file
 * @return JSON object
 */
Json::Value jsonFromGzipFile(const std::string &path);

/**
 * Save a JSON object to a gzip-compressed file
 *
 * @param all_trees  JSON object
 * @param path  Path to the file
 */
void jsonToGzipFile(const Json::Value &all_trees, const std::string &path);

/**
 * Load a JSON object from a file
 *
 * @param path  Path to the file
 * @return JSON object
 */
Json::Value loadJsonFromFile(const std::string &path);

/**
 * Convert a JSON array to a vector of PtpSpec vectors
 *
 * @param drone  Robot model
 * @param stats  JSON array
 * @return Vector of PtpSpec vectors
 */
std::vector<std::vector<PtpSpec>> ptpSpecsFromJson(const moveit::core::RobotModelPtr &drone,
                                                   const Json::Value &stats);

/**
 * Convert a vector of PtpSpec vectors to a JSON array
 *
 * @param specs  Vector of PtpSpec vectors
 * @return JSON array
 */
Json::Value ptpSpecsToJson(const std::vector<std::vector<PtpSpec>> &specs);

/**
 * Convert a Sphere struct to a JSON object
 *
 * @param sphere  Sphere struct
 * @return JSON object
 */
Json::Value toJSON(const Sphere &sphere);

/**
 * Convert a JSON object to a Sphere struct
 *
 * @param json  JSON object
 * @return Sphere struct
 */
std::optional<Sphere> sphereFromJSON(const Json::Value &json);

#endif //NEW_PLANNERS_JSON_UTILS_H
