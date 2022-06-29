//
// Created by werner on 28-09-21.
//

#ifndef NEW_PLANNERS_JSON_UTILS_H
#define NEW_PLANNERS_JSON_UTILS_H

#include "procedural_tree_generation.h"
#include "LeavesCollisionChecker.h"
#include "ompl_custom.h"
#include "experiment_utils.h"

Json::Value toJSON(const LeafCollisions &leaf_collisions);

Json::Value toJSON(const Eigen::Vector3d &v);

Eigen::Vector3d fromJsonVector3d(const Json::Value &json);

Json::Value toJSON(const Eigen::Quaterniond &q);

Eigen::Quaterniond fromJsonQuaternion3d(const Json::Value &json);

Json::Value toJSON(const Eigen::Isometry3d &isom);

Eigen::Isometry3d fromJsonIsometry3d(const Json::Value &json);

Json::Value toJSON(const TreeSceneData &tree_scene);

std::optional<TreeSceneData> treeSceneFromJson(const Json::Value &json);

Json::Value jsonFromGzipFile(const std::string &path);

void jsonToGzipFile(const Json::Value &all_trees, const std::string &path);

Json::Value loadJsonFromFile(const std::string &path);

std::vector<std::vector<PtpSpec>> ptpSpecsFromJson(const moveit::core::RobotModelPtr &drone,
                                                   const Json::Value &stats);

Json::Value ptpSpecsToJson(const std::vector<std::vector<PtpSpec>> &specs);

#endif //NEW_PLANNERS_JSON_UTILS_H
