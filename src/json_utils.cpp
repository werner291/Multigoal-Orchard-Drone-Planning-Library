
#include <random>
#include <condition_variable>
#include <filesystem>
#include "../src/json_utils.h"
#include "procedural_tree_generation.h"
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <fstream>
#include "../src/experiment_utils.h"
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <cstddef>
#include <json/json.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "LeavesCollisionChecker.h"
#include "ompl_custom.h"
#include "json_utils.h"

Json::Value jsonFromGzipFile(const std::string &path);

Json::Value toJSON(const LeafCollisions &leaf_collisions) {
    Json::Value leaf_collisions_json;
    leaf_collisions_json["t"] = leaf_collisions.t;
    leaf_collisions_json["contacts_ended"] = (int) leaf_collisions.new_contacts;
    leaf_collisions_json["new_leaves_in_contact"] = (int) leaf_collisions.removed_contacts;
    return leaf_collisions_json;
}

double trajLen(robot_trajectory::RobotTrajectory &traj) {
    double length = 0.0;

    for (std::size_t k = 1; k < traj.getWayPointCount(); ++k) {
        const auto &s1 = traj.getWayPoint(k - 1);
        const auto &s2 = traj.getWayPoint(k);

        length += s1.distance(s2);
    }

    return length;
}


Json::Value toJSON(const Eigen::Vector3d &v) {
    Json::Value json;
    json["x"] = v.x();
    json["y"] = v.y();
    json["z"] = v.z();
    return json;
}

Eigen::Vector3d fromJsonVector3d(const Json::Value &json) {
    return Eigen::Vector3d(
            json["x"].asDouble(),
            json["y"].asDouble(),
            json["z"].asDouble()
    );
}

Json::Value toJSON(const Eigen::Quaterniond &q) {
    Json::Value json;
    json["x"] = q.x();
    json["y"] = q.y();
    json["z"] = q.z();
    json["w"] = q.w();
    return json;
}

Eigen::Quaterniond fromJsonQuaternion3d(const Json::Value &json) {
    return {
            json["w"].asDouble(),
            json["x"].asDouble(),
            json["y"].asDouble(),
            json["z"].asDouble()
    };
}

Json::Value toJSON(const Eigen::Isometry3d &isom) {
    Json::Value json;
    json["translation"] = toJSON(isom.translation());
    json["orientation"] = toJSON(Eigen::Quaterniond(isom.rotation()));
    return json;
}

Eigen::Isometry3d fromJsonIsometry3d(const Json::Value &json) {

    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.translate(fromJsonVector3d(json["translation"]));
    iso.rotate(fromJsonQuaternion3d(json["orientation"]));
    return iso;
}

Json::Value toJSON(const TreeSceneData &tree_scene) {
    Json::Value output;

    for (const auto &branch: tree_scene.branches) {

        Json::Value branch_json;

        branch_json["radius"] = branch.radius;
        branch_json["length"] = branch.length;
        branch_json["root_at"] = toJSON(branch.root_at_absolute);

        output["branches"].append(branch_json);
    }

    for (const auto &vertex: tree_scene.leaf_vertices) {
        output["leaf_vertices"].append(toJSON(vertex));
    }

    for (const auto &apple: tree_scene.apples) {

        Json::Value apple_json;
        apple_json["center"] = toJSON(apple.center);
        apple_json["branch_normal"] = toJSON(apple.branch_normal);
        output["apples"].append(apple_json);

    }

    return output;
}

std::optional<TreeSceneData> treeSceneFromJson(const Json::Value &json) {

    TreeSceneData scene_data;

    scene_data.branches.reserve(json["branches"].size());

    for (const auto &branch: json["branches"]) {
        scene_data.branches.push_back({
                                              .root_at_absolute = fromJsonIsometry3d(branch["root_at"]),
                                              .length = branch["length"].asDouble(),
                                              .radius = branch["radius"].asDouble(),
                                      });
    }

    scene_data.leaf_vertices.reserve(json["leaf_vertices"].size());

    for (const auto &vertex: json["leaf_vertices"]) {
        scene_data.leaf_vertices.push_back(fromJsonVector3d(vertex));
    }

    scene_data.apples.reserve(json["apples"].size());

    for (const auto &apple: json["apples"]) {

        scene_data.apples.push_back(
                {
                        .center = fromJsonVector3d(apple["center"]),
                        .branch_normal = fromJsonVector3d(apple["branch_normal"]),
                }
        );

    }

    return scene_data;

}

Json::Value jsonFromGzipFile(const std::string &path) {

    std::ifstream file(path, std::ios_base::out | std::ios_base::binary);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::gzip_decompressor());
    inbuf.push(file);
    std::istream in(&inbuf);

    Json::Value trees_json;
    in >> trees_json;

    boost::iostreams::close(in);
    file.close();

    return trees_json;
}

void jsonToGzipFile(const Json::Value &all_trees, const std::string &path) {
    std::ofstream file(path, std::ios_base::out | std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
    outbuf.push(boost::iostreams::gzip_compressor());
    outbuf.push(file);
    std::ostream out(&outbuf);
    out << all_trees;
    boost::iostreams::close(outbuf);
    file.close();
}

Json::Value loadJsonFromFile(const std::string &path) {
    Json::Value stats;

    {
        std::ifstream statfile(path);
        if (statfile.is_open()) {
            statfile >> stats;
            statfile.close();
        }
    }
    return stats;
}

std::vector<std::vector<PtpSpec>> ptpSpecsFromJson(const moveit::core::RobotModelPtr &drone, const Json::Value &stats) {
    std::vector<std::vector<PtpSpec>> ptp_specs;

    for (const auto &ptp: stats) {
        ptp_specs.emplace_back(/* empty vector */);

        for (const auto &pair: ptp) {
            moveit::core::RobotState start_state(drone);
            for (Json::ArrayIndex vidx = 0; vidx < pair["start_state"].size(); vidx++) {
                start_state.setVariablePosition((int) vidx, pair["start_state"][vidx].asDouble());
            }
            ptp_specs.back().push_back({
                                               start_state, (size_t) pair["from_goal"].asInt(),
                                               (size_t) pair["to_goal"].asInt()
                                       });
        }
    }

    return ptp_specs;
}

Json::Value ptpSpecsToJson(const std::vector<std::vector<PtpSpec>> &specs) {

    Json::Value specs_json;

    for (const auto &ptp: specs) {
        Json::Value ptp_json;

        for (const auto &pair: ptp) {
            Json::Value pair_json;
            pair_json["from_goal"] = (int) pair.from_goal_idx;
            pair_json["to_goal"] = (int) pair.goal_idx;
            for (size_t var_i = 0; var_i < pair.start_state.getVariableCount(); var_i++) {
                pair_json["start_state"][(Json::ArrayIndex) var_i] = pair.start_state.getVariablePosition((int) var_i);
            }
            ptp_json.append(pair_json);
        }

        specs_json.append(ptp_json);
    }

    return specs_json;
}

Json::Value toJSON(const double& d) {
    return Json::Value(d);
}


template<typename T>
Json::Value vectorToJSON(const std::vector<T> v) {
    Json::Value json;

    for (const auto& value: v) {
        json.append(toJSON(value));
    }

    return json;

}

Json::Value toJSON(const ompl::base::ScopedStatePtr& state) {
    return vectorToJSON(state->reals());
}