#include <gtest/gtest.h>
#include "../src/utilities/experiment_utils.h"
#include "../src/utilities/json_utils.h"

TEST(JsonTest, tree_scene_test) {

    auto tree_scene = generateTreeScene(50);

    auto after_json = *treeSceneFromJson(toJSON(tree_scene));

    assert(tree_scene.branches.size() == after_json.branches.size());
    assert(tree_scene.leaf_vertices.size() == after_json.leaf_vertices.size());
    assert(tree_scene.apples.size() == after_json.apples.size());

    for (size_t i = 0; i < tree_scene.branches.size(); i++) {
        assert((tree_scene.branches[i].root_at_absolute.translation() -
                after_json.branches[i].root_at_absolute.translation()).norm() < 1.0e-10);
        assert(abs(tree_scene.branches[i].radius - after_json.branches[i].radius) < 1.0e-10);
        assert(abs(tree_scene.branches[i].length - after_json.branches[i].length) < 1.0e-10);
    }

    for (size_t i = 0; i < tree_scene.leaf_vertices.size(); i++) {
        assert((tree_scene.leaf_vertices[i] - after_json.leaf_vertices[i]).norm() < 1.0e-10);
    }

    for (size_t i = 0; i < tree_scene.apples.size(); i++) {
        assert((tree_scene.apples[i].center - after_json.apples[i].center).norm() < 1.0e-10);
        assert((tree_scene.apples[i].branch_normal - after_json.apples[i].branch_normal).norm() < 1.0e-10);
    }

}

TEST(JsonTest, ptp_pairs_test) {

    // Get the drone model from URDF and SRDF
    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");

    // Load the scene data
    const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");

    // Limit ourselves to a subset of all trees.
    const size_t MAX_TREES = 50;

    std::random_device dev;
    std::mt19937 gen(dev());

    auto generated_specs = genPointToPointSpecs(drone, trees_data, gen, 10);

    assert(generated_specs.size() == trees_data.size());

    auto after_json = ptpSpecsFromJson(drone, ptpSpecsToJson(generated_specs));

    assert(generated_specs.size() == after_json.size());

    for (size_t i = 0; i < generated_specs.size(); ++i) {
        assert(generated_specs[i].size() == after_json[i].size());

        for (size_t j = 0; j < generated_specs[i].size(); ++j) {

            assert(generated_specs[i][j].goal_idx == after_json[i][j].goal_idx);
            assert(generated_specs[i][j].from_goal_idx == after_json[i][j].from_goal_idx);

            for (size_t var_i = 0; var_i < drone->getVariableCount(); ++var_i) {
                assert(generated_specs[i][j].start_state.getVariablePosition(var_i) ==
                       after_json[i][j].start_state.getVariablePosition(var_i));
            }
        }
    }

}