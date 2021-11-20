#include <gtest/gtest.h>
#include "../src/experiment_utils.h"
#include "../src/json_utils.h"

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