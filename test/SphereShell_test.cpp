
#include <gtest/gtest.h>
#include "../src/shell_space/SphereShell.h"
#include "../src/utilities/msgs_utilities.h"

std::vector<Eigen::Vector3d> extractLeafVerticesFromPlanningSceneMessage(const AppleTreePlanningScene& scene) {
	std::vector<Eigen::Vector3d> points;

	for (const auto& col : scene.scene_msg.world.collision_objects) {
		if (col.id == "leaves") {
			for (const auto& mesh : col.meshes) {
				for (auto v : mesh.vertices) {
					points.push_back(toEigen(v));
				}
			}
		}
	}

	return points;
}

TEST(SphereShellTest, test) {

	const double padding = 0.5;

	AppleTreePlanningScene scene = createMeshBasedAppleTreePlanningSceneMessage("appletree", true);

	auto shell = paddedSphericalShellAroundLeaves(scene, padding);

	// Ensure it's the minimum enclosing sphere. That is:
	// 		All leaf vertices must lie inside.
	// 		At least 2 leaf vertices must lie on the surface.

	std::vector<Eigen::Vector3d> leafVertices = extractLeafVerticesFromPlanningSceneMessage(scene);

	size_t on_shell = 0;

	for (const auto& vtx : leafVertices) {

		double distance = (vtx - shell->getCenter()).norm();

		EXPECT_LE(distance, shell->getRadius());

		if (std::abs(distance + padding - shell->getRadius()) < 1e-6) {
			on_shell++;
		}
	}

	EXPECT_GE(on_shell, 2);

}