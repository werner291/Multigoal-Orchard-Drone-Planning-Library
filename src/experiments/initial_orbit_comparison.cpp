#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <utility>
#include <json/value.h>
#include "../TreeMeshes.h"
#include "../utilities/alpha_shape.h"
#include "../utilities/MeshOcclusionModel.h"
#include "../utilities/enclosing_sphere.h"
#include "../utilities/json_utils.h"
#include "../utilities/mesh_utils.h"
#include "../utilities/math_utils.h"
#include "../vtk/SimpleVtkViewer.h"
#include "../OrbitPath.h"


void visualizeAlphaShape(TreeMeshes &meshes, const shape_msgs::msg::Mesh &alphashape) {
	// Visualize the models quickly.

	SimpleVtkViewer viewer;

	//		viewer.addMesh(meshes.leaves_mesh, {0.0,1.0,0.0});

	for (const auto &mesh: meshes.fruit_meshes) {
		viewer.addMesh(mesh, {1.0, 0.0, 0.0});
	}

	viewer.addMesh(alphashape, {0.0, 0.5, 0.0});

	viewer.addMesh(meshes.trunk_mesh, {0.3, 0.2, 0.1});

	viewer.start();

	exit(1);

}

std::vector<Eigen::Vector3d> leafPointsForAlphaShape(TreeMeshes &meshes) {
	std::vector<Eigen::Vector3d> leaf_points;

	// Loop through all the vertices in the "leaves_mesh" of the input "meshes" object
	for (const auto &v: meshes.leaves_mesh.vertices) {
		Eigen::Vector3d p(v.x, v.y, v.z);
		leaf_points.push_back(p);
	}

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dist(0.0, 1.0);

	// Loop through all the triangles in the "leaves_mesh" of the input "meshes" object
	for (const auto &f: meshes.leaves_mesh.triangles) {
		// Get the vertices of the current triangle
		const auto &v0 = meshes.leaves_mesh.vertices[f.vertex_indices[0]];
		const auto &v1 = meshes.leaves_mesh.vertices[f.vertex_indices[1]];
		const auto &v2 = meshes.leaves_mesh.vertices[f.vertex_indices[2]];

		// Create Eigen::Vector3d objects representing the vertices of the current triangle
		Eigen::Vector3d a(v0.x, v0.y, v0.z);
		Eigen::Vector3d b(v1.x, v1.y, v1.z);
		Eigen::Vector3d c(v2.x, v2.y, v2.z);

		// Sample 10 points on the current triangle
		for (int i = 0; i < 10; i++) {
			// Sample two random numbers between 0 and 1
			double u = dist(gen);
			double v = dist(gen);

			u = sqrt(u);
			v = sqrt(v);

			// Use the alternative method to compute the point on the triangle
			Eigen::Vector3d p = a + u * (b - a) + v * (c - a);

			// Add the computed point to the "leaf_points" vector
			leaf_points.push_back(p);
		}
	}

	// Return the vector of leaf points
	return leaf_points;
}

int main() {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");

	std::vector<Eigen::Vector3d> apple_positions = meshes.fruit_meshes | ranges::views::transform([](const auto &mesh) {
		return Eigen::Vector3d(mesh_aabb(mesh).center());
	}) | ranges::to_vector;

	std::vector<Eigen::Vector3d> leaf_points = leafPointsForAlphaShape(meshes);

	auto alphashape = alphaShape(leaf_points, sqrt(0.00001));

	//	visualizeAlphaShape(meshes, alphashape);

	MeshOcclusionModel occlusion_model(alphashape, 0.05);

	const auto paddings = {0.0, 0.5, 1.0, 2.0};

	// Get a minimum enclosing sphere.
	auto bounding_sphere = utilities::compute_enclosing_sphere_around_points(
			meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
				return Eigen::Vector3d{v.x, v.y, v.z};
			}) | ranges::to_vector);

	Json::Value json;

	{
		std::vector<bool> apples_seen(apple_positions.size(), false);

		// Pick 10000 points at random on the sphere and check if the apples are visible from any of them.

		for (int i = 0; i < 10000; i++) {
			Eigen::Vector3d point = math_utils::sample_point_on_sphere(bounding_sphere.center, bounding_sphere.radius);

			int seen_from_point = 0;

			for (int j = 0; j < apple_positions.size(); j++) {
				if (!occlusion_model.checkOcclusion(apple_positions[j], point)) {
					apples_seen[j] = true;

					seen_from_point++;
				}
			}

			json["from_points"][i]["n_seen"] = seen_from_point;
			json["from_points"][i]["point"] = toJSON(point);
		}

		json["random_sample_visible"] = std::count(apples_seen.begin(), apples_seen.end(), true);
	}

	for (const double padding: paddings) {

		// Create a sphere with the given padding.
		Eigen::Vector3d center = bounding_sphere.center;
		double radius = bounding_sphere.radius + padding;

		auto paths = {pairWithJson(FlatOrbitPath(center, radius, 0, M_PI)),
					  pairWithJson(FlatOrbitPath(center, radius, 0, 2 * M_PI)),
					  pairWithJson(CylindricalOrbitPath(center, radius, 2 * radius, 0, 2 * M_PI)),
					  pairWithJson(SphericalOscillatingOrbitPath(center, radius, 0, 2 * M_PI)),
					  pairWithJson(HelicalOrbit(center, radius, 2 * radius, 0, 2 * M_PI))};

		for (const auto &[path_json, path]: paths) {

			std::vector<bool> apples_seen(apple_positions.size(), false);

			double length = 0.0;

			for (int i = 0; i < 10000; ++i) {

				double t = i / 10000.0;

				Eigen::Vector3d position = path->at_t(t);

				for (int j = 0; j < apple_positions.size(); ++j) {
					if (!occlusion_model.checkOcclusion(apple_positions[j], position)) {
						apples_seen[j] = true;
					}
				}

				double t_next = (i + 1) / 10000.0;
				length += (path->at_t(t_next) - position).norm();

			}

			int apples_seen_count = (int) std::count(apples_seen.begin(), apples_seen.end(), true);

			Json::Value result;
			result["params"] = path_json;
			result["apples_seen"] = apples_seen_count;
			result["length"] = length;

			json["data"].append(result);

		}

	}

	for (const auto &apple: apple_positions) {
		json["rays"].append(occlusion_model.exteriorVisibilityScore(apple, 1000));
	}

	std::ofstream out("analysis/data/orbit_results.json");
	out << json;
	out.close();
}