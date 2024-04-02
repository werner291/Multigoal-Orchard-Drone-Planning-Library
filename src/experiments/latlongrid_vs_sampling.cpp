/*
 * The purpose of this experiment is to get hard numbers on the performance of the lat/lon grid vs. sampling naively.
 */

#include <random_numbers/random_numbers.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/detail/traversal/collision_node-inl.h>
#include <chrono>

#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/fcl_utils.h"
#include "../planning/LatitudeLongitudeGrid.h"
#include "../planning/collision_detection.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../visualization/VtkLineSegmentVizualization.h"

using namespace mgodpl;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

struct RotatedLatLonGrid {
	LatLonGrid grid;
	Eigen::Matrix3d rot;
	Eigen::Matrix3d inv_rot;
	math::Vec3d center;
};

RotatedLatLonGrid mk_rotated_lat_lon_grid(const math::Vec3d& ideal_vector, const math::Vec3d& center) {
	// Build the lat/lon grid.
	LatLonGrid grid{
			{-M_PI/2.0, M_PI/2.0},
			{-M_PI/2.0, M_PI/2.0},
			0.025,
			50,
			50
	};

	// Put the triangles in:
	// Compute a rotation matrix such that the ideal vector is the unit X axis.
	Eigen::Matrix3d rot = Eigen::Quaterniond::FromTwoVectors(
			Eigen::Vector3d(ideal_vector.x(), ideal_vector.y(), ideal_vector.z()),
			Eigen::Vector3d(1.0, 0.0, 0.0)
			).toRotationMatrix();

	Eigen::Matrix3d inv_rot = rot.inverse();

	return RotatedLatLonGrid {
		.grid = grid,
		.rot = rot,
		.inv_rot = inv_rot,
		.center = center
	};
}

void insert_triangle_into_grid(RotatedLatLonGrid& grid, const mgodpl::Triangle& triangle) {
	// Rotate the triangle:
	Eigen::Vector3d a(triangle.vertices[0].x(), triangle.vertices[0].y(), triangle.vertices[0].z());
	Eigen::Vector3d b(triangle.vertices[1].x(), triangle.vertices[1].y(), triangle.vertices[1].z());
	Eigen::Vector3d c(triangle.vertices[2].x(), triangle.vertices[2].y(), triangle.vertices[2].z());

	a -= Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());
	b -= Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());
	c -= Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());

	a = grid.rot * a;
	b = grid.rot * b;
	c = grid.rot * c;

	mgodpl::Triangle tri {
		.vertices = {
			math::Vec3d {a.x(), a.y(), a.z()},
			math::Vec3d {b.x(), b.y(), b.z()},
			math::Vec3d {c.x(), c.y(), c.z()}
		}
	};

	grid.grid.insert_triangle(tri);
}

std::optional<RobotState> findGoalStateThroughLatLonGrid(
		const math::Vec3d& target,
		const math::Vec3d& canopy_middle,
		const robot_model::RobotModel& robot,
		const robot_model::RobotModel::LinkId& flying_base,
		const robot_model::RobotModel::LinkId& stick,
		const robot_model::RobotModel::LinkId& end_effector,
		const fcl::CollisionObjectd& tree_trunk_object,
		random_numbers::RandomNumberGenerator rng,
		const std::vector<mgodpl::Triangle>& triangles
		) {

	// Build the lat/lon grid.
	RotatedLatLonGrid grid = mk_rotated_lat_lon_grid(target - canopy_middle, target);

	// Put the triangles in:
	for (const auto& triangle : triangles) {
		insert_triangle_into_grid(grid, triangle);
	}

	// Go find the empty cells:
	for (size_t lat_i = 0; lat_i < grid.grid.latitude_cells; ++lat_i) {
		for (size_t lon_i = 0; lon_i < grid.grid.longitude_cells; ++lon_i) {
			auto cell = grid.grid.cells[grid.grid.cell_index({lat_i, lon_i})];

			if (cell.triangles.empty()) {
				// Generate a Lat/lon inside the cell.
				auto lats = grid.grid.latitude_range_of_cell(lat_i);
				auto lons = grid.grid.longitude_range_of_cell(lon_i);

				double lat = lats.interpolate(rng.uniform01());
				double lon = lons.interpolate(rng.uniform01()).longitude;

				// Convert to a vector
				mgodpl::math::Vec3d vec = spherical_geometry::RelativeVertex{lon, lat}.to_cartesian();

				// Un-rotate it.
				Eigen::Vector3d vec_eigen(vec.x(), vec.y(), vec.z());
				vec_eigen = grid.inv_rot * vec_eigen;

				// Generate a state.
				std::vector<double> arm_angles { -spherical_geometry::latitude(vec) };

				math::Transformd flying_base_tf {
						.translation = math::Vec3d(0.0, 0.0, 0.0),
						.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), spherical_geometry::longitude(vec) + M_PI/2.0)
				};

				const auto& fk = mgodpl::robot_model::forwardKinematics(robot, arm_angles, flying_base, flying_base_tf);

				math::Vec3d end_effector_position = fk.forLink(end_effector).translation;

				flying_base_tf.translation = flying_base_tf.translation + target - end_effector_position;

				const auto& fk2 = mgodpl::robot_model::forwardKinematics(robot, arm_angles, flying_base, flying_base_tf);

				PositionedShape positioned_shape {
						.shape = robot.getLinks()[stick].collision_geometry[0].shape,
						.transform = fk2.forLink(stick).then(robot.getLinks()[stick].collision_geometry[0].transform)
				};

				bool collision = check_link_collision(robot.getLinks()[stick], tree_trunk_object, fk2.forLink(stick));

				if (!collision)
					return RobotState {
						.base_tf = flying_base_tf,
						.joint_values = arm_angles
					};

			}
		}
	}

	return std::nullopt;


}

int main()
{
    const auto& robot = mgodpl::experiments::createProceduralRobotModel();

    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");
	robot_model::RobotModel::LinkId stick = robot.findLinkByName("stick");

    const auto& tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	std::vector<mgodpl::Triangle> triangles;
	for (const auto& triangle : tree_model.trunk_mesh.triangles) {
		triangles.push_back({
			.vertices = {
				math::Vec3d { tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].x, tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].y, tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].z },
				math::Vec3d { tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].x, tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].y, tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].z },
				math::Vec3d { tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].x, tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].y, tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].z }
			}
		});
	}

    // Allocate a BVH mesh for the tree trunk.
    const auto& tree_trunk_bvh = mgodpl::fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
    fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

    random_numbers::RandomNumberGenerator rng(42);

//    mgodpl::SimpleVtkViewer viewer(false);
//    viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
//    viewer.addMesh(createGroundPlane(5.0, 5.0), FLOOR_COLOR);

    const auto& targets = computeFruitPositions(tree_model);

	math::Vec3d canopy_middle = mesh_aabb(tree_model.leaves_mesh).center();

	size_t uniform_successes = 0;
	size_t grid_successes = 0;

	// Tally the total durations:
	long uniform_duration = 0;
	long grid_duration = 0;

	// Let's examine, for every target, how hard sampling is through either method.
	for (size_t i = 0; i < targets.size(); ++i) {
		std::cout << "Target " << i << " is at " << targets[i] << std::endl;

		math::Vec3d target = targets[i];
		math::Vec3d ideal_vector = target - canopy_middle;

		auto start = std::chrono::high_resolution_clock::now();

		bool found_uniform = findGoalStateByUniformSampling(
				target,
				robot,
				flying_base,
				end_effector,
				tree_trunk_object,
				rng,
				10000
		).has_value();

		auto end = std::chrono::high_resolution_clock::now();
		uniform_duration += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

		auto grid_start = std::chrono::high_resolution_clock::now();
		// And now through the grid.
		bool found_grid = findGoalStateThroughLatLonGrid(
				target,
				canopy_middle,
				robot,
				flying_base,
				stick,
				end_effector,
				tree_trunk_object,
				rng,
				triangles
		).has_value();
		auto grid_end = std::chrono::high_resolution_clock::now();

		grid_duration += std::chrono::duration_cast<std::chrono::milliseconds>(grid_end - grid_start).count();

		std::cout << "Uniform sampling: " << (found_uniform ? "found" : "not found") << std::endl;
		std::cout << "Grid sampling: " << (found_grid ? "found" : "not found") << std::endl;

		if (found_uniform) {
			++uniform_successes;
		}

		if (found_grid) {
			++grid_successes;
		}

		// Stats so far:
		std::cout << "Uniform sampling successes: " << uniform_successes << " with t = " << uniform_duration << " ms" << std::endl;
		std::cout << "Grid sampling successes: " << grid_successes << " with t = " << grid_duration << " ms" << std::endl;

	}

	std::cout << "Uniform sampling successes: " << uniform_successes << std::endl;
	std::cout << "Grid sampling successes: " << grid_successes << std::endl;

	SimpleVtkViewer viewer;

	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	for (const auto& target : targets) {
		viewer.addSphere(0.05, target, {1.0, 0.0, 0.0}, 1.0);

		// Make the rotated grid:
		RotatedLatLonGrid grid = mk_rotated_lat_lon_grid(target - canopy_middle, target);

		for (const auto& triangle : triangles) {
			insert_triangle_into_grid(grid, triangle);
		}

		// Then, vizualise it as a wireframe:
		VtkTriangleSetVisualization viz(0.0, 1.0, 0.5);
		VtkTriangleSetVisualization viz_occu(1.0, 0.0, 0.5);
		VtkLineSegmentsVisualization assoc_triangles(0.0, 0.0, 0.0);

		std::vector<std::array<math::Vec3d, 3>> triangles;
		std::vector<std::array<math::Vec3d, 3>> triangles_occu;
		std::vector<std::pair<math::Vec3d, math::Vec3d>> assoc_lines;

		for (size_t lat_i = 0; lat_i < grid.grid.latitude_cells; ++lat_i) {
			for (size_t lon_i = 0; lon_i < grid.grid.longitude_cells; ++lon_i) {
				auto cell = grid.grid.cells[grid.grid.cell_index({lat_i, lon_i})];

				// Generate a Lat/lon inside the cell.
				auto lats = grid.grid.latitude_range_of_cell(lat_i);
				auto lons = grid.grid.longitude_range_of_cell(lon_i);

				auto a = spherical_geometry::RelativeVertex{lons.start, lats.min}.to_cartesian() * 0.1;
				auto b = spherical_geometry::RelativeVertex{lons.end, lats.min}.to_cartesian() * 0.1;
				auto c = spherical_geometry::RelativeVertex{lons.end, lats.max}.to_cartesian() * 0.1;
				auto d = spherical_geometry::RelativeVertex{lons.start, lats.max}.to_cartesian() * 0.1;

				// Un-rotate them:
				Eigen::Vector3d a_eigen(a.x(), a.y(), a.z());
				Eigen::Vector3d b_eigen(b.x(), b.y(), b.z());
				Eigen::Vector3d c_eigen(c.x(), c.y(), c.z());
				Eigen::Vector3d d_eigen(d.x(), d.y(), d.z());

				a_eigen = grid.inv_rot * a_eigen;
				b_eigen = grid.inv_rot * b_eigen;
				c_eigen = grid.inv_rot * c_eigen;
				d_eigen = grid.inv_rot * d_eigen;

				// Add them to the center:
				a_eigen = a_eigen + Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());
				b_eigen = b_eigen + Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());
				c_eigen = c_eigen + Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());
				d_eigen = d_eigen + Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());

				if (cell.triangles.empty()) {
					// Draw a square:
					triangles.push_back({
												math::Vec3d{a_eigen.x(), a_eigen.y(), a_eigen.z()},
												math::Vec3d{b_eigen.x(), b_eigen.y(), b_eigen.z()},
												math::Vec3d{c_eigen.x(), c_eigen.y(), c_eigen.z()}
										});

					triangles.push_back({
												math::Vec3d{a_eigen.x(), a_eigen.y(), a_eigen.z()},
												math::Vec3d{c_eigen.x(), c_eigen.y(), c_eigen.z()},
												math::Vec3d{d_eigen.x(), d_eigen.y(), d_eigen.z()}
										});
				} else {
					// Draw a square:
					triangles_occu.push_back({
													 math::Vec3d{a_eigen.x(), a_eigen.y(), a_eigen.z()},
													 math::Vec3d{b_eigen.x(), b_eigen.y(), b_eigen.z()},
													 math::Vec3d{c_eigen.x(), c_eigen.y(), c_eigen.z()}
											 });

					triangles_occu.push_back({
													 math::Vec3d{a_eigen.x(), a_eigen.y(), a_eigen.z()},
													 math::Vec3d{c_eigen.x(), c_eigen.y(), c_eigen.z()},
													 math::Vec3d{d_eigen.x(), d_eigen.y(), d_eigen.z()}
											 });

					// Draw a line from the center of the cell to the center of the triangle that occupies it.
					auto center = (a_eigen + b_eigen + c_eigen + d_eigen) / 4.0;

					for (const auto& triangle : cell.triangles) {
						auto center2 = (triangle.vertices[0] + triangle.vertices[1] + triangle.vertices[2]) / 3.0;

						// Un-rotate them:
						Eigen::Vector3d center2_eigen(center2.x(), center2.y(), center2.z());

						center2_eigen = grid.inv_rot * center2_eigen;

						// Add them to the center:
						center2_eigen = center2_eigen + Eigen::Vector3d(grid.center.x(), grid.center.y(), grid.center.z());

						assoc_lines.push_back({
							math::Vec3d{center.x(), center.y(), center.z()},
							math::Vec3d{center2_eigen.x(), center2_eigen.y(), center2_eigen.z()}
						});
					}
				}

			}
		}

		viz.updateTriangles(triangles);
		viz_occu.updateTriangles(triangles_occu);
		assoc_triangles.updateLine(assoc_lines);

		viewer.addActor(viz.getActor());
		viewer.addActor(viz_occu.getActor());
		viewer.addActor(assoc_triangles.getActor());

		break;
	}

	viewer.start();
}
