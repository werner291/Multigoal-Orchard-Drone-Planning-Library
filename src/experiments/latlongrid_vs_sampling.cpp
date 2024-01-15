/*
 * The purpose of this experiment is to get hard numbers on the performance of the lat/lon grid vs. sampling naively.
 */

#include <vtkProperty.h>
#include <random_numbers/random_numbers.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <vtkRenderer.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/detail/traversal/collision_node-inl.h>
#include <chrono>

#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/quick_markers.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/fcl_utils.h"
#include "../planning/RobotModel.h"
#include "../planning/RobotState.h"
#include "../planning/LatitudeLongitudeGrid.h"

using namespace mgodpl;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

/**
 * Check for collisions of a single robot link with a given fcl collision object.
 *
 * @param link 					The link to check.
 * @param tree_trunk_object 	The collision object to check against.
 * @param link_tf 				The transform of the link.
 * @return 						True if there is a collision, false otherwise.
 */
bool check_link_collision(const mgodpl::robot_model::RobotModel::Link& link,
                          const fcl::CollisionObjectd& tree_trunk_object,
                          const std::vector<math::Transformd>::value_type& link_tf)
{
    bool collision = false;

    for (const auto& collision_geometry : link.collision_geometry)
    {
        if (const auto& box = std::get_if<Box>(&collision_geometry.shape))
        {
            math::Transformd total_tf = link_tf.then(collision_geometry.transform);

            fcl::Transform3d fcl_tf;
            fcl_tf.setIdentity();
            fcl_tf.translation() = fcl::Vector3d(total_tf.translation.x(), total_tf.translation.y(),
                                                 total_tf.translation.z());
            fcl_tf.rotate(fcl::Quaterniond(total_tf.orientation.w, total_tf.orientation.x,
                                           total_tf.orientation.y, total_tf.orientation.z));

            fcl::CollisionObjectd box_object(
                std::make_shared<fcl::Boxd>(box->size.x(), box->size.y(), box->size.z()),
                fcl_tf);

            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;
            fcl::collide(
                &tree_trunk_object,
                &box_object,
                request,
                result
            );

            if (result.isCollision())
            {
                collision = true;
                break;
            }
        }
        else
        {
            throw std::runtime_error("Only boxes are implemented for collision geometry.");
        }
    }

    return collision;
}

/**
 * Check for collisions between a whole robot and a single FCL collision object.
 *
 * @param robot 					The robot to check.
 * @param tree_trunk_object 		The collision object to check against.
 * @param state 					The state of the robot.
 * @return 							True if there is a collision, false otherwise.
 */
bool check_robot_collision(const mgodpl::robot_model::RobotModel& robot,
						   const fcl::CollisionObjectd& tree_trunk_object,
						   const mgodpl::RobotState& state)
{
	bool collision = false;

	const auto& fk = robot_model::forwardKinematics(
		robot,
		state.joint_values,
		robot.findLinkByName("flying_base"),
		state.base_tf
	);

	for (size_t i = 0; i < fk.link_transforms.size(); ++i)
	{
		const auto& link_tf = fk.link_transforms[i];

		collision |= check_link_collision(robot.getLinks()[i], tree_trunk_object, link_tf);

		if (collision)
		{
			break;
		}
	}

	return collision;
}

/**
 * Generate an upright robot state, without checking for collisions.
 * @param rng 		The random number generator to use.
 * @return 			The generated robot state.
 */
RobotState genUprightState(random_numbers::RandomNumberGenerator rng)
{
    RobotState state = {
        .base_tf = math::Transformd{
            .translation = math::Vec3d(0.0, 0.0, 0.0),
            .orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), rng.uniformReal(-M_PI, M_PI))
        },
        .joint_values = {rng.uniformReal(-M_PI / 2.0, M_PI / 2.0)}
    };

    return state;
}

/**
 * Generate a state where the end effector is at the given target, not checking for collisions.
 *
 * @param rng 					The random number generator to use.
 * @param target 				The target position.
 * @param robot 				The robot model.
 * @param flying_base 			The link ID of the flying base.
 * @param end_effector 			The link ID of the end effector.
 * @return 						The generated robot state.
 */
RobotState genGoalStateUniform(
    random_numbers::RandomNumberGenerator rng,
    const math::Vec3d& target,
    const robot_model::RobotModel& robot,
    const robot_model::RobotModel::LinkId& flying_base,
    const robot_model::RobotModel::LinkId& end_effector)
{
    RobotState state = genUprightState(rng);

    const auto& fk = robot_model::forwardKinematics(
        robot,
        state.joint_values,
        flying_base,
        state.base_tf
    );

    math::Vec3d target_delta = target - fk.forLink(end_effector).translation;

    state.base_tf.translation = state.base_tf.translation + target_delta;

    return state;
}

/**
 * Attempt to find a collision-free goal state by uniform sampling.
 *
 * @param target 					The target position that the end-effector should be near.
 * @param robot 					The robot model.
 * @param flying_base 				The link ID of the flying base.
 * @param end_effector 				The link ID of the end effector.
 * @param tree_trunk_object 		The collision object of the tree trunk.
 * @param rng 						The random number generator to use.
 * @param max_attempts 				The maximum number of attempts to make.
 * @return 							The generated robot state, or nullopt if no state was found.
 */
std::optional<RobotState> findGoalStateByUniformSampling(
			const math::Vec3d& target,
	const robot_model::RobotModel& robot,
	const robot_model::RobotModel::LinkId& flying_base,
	const robot_model::RobotModel::LinkId& end_effector,
	const fcl::CollisionObjectd& tree_trunk_object,
	random_numbers::RandomNumberGenerator rng,
	size_t max_attempts) {

	for (size_t i = 0; i < max_attempts; ++i) {
		RobotState state = genGoalStateUniform(rng, target, robot, flying_base, end_effector);

		if (!check_robot_collision(robot, tree_trunk_object, state)) {
			return state;
		}
	}

	return std::nullopt;

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
	LatLonGrid grid{
			{-M_PI/2.0, M_PI/2.0},
			{-M_PI, M_PI},
			0.025,
			20,
			20
	};

	// Put the triangles in:
	math::Vec3d ideal_vector = target - canopy_middle;

	// Compute a rotation matrix such that the ideal vector is the unit X axis.
	Eigen::Matrix3d rot = Eigen::Quaterniond::FromTwoVectors(
			Eigen::Vector3d(ideal_vector.x(), ideal_vector.y(), ideal_vector.z()),
			Eigen::Vector3d(1.0, 0.0, 0.0)
			).toRotationMatrix();

	Eigen::Matrix3d inv_rot = rot.inverse();

	for (const auto& triangle : triangles) {

		Eigen::Vector3d a(triangle.vertices[0].x(), triangle.vertices[0].y(), triangle.vertices[0].z());
		Eigen::Vector3d b(triangle.vertices[1].x(), triangle.vertices[1].y(), triangle.vertices[1].z());
		Eigen::Vector3d c(triangle.vertices[2].x(), triangle.vertices[2].y(), triangle.vertices[2].z());

		// Rotate them.
		a = rot * a;
		b = rot * b;
		c = rot * c;

		mgodpl::Triangle tri {
			.vertices = {
				math::Vec3d {a.x(), a.y(), a.z()} - target,
				math::Vec3d {b.x(), b.y(), b.z()} - target,
				math::Vec3d {c.x(), c.y(), c.z()} - target
			}
		};

		grid.insert_triangle(tri);
	}

	// Go find the empty cells:
	for (size_t lat_i = 0; lat_i < grid.latitude_cells; ++lat_i) {
		for (size_t lon_i = 0; lon_i < grid.longitude_cells; ++lon_i) {
			auto cell = grid.cells[grid.cell_index({lat_i, lon_i})];

			if (cell.triangles.empty()) {
				// Generate a Lat/lon inside the cell.
				auto lats = grid.latitude_range_of_cell(lat_i);
				auto lons = grid.longitude_range_of_cell(lon_i);

				double lat = lats.interpolate(rng.uniform01());
				double lon = lons.interpolate(rng.uniform01()).longitude;

				// Convert to a vector
				mgodpl::math::Vec3d vec = spherical_geometry::RelativeVertex{lat, lon}.to_cartesian();

				// Un-rotate it.
				Eigen::Vector3d vec_eigen(vec.x(), vec.y(), vec.z());
				vec_eigen = inv_rot * vec_eigen;

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

//    std::cout
//        << "Hardest target is " << hardest_target
//		<< " at coords " << targets[hardest_target]
//		<< " with collision probability " << collision_probabilities[hardest_target] << std::endl;
//
//    {
//        gen_color_coded_states(robot, flying_base, end_effector, tree_trunk_object, rng, viewer, targets,
//                               hardest_target);
//
//        viewer.addTimerCallback([&]()
//        {
//        });
//
//        viewer.start();
//
//        std::cout << "Done." << std::endl;
//    }
}
