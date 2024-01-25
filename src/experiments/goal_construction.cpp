//
// Created by werner on 17-11-23.
//

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

struct RobotVizualization
{
    std::vector<vtkSmartPointer<vtkActor>> actors;
};

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

void gen_color_coded_states(const mgodpl::robot_model::RobotModel& robot,
                            robot_model::RobotModel::LinkId flying_base,
                            robot_model::RobotModel::LinkId end_effector,
                            fcl::CollisionObjectd tree_trunk_object,
                            random_numbers::RandomNumberGenerator rng,
                            mgodpl::SimpleVtkViewer& viewer,
                            const std::vector<math::Vec3d>& targets,
                            size_t hardest_target)
{
    const size_t N_SAMPLES = 50;
    for (size_t i = 0; i < N_SAMPLES; ++i)
    {
        RobotState state = genUprightState(rng);

        const auto& fk = robot_model::forwardKinematics(
            robot,
            state.joint_values,
            flying_base,
            state.base_tf
        );

        math::Vec3d target_delta = targets[hardest_target] - fk.forLink(end_effector).translation;

        state.base_tf.translation = state.base_tf.translation + target_delta;

        const auto& fk_at_target = robot_model::forwardKinematics(
            robot,
            state.joint_values,
            flying_base,
            state.base_tf
        );

        for (size_t i = 0; i < fk_at_target.link_transforms.size(); ++i)
        {
            auto& link_tf = fk_at_target.link_transforms[i];

            bool collision = check_link_collision(robot.getLinks()[i], tree_trunk_object, link_tf);

            for (const auto& geometry : robot.getLinks()[i].visual_geometry.empty()
                                            ? robot.getLinks()[i].collision_geometry
                                            : robot.getLinks()[i].visual_geometry)
            {
                math::Vec3d color = collision ? math::Vec3d(1.0, 0.0, 0.0) : math::Vec3d(0.0, 1.0, 0.0);

                PositionedShape positioned_shape{
                    .shape = geometry.shape,
                    .transform = link_tf.then(geometry.transform)
                };

                viewer.addPositionedShape(positioned_shape, color);
            }
        }
    }
}

struct Triangle
{
    math::Vec3d a, b, c;
};

std::vector<double> compute_collision_probabilities(const mgodpl::robot_model::RobotModel& robot,
                                                    robot_model::RobotModel::LinkId flying_base,
                                                    robot_model::RobotModel::LinkId end_effector,
                                                    fcl::CollisionObjectd tree_trunk_object,
                                                    random_numbers::RandomNumberGenerator rng,
                                                    const std::vector<math::Vec3d>& targets)
{
    std::vector<double> collision_probabilities;

    for (const auto& target : targets)
    {
        assert(robot.count_joint_variables() == 1);

        const size_t N_SAMPLES = 1000;

        int n_collisions = 0;

        for (size_t i = 0; i < N_SAMPLES; ++i)
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

            const auto& fk_at_target = robot_model::forwardKinematics(
                robot,
                state.joint_values,
                flying_base,
                state.base_tf
            );

            for (size_t i = 0; i < fk_at_target.link_transforms.size(); ++i)
            {
                auto& link_tf = fk_at_target.link_transforms[i];

                bool collision = check_link_collision(robot.getLinks()[i], tree_trunk_object, link_tf);

                if (collision)
                {
                    n_collisions++;
                    break;
                }
            }
        }

        double collision_probability = (double)n_collisions / (double)N_SAMPLES;

        collision_probabilities.push_back(collision_probability);
    }
    return collision_probabilities;
}

int main()
{
    const auto& robot = mgodpl::experiments::createProceduralRobotModel();

    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");
	robot_model::RobotModel::LinkId stick = robot.findLinkByName("stick");

    const auto& tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

    // Allocate a BVH mesh for the tree trunk.
    const auto& tree_trunk_bvh = mgodpl::fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
    fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

    random_numbers::RandomNumberGenerator rng(42);

    mgodpl::SimpleVtkViewer viewer(false);
    viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
    viewer.addMesh(createGroundPlane(5.0, 5.0), FLOOR_COLOR);

    const auto& targets = computeFruitPositions(tree_model);

	auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<double> collision_probabilities =
        compute_collision_probabilities(
            robot,
            flying_base,
            end_effector,
            tree_trunk_object,
            rng,
            targets);

	math::Vec3d canopy_middle = mesh_aabb(tree_model.leaves_mesh).center();

	auto end_time = std::chrono::high_resolution_clock::now();
	std::cout << "Collision probability computation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms." << std::endl;
	std::cout << "That is, " << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / (double)targets.size() << " us per target." << std::endl;

    for (size_t i = 0; i < targets.size(); ++i)
    {
        math::Vec3d color =
            math::Vec3d(1.0, 0.0, 0.0) * collision_probabilities[i] +
            math::Vec3d(0.0, 1.0, 0.0) * (1.0 - collision_probabilities[i]);

        visualization::mkPointMarkerSphere(viewer, targets[i], color);
    }

    // Pick the hardest non-impossible target; that is, the target with the highest collision probability that is not 1.0.
    size_t hardest_target = 0;
    for (size_t i = 0; i < collision_probabilities.size(); ++i)
    {
        if (collision_probabilities[i] > collision_probabilities[hardest_target] && collision_probabilities[i] < 1.0)
        {
            hardest_target = i;
        }
    }

	// Now, let's use the lat/lon grid to try to accelerate the computation.
	for (size_t i = 0; i < targets.size(); ++i) {
		std::cout << "Target " << i << " is at " << targets[i] << std::endl;

		math::Vec3d target = targets[i];
		math::Vec3d ideal_vector = target - canopy_middle;

		// Compute a rotation matrix such that the ideal vector is the unit X axis.
		Eigen::Matrix3d rot = Eigen::Quaterniond::FromTwoVectors(
				Eigen::Vector3d(ideal_vector.x(), ideal_vector.y(), ideal_vector.z()),
				Eigen::Vector3d(1.0, 0.0, 0.0)
				).toRotationMatrix();

		LatLonGrid grid{
				{-M_PI/2.0, M_PI/2.0},
				{-M_PI, M_PI},
				0.025,
				20,//static_cast<size_t>(rng.uniformInteger(5, 20)),
				20//static_cast<size_t>(rng.uniformInteger(5, 20))
		};

		// Put the triangles in:
		for (const auto& triangle : tree_model.trunk_mesh.triangles) {

			Eigen::Vector3d a(tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].x, tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].y, tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]].z);
			Eigen::Vector3d b(tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].x, tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].y, tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]].z);
			Eigen::Vector3d c(tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].x, tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].y, tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]].z);

			// Rotate them.
			a = rot * a;
			b = rot * b;
			c = rot * c;

			mgodpl::Triangle tri {
				.vertices = {
					math::Vec3d {a.x(), a.y(), a.z()},
					math::Vec3d {b.x(), b.y(), b.z()},
					math::Vec3d {c.x(), c.y(), c.z()}
				}
			};
		}


		Eigen::Matrix3d inv_rot = rot.inverse();

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

					if (collision)
					std::cout << "Found." << std::endl;

				}
			}
		}
	}

    std::cout
        << "Hardest target is " << hardest_target
		<< " at coords " << targets[hardest_target]
		<< " with collision probability " << collision_probabilities[hardest_target] << std::endl;

    {
        gen_color_coded_states(robot, flying_base, end_effector, tree_trunk_object, rng, viewer, targets,
                               hardest_target);

        viewer.addTimerCallback([&]()
        {
        });

        viewer.start();

        std::cout << "Done." << std::endl;
    }
}
