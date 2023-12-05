//
// Created by werner on 29-11-23.
//

#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/detail/traversal/collision_node-inl.h>
#include <chrono>

#include "../experiment_utils/TreeMeshes.h"
#include "../visualization/quick_markers.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkPolyLineVisualization.h"

#include "../planning/latitude_sweep.h"
#include "../planning/RobotState.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../experiment_utils/positioned_shape.h"

#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/fcl_utils.h"

using namespace mgodpl;

void update_free_latitude_ranges_visualization(const math::Vec3d &target,
											   VtkTriangleSetVisualization &intersections_visualization,
											   double longitude,
											   const std::vector<std::array<double, 2>> &free_latitudes) {
	std::vector<std::array<math::Vec3d, 3>> intersection_points;
	for (const auto& latitudes : free_latitudes)
	{
		double length = latitudes[1] - latitudes[0];

		size_t n_points = std::max(1, (int) (length * 16.0));

		for (int lat_i = 0; lat_i < n_points; ++lat_i)
		{
			double latitude1 = latitudes[0] + lat_i * (latitudes[1] - latitudes[0]) / (double) n_points;
			double latitude2 = latitudes[0] + (lat_i + 1) * (latitudes[1] - latitudes[0]) / (double) n_points;

			math::Vec3d ray1(
				cos(latitude1) * cos(longitude),
				cos(latitude1) * sin(longitude),
				sin(latitude1)
			);

			math::Vec3d ray2(
				cos(latitude2) * cos(longitude),
				cos(latitude2) * sin(longitude),
				sin(latitude2)
			);

			std::array<math::Vec3d, 3> triangle_points{
				target + ray1 * 1.0,
				target + ray2 * 1.0,
				target
			};

			intersection_points.push_back(triangle_points);
		}
	}

	intersections_visualization.updateTriangles(intersection_points);
}


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

int main(int argc, char** argv)
{
    const math::Vec3d WOOD_COLOR = {0.5, 0.35, 0.05};

    // Get a tree model.
    const auto& model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	const auto& robot = mgodpl::experiments::createProceduralRobotModel();

	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId stick = robot.findLinkByName("stick");
	robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

	// Allocate a BVH mesh for the tree trunk.
	const auto& tree_trunk_bvh = mgodpl::fcl_utils::meshToFclBVH(model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

    const math::Vec3d target(-0.195487, -0.769218, 1.34947);

    SimpleVtkViewer viewer(false);
    viewer.lockCameraUp();
    viewer.setCameraTransform(target + math::Vec3d{1.0, 0.0, 0.0}, target);

    visualization::mkPointMarkerSphere(viewer, target, {1.0, 0.0, 0.0});

    viewer.addMesh(
        model.trunk_mesh,
        WOOD_COLOR
    );

    VtkPolyLineVisualization line_visualization(1.0,0.0,1.0);
    viewer.addActor(line_visualization.getActor());

    VtkTriangleSetVisualization intersections_visualization(1.0, 1.0, 0.0);
    viewer.addActor(intersections_visualization.getActor());

    const std::vector<Triangle> triangles = model.trunk_mesh.triangles | ranges::views::transform(
        [&](const auto& triangle)
        {
            return Triangle{
                {
                    math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[0]].x,
                                model.trunk_mesh.vertices[triangle.vertex_indices[0]].y,
                                model.trunk_mesh.vertices[triangle.vertex_indices[0]].z),
                    math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[1]].x,
                                model.trunk_mesh.vertices[triangle.vertex_indices[1]].y,
                                model.trunk_mesh.vertices[triangle.vertex_indices[1]].z),
                    math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[2]].x,
                                model.trunk_mesh.vertices[triangle.vertex_indices[2]].y,
                                model.trunk_mesh.vertices[triangle.vertex_indices[2]].z)
                }
            };
        }) | ranges::to<std::vector>();

	const double STARTING_LONGITUDE = 0.0;

    double longitude = STARTING_LONGITUDE;

	std::cout << "Sweepline starting..." << std::endl;

	std::vector<RobotState> valid_states;

	const auto start_time = std::chrono::high_resolution_clock::now();
	LongitudeSweep sweepline_algorithm(triangles, target, STARTING_LONGITUDE);

	for (size)

		const auto& longitude_range = sweepline_algorithm.current_longitude_range();

		double longitude_midpoint = (longitude_range[0] + longitude_range[1]) / 2.0;

		std::cout << "Longitude midpoint: " << longitude_midpoint << std::endl;

		for (const auto& latitude_range : free_latitude_ranges(sweepline_algorithm.current_intersections(), target, longitude_midpoint, triangles, 0.25)) {
			double latitude_midpoint = (latitude_range[0] + latitude_range[1]) / 2.0;

			std::vector<double> arm_angles { -latitude_midpoint };

			math::Transformd flying_base_tf {
					.translation = math::Vec3d(0.0, 0.0, 0.0),
					.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), longitude_midpoint + M_PI/2.0)
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

			if (!collision) {
				// Check the base too.
				PositionedShape positioned_shape2 {
						.shape = robot.getLinks()[flying_base].collision_geometry[0].shape,
						.transform = fk2.forLink(flying_base).then(robot.getLinks()[flying_base].collision_geometry[0].transform)
				};

				bool collision2 = check_link_collision(robot.getLinks()[flying_base], tree_trunk_object, fk2.forLink(flying_base));

				if (!collision2) {
					valid_states.push_back(
							RobotState {
								flying_base_tf,
								arm_angles
							});
				}
			}
		}

	} while (sweepline_algorithm.has_more_events());

	// Now, we have a list of valid states. Vizualize them.
	for (const auto& state : valid_states) {

		// Use forward kinematics to find the position of the base and the arm.
		const auto& fk = mgodpl::robot_model::forwardKinematics(robot, state.joint_values, flying_base, state.base_tf);

		// Add the base.
		PositionedShape positioned_shape {
				.shape = robot.getLinks()[flying_base].visual_geometry[0].shape,
				.transform = fk.forLink(flying_base).then(robot.getLinks()[flying_base].collision_geometry[0].transform)
		};

		viewer.addPositionedShape(positioned_shape, {0.0, 1.0, 0.0});

		// Add the stick.
		PositionedShape positioned_shape2 {
				.shape = robot.getLinks()[stick].visual_geometry[0].shape,
				.transform = fk.forLink(stick).then(robot.getLinks()[stick].collision_geometry[0].transform)
		};

		viewer.addPositionedShape(positioned_shape2, {0.0, 1.0, 0.0});

	}

//    viewer.addTimerCallback([&]()
//    {
//
//        std::vector<math::Vec3d> points;
//        for (int lat_i = 0; lat_i <= 32; ++lat_i)
//        {
//            // Latitude from [-pi/2, pi/2]
//            double latitude = -M_PI / 2.0 + lat_i * M_PI / 32.0;
//
//            math::Vec3d ray(
//                cos(latitude) * cos(longitude),
//                cos(latitude) * sin(longitude),
//                sin(latitude)
//            );
//
//            points.push_back(target + ray * 1.0);
//        }
//
//        line_visualization.updateLine(points);
//
//	});

    viewer.start();


}