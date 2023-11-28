//
// Created by werner on 17-11-23.
//

#include <vtkProperty.h>
#include <moveit/robot_model/robot_model.h>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_state/robot_state.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <vtkRenderer.h>

#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/quick_markers.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/fcl_utils.h"

using namespace mgodpl;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

/**
 * Applies the given Eigen transform to the specified vtkActor.
 *
 * @param transform The Eigen transform to apply.
 * @param actor The vtkActor to apply the transform to.
 */
void applyEigenTransformToActor(Eigen::Isometry3d &transform, vtkActor *actor) {
	actor->SetPosition(
			transform.translation().x(),
			transform.translation().y(),
			transform.translation().z());

	Eigen::AngleAxisd tfRot(transform.rotation());

	actor->SetOrientation(0.0, 0.0, 0.0);

	actor->RotateWXYZ(tfRot.angle() / M_PI * 180.0,
					  tfRot.axis().x(),
					  tfRot.axis().y(),
					  tfRot.axis().z());
}

struct RobotState
{
	math::Transformd base_tf;
	std::vector<double> joint_values;
};

int main() {

	const auto &robot = mgodpl::experiments::createProceduralRobotModel();

	const auto &tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	math::Vec3d target(1.0, 0.0, 2.0);

	random_numbers::RandomNumberGenerator rng(42);

	mgodpl::SimpleVtkViewer viewer;
	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(createGroundPlane(5.0, 5.0), FLOOR_COLOR);


	mgodpl::visualization::mkPointMarkerSphere(target, viewer)->GetProperty()->SetOpacity(0.5);

	{
		viewer.setCameraTransform(target + math::Vec3d(2.0, 1.0, 1.0), target);
	}

	bool algo_continue = true;

	assert(robot.count_joint_variables() == 1);

	const size_t N_SAMPLES = 10;

	for (size_t i = 0; i < N_SAMPLES; ++i)
	{
		RobotState state {
			.base_tf = math::Transformd {
				.translation = math::Vec3d(0.0, 0.0, 0.0),
				.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), rng.uniformReal(-M_PI, M_PI))
				},
			.joint_values = { rng.uniformReal(-M_PI/2.0, M_PI/2.0) }
		};

		robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
		robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

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

		// Allocate a BVH mesh for the tree trunk.
		const auto& tree_trunk_bvh = mgodpl::fcl_utils::meshToFclBVH(tree_model.trunk_mesh);

		for (int i = 0; i < fk_at_target.link_transforms.size(); ++i)
		{
			auto& link_tf = fk_at_target.link_transforms[i];

			for (const auto& geometry : robot.getLinks()[i].visual_geometry.empty() ? robot.getLinks()[i].collision_geometry : robot.getLinks()[i].visual_geometry)
			{
				if (const auto& box = std::get_if<Box>(&geometry.shape))
				{
					viewer.addBox(box->size, link_tf.then(geometry.transform), {0.5,0.5,0.5});
				}
				else if (const auto& mesh = std::get_if<Mesh>(&geometry.shape))
				{
					viewer.addMesh(*mesh, link_tf.then(geometry.transform), {0.5,0.5,0.5});
				}
				else
				{
					throw std::runtime_error("Unknown geometry type.");
				}
			}
		}
	}

	viewer.addTimerCallback([&]() {

	});

	viewer.start();

	std::cout << "Done." << std::endl;
}
