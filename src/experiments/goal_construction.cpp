//
// Created by werner on 17-11-23.
//

#include <vtkProperty.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/robot_model.h>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_state/robot_state.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_request.h>
#include <geometric_shapes/shapes.h>
#include <vtkRenderer.h>

#include "../experiment_utils/load_robot_model.h"
#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"
#include "../visualization/quick_markers.h"
#include "../planning/JointSpacePoint.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/fcl_utils.h"
#include "../planning/IncrementalGoalStateGenerator.h"

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

int main() {

	const auto &robot = mgodpl::experiment_assets::loadRobotModel(1.0);

	const auto &tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	math::Vec3d target(1.0, 0.0, 2.0);

	random_numbers::RandomNumberGenerator rng(43);

	GoalStateAlgorithm gsa(
			robot,
			tree_model.trunk_mesh,
			target,
			rng);


	mgodpl::SimpleVtkViewer viewer;
	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(createGroundPlane(5.0, 5.0), FLOOR_COLOR);


	mgodpl::visualization::mkPointMarkerSphere(target, viewer)->GetProperty()->SetOpacity(0.5);

	{
		viewer.setCameraTransform(target + math::Vec3d(2.0, 1.0, 1.0), target);
	}

	bool algo_continue = true;

	std::vector<vtkSmartPointer<vtkActor>> link_actors;

	int timer = 10;

	viewer.addTimerCallback([&]() {

		if (timer-- == 0) {
			gsa.iterate();
			timer = 10;

			// Delete old actors.
			for (auto &actor: link_actors) {
				viewer.viewerRenderer->RemoveActor(actor);
			}

			for (auto &current_node: gsa.nodes) {

				const moveit::core::LinkModel *link = gsa.kinematic_chain[current_node.depth];

				if (!link->getCollisionOriginTransforms().empty()) {
					Eigen::Isometry3d transform = current_node.link_tf * link->getCollisionOriginTransforms()[0];


					auto link_actor = visualization::VtkRobotModel::actorForLink({
																						 0.8,
																						 0.8,
																						 0.8
																				 },
																				 link);

					applyEigenTransformToActor(transform, link_actor);

					viewer.addActor(link_actor);
				}
			}
		}

	});

//	viewer.startRecording("last_link_steady.ogv");

	viewer.start();

	std::cout << "Done." << std::endl;
}
