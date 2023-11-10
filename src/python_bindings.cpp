// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <moveit_core/moveit/robot_model/robot_model.h>
#include "experiment_utils/load_robot_model.h"
#include "planning/moveit_state_tools.h"

#include <vtkActor.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "experiment_utils/TreeMeshes.h"
#include "visualization/SimpleVtkViewer.h"
#include "experiment_utils/load_robot_model.h"
#include "planning/JointSpacePoint.h"
#include "planning/moveit_state_tools.h"
#include "visualization/VtkRobotModel.h"
#include "planning/CollisionDetection.h"

using namespace mgodpl;
using namespace tree_meshes;
//using namespace visualization;
using namespace math;
using namespace moveit_facade;

vtkSmartPointer<vtkActor> mkPointMarkerSphere(math::Vec3d &target, SimpleVtkViewer &viewer) {// Create a mapper and actor for the sphere.
	// Create a small sphere at the target point.
	vtkNew<vtkSphereSource> sphere;
	sphere->SetRadius(0.1);
	sphere->Update();

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphere->GetOutputPort());
	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(1, 0, 0);
	sphereActor->SetPosition(target.components.data());
	viewer.addActor(sphereActor);

	return sphereActor;
}



void thingy() {

	const auto &robot = experiment_assets::loadRobotModel(1.0);

	SimpleVtkViewer viewer;
	viewer.lockCameraUp();
	Vec3d target(0, 0, 2);
	auto target_marker = mkPointMarkerSphere(target, viewer);
	const size_t N_SAMPLES = 20;

	for (int i = 0; i < N_SAMPLES; ++i) {
		JointSpacePoint jt = mgodpl::experiment_state_tools::genGoalSampleUniform(target, i, *robot);

		visualization::VtkRobotModel robotModelViz(robot, jt, {0.5, 0.5, 0.5});
		viewer.addActorCollection(robotModelViz.getLinkActors());
	}
	viewer.start();

}

namespace py = pybind11;

int add(int i, int j) {
	return i + j;
}

PYBIND11_MODULE(pymgodpl, m) {

	py::class_<math::Vec3d>(m, "Vec3d")
		.def(py::init<double, double, double>())
		.def_property("x", &math::Vec3d::getX, &math::Vec3d::setX)
		.def_property("y", &math::Vec3d::getY, &math::Vec3d::setY)
		.def_property("z", &math::Vec3d::getZ, &math::Vec3d::setZ)
		.def("__getitem__", [](const math::Vec3d &v, size_t i) {
			if (i >= 3) throw py::index_error();
			return v.components[i];
		})
		.def("__setitem__", [](math::Vec3d &v, size_t i, double val) {
			if (i >= 3) throw py::index_error();
			v.components[i] = val;
		})
		.def("__repr__", [](const math::Vec3d &v) {
			return "Vec3d(" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " + std::to_string(v.z()) + ")";
		});

	py::class_<moveit_facade::JointSpacePoint> (m, "JointSpacePoint")
		.def(py::init<const std::vector<double>&>())
		.def("__getitem__", [](const moveit_facade::JointSpacePoint &v, size_t i) {
			if (i >= v.joint_values.size()) throw py::index_error();
			return v.joint_values[i];
		})
		.def("__setitem__", [](moveit_facade::JointSpacePoint &v, size_t i, double val) {
			if (i >= v.joint_values.size()) throw py::index_error();
			v.joint_values[i] = val;
		});

	py::class_<TreeMeshes>(m, "TreeMeshes")
		.def_readonly("name", &TreeMeshes::tree_name)
		.def_readonly("trunk", &TreeMeshes::trunk_mesh)
		.def_readonly("leaves", &TreeMeshes::leaves_mesh)
		.def("fruit_positions", tree_meshes::computeFruitPositions);

	m.def("load_tree_meshes", &mgodpl::tree_meshes::loadTreeMeshes, "Load the meshes for the tree with the given name.");

	m.def("load_all_tree_meshes", &mgodpl::tree_meshes::loadAllTreeMeshes, "Load the meshes for all trees.");

	// I'm sure this is actually supposed to be in the ROS libraries but I don't feel like digging right now.
	// I'll just treat is as an opaque type.
	py::class_<shape_msgs::msg::Mesh>(m, "Mesh");

	py::class_<mgodpl::moveit_facade::CollisionDetection>(m, "CollisionDetection")
		.def(py::init<const std::vector<shape_msgs::msg::Mesh>&, const moveit::core::RobotModelConstPtr>())
		.def("collides", &mgodpl::moveit_facade::CollisionDetection::collides)
		.def("collides_ccd", &mgodpl::moveit_facade::CollisionDetection::collides_ccd)
		.def("path_collides", &mgodpl::moveit_facade::CollisionDetection::path_collides)
		.def("collision_ccd_toi", &mgodpl::moveit_facade::CollisionDetection::collision_ccd_toi);

	py::class_<moveit::core::RobotModel, std::shared_ptr<moveit::core::RobotModel>>(m, "RobotModel");

	m.doc() = "pybind11 example plugin"; // optional module docstring

	m.def("add", &add, "A function that adds two numbers");
	m.def("load_robot_model", &mgodpl::experiment_assets::loadRobotModel, "Load the default drone model.");

	m.def("sample_state_uniform", &mgodpl::experiment_state_tools::randomUprightWithBase, "Generate a random state that is valid for the drone.");

	m.def("sample_goal_region", &mgodpl::experiment_state_tools::genGoalSampleUniform, "Generate a random state that is valid for the drone where the drone's end-effector is near the target.");

	m.def("thingy", &thingy, "Do the thingy.");
}