// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.


#ifndef NEW_PLANNERS_SIMPLEVTKVIEWER_H
#define NEW_PLANNERS_SIMPLEVTKVIEWER_H

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNew.h>
#include <functional>
#include <optional>
#include <shape_msgs/msg/mesh.hpp>
#include <Eigen/Core>
#include "VideoRecorder.h"

#include "../RobotPath.h"
#include "../TreeMeshes.h"

class SimpleVtkViewer {

public:
	// Create the VTK rendering objects.
	vtkNew<vtkRenderer> viewerRenderer;
	vtkNew<vtkRenderWindow> visualizerWindow;
	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

	// Optionally, a video recorder to record a video of the simulation.
	std::optional<VideoRecorder> videoRecorder;

	/**
	 * Create a new VTK viewer.
	 */
	SimpleVtkViewer();

	/**
	 * Start recording a video.
	 *
	 * @param filename The filename to save the video to.
	 */
	void startRecording(const std::string &filename);

	/**
	 * Capture a screenshot.
	 *
	 * @param filename The filename to save the screenshot to. (Must end in .png)
	 * @param bool Whether to render the window before capturing the screenshot.
	 */
	 void captureScreenshot(const std::string &filename, bool render = true);

	/**
	 * Stop recording a video and discard the results.
	 */
	void discardVideo();

	/**
	 * Add a vtkActor to the scene.
	 * @param actor 		The actor to add.
	 */
	void addActor(vtkActor *actor);

	/**
	 * Add a collection of vtkActors to the scene.
	 * @param actors 		The actors to add.
	 */
	void addActorCollection(vtkActorCollection *actors);

	/**
	 * Add a mesh to the scene, adsuming no transformations.
	 *
	 * @param mesh 			The mesh to add.
	 * @param color 		The color of the mesh.
	 */
	void addMesh(const shape_msgs::msg::Mesh &mesh,
				 const Eigen::Vector3d &color,
				 double opacity,
				 const Eigen::Vector3d &position = Eigen::Vector3d::Zero());

	/**
	 * Add a callback that is called every time the simulation is rendered.
	 * @param callback 		The callback to add.
	 */
	void addTimerCallback(std::function<void()> callback);

	/**
	 * Start rendering the scene; won't return until the user closes the window,
	 * or stop() is called.
	 */
	void start();

	/**
	 * Stop rendering the scene. This will cause start() to return,
	 * and if a video is being recorded, it will be saved to the file.
	 */
	void stop();

	void addStaticPolyline(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& color);

	void addStaticLines(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& lines, const Eigen::Vector3d& color);

};

/**
 * @brief Visualizes the traces of the base and end effector of a robot along a path, as well as the "ladder" connecting these traces.
 *
 * The "ladder" visualization is a method of showing the spatial relationship between the base and end effector of a robot along a path.
 * It is called a "ladder" because it consists of lines connecting the positions of the base and the end effector at each state along the path,
 * which can look like the rungs of a ladder when the robot is moving in a straight line.
 *
 * @param viewer The VTK viewer to use for the visualization.
 * @param rpath_moveit The path along which to visualize the traces and ladder.
 */
void visualizeBaseEndEffectorLadderTrace(SimpleVtkViewer &viewer, const RobotPath &rpath_moveit);

/**
 * @brief Adds a simplified orchard to the VTK viewer.
 *
 * This function adds the trees from a simplified orchard to a SimpleVtkViewer.
 * Each tree's trunk and leaves are represented with different colors, and each fruit of the tree is also added.
 * The trees are positioned in the viewer according to their positions in the orchard.
 *
 * @param viewer The VTK viewer to add the orchard to.
 * @param orchard The simplified orchard to be added to the viewer.
 */
void addTreeMeshesToViewer(SimpleVtkViewer &viewer, const TreeMeshes &current_tree_models);

/**
 * @brief Adds a simplified orchard to the VTK viewer.
 *
 * This function adds the trees from a simplified orchard to a SimpleVtkViewer.
 * Each tree's trunk and leaves are represented with different colors, and each fruit of the tree is also added.
 * The trees are positioned in the viewer according to their positions in the orchard.
 *
 * @param viewer The VTK viewer to add the orchard to.
 * @param orchard The simplified orchard to be added to the viewer.
 */
void addSimplifiedOrchardToViewer(SimpleVtkViewer&viewer, const SimplifiedOrchard& orchard);

#endif //NEW_PLANNERS_SIMPLEVTKVIEWER_H
