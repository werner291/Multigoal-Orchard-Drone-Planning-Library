// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 6-9-24.
//

#ifndef MGODPL_CAMERATRACKER_H
#define MGODPL_CAMERATRACKER_H

#include "SimpleVtkViewer.h"

namespace mgodpl {
	namespace visualization {

		/**
		 * \class CameraTracker
		 * \brief A class to manage and smoothly update the camera position and focus in a SimpleVtkViewer.
		 */
		class CameraTracker {

			SimpleVtkViewer &viewer; ///< Reference to the SimpleVtkViewer instance.
			double aggressiveness; ///< Interpolation aggressiveness factor.
			math::Vec3d camera_center; ///< Current camera center position.
			math::Vec3d camera_target; ///< Current camera target position.

		public:
			/**
			 * \brief Constructor for CameraTracker.
			 * \param viewer A reference to the SimpleVtkViewer instance.
			 * \param aggressiveness A factor in the range [0, 1] that determines the interpolation aggressiveness.
			 */
			CameraTracker(SimpleVtkViewer &viewer, double aggressiveness);

			/**
			 * \brief Sets the new camera position and focus with interpolation.
			 * \param new_camera_center The new target position for the camera center.
			 * \param new_camera_target The new target position for the camera focus.
			 */
			void setPositionAndFocus(const math::Vec3d &new_camera_center, const math::Vec3d &new_camera_target);

		};

	} // visualization
} // mgodpl

#endif //MGODPL_CAMERATRACKER_H
