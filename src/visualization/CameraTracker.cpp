// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 6-9-24.
//

#include "CameraTracker.h"

namespace mgodpl {
	namespace visualization {
		CameraTracker::CameraTracker(SimpleVtkViewer &viewer, double aggressiveness)
				: viewer(viewer), aggressiveness(aggressiveness) {
			camera_center = {10.0, 10.0, 10.0};
			camera_target = {0.0, 0.0, 0.0};
		}

		void
		CameraTracker::setPositionAndFocus(const math::Vec3d &new_camera_center, const math::Vec3d &new_camera_target) {
			camera_center = camera_center * (1.0 - aggressiveness) + new_camera_center * aggressiveness;
			camera_target = camera_target * (1.0 - aggressiveness) + new_camera_target * aggressiveness;
			viewer.setCameraTransform(camera_center, camera_target);
		}
	} // visualization
} // mgodpl