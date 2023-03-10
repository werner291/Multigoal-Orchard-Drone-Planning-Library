// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#include "apple_status_color_coding.h"
#include <range/v3/view/zip.hpp>
#include <vtkProperty.h>

std::array<double, 3> getColorForDiscoveryStatus(const utilities::DiscoveryStatus &status) {
	switch (status) {
		case utilities::VISITED:
			return APPLE_VISITED_COLOR;
		case utilities::EXISTS_BUT_UNKNOWN_TO_ROBOT:
			return APPLE_UNKNOWN_COLOR;
		case utilities::KNOWN_TO_ROBOT:
			return APPLE_KNOWN_COLOR;
		case utilities::ROBOT_THINKS_EXISTS_BUT_DOESNT:
			return APPLE_FALSE_COLOR;
		case utilities::REMOVED:
			return APPLE_REMOVED_COLOR;
		default:
			throw std::runtime_error("Unknown discovery status");
	}
}

void updateAppleColors(const std::vector<utilities::DiscoveryStatus> &discovery_statuses,
					   const std::vector<vtkActor *> &apple_actors) {
	for (const auto &[apple_actor, status]: ranges::views::zip(apple_actors, discovery_statuses)) {
		apple_actor->GetProperty()->SetDiffuseColor(getColorForDiscoveryStatus(status).data());
	}
}
