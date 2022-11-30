#include "SurfletVolume.h"

void SurfletVolume::add(SurfletVolume::Surflet s) {
	surflets.push_back(std::move(s));
}

SurfletVolume::NearbySurflet SurfletVolume::closest(const Eigen::Vector3d &query) const {

	assert(!surflets.empty());

	NearbySurflet nearest{surflets.front(), (surflets.front().point - query).squaredNorm()};

	for (size_t i = 1; i < surflets.size(); ++i) {

		const double distance_squared = (surflets[i].point - query).squaredNorm();

		if (distance_squared < nearest.distance_squared) {
			nearest = {surflets[i], distance_squared};
		}

	}

	return nearest;

}

bool SurfletVolume::isInside(const Eigen::Vector3d &p) const {

	auto nearest = closest(p);

	std::cout << "Nearest p: " << nearest.surflet.point.transpose() << " n: " << nearest.surflet.normal.transpose()
			  << std::endl;

	return (p - nearest.surflet.point).dot(nearest.surflet.normal) <= 0;

}

SurfletVolume SurfletVolume::unionWith(const SurfletVolume &other) {

	SurfletVolume result;

	// The union will have the surflets of both volumes, unless the surflet of one volume is inside the other.

	for (const auto &s: surflets) {
		if (!other.isInside(s.point)) {
			result.add(s);
		}
	}

	for (const auto &s: other.surflets) {
		if (!isInside(s.point)) {
			result.add(s);
		}
	}

	return result;

}
