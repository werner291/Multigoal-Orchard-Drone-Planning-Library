#include "SurfletVolume.h"

#include "../HashedSpatialIndex.h"

void SurfletVolume::add(SurfletVolume::Surflet s) {
	surflets.push_back(std::move(s));
}

std::optional<SurfletVolume::NearbySurflet> SurfletVolume::closest(const Eigen::Vector3d &query) const {

	std::optional<SurfletVolume::NearbySurflet> nearest = {};

	for (const auto &surflet: surflets) {

		const double distance_squared = (surflet.point - query).squaredNorm();

		if (!nearest || distance_squared < nearest->distance_squared) {
			nearest = {surflet, distance_squared};
		}

	}

	return nearest;

}

bool SurfletVolume::isInside(const Eigen::Vector3d &p) const {

	return signedDistance(p) <= 0;

}

SurfletVolume SurfletVolume::unionWith(const SurfletVolume &other) {

	if (this->surflets.empty()) {
		return other;
	}

	if (other.surflets.empty()) {
		return *this;
	}

	SurfletVolume result;

	// The union will have the surflets of both volumes, unless the surflet of one volume is inside the other.

	for (const auto &s: surflets) {

		auto nearest = other.closest(s.point);

		// TODO Use the smoothed normals here instead of the original normals?
		Eigen::Hyperplane<double, 3> other_surflet_plane(nearest->surflet.normal, nearest->surflet.point);
		Eigen::Vector3d projection = other_surflet_plane.projection(s.point);

		double distance_from_other_surflet = (projection - nearest->surflet.point).norm();
		double signed_distance = other_surflet_plane.signedDistance(s.point);

		if (signed_distance >= -distance_from_other_surflet * 0.5) {
			result.add(s);
		}
	}

	// TODO Fix the repeated code here
	for (const auto &s: other.surflets) {

		auto nearest = this->closest(s.point);

		Eigen::Hyperplane<double, 3> this_surflet_plane(nearest->surflet.normal, nearest->surflet.point);
		Eigen::Vector3d projection = this_surflet_plane.projection(s.point);

		double distance_from_this_surflet = (projection - nearest->surflet.point).norm();
		double signed_distance = this_surflet_plane.signedDistance(s.point);

		if (signed_distance >= -distance_from_this_surflet * 0.5 && nearest->distance_squared > 0.01) {

			Eigen::Vector3d point_delta = nearest->surflet.point - s.point;

			result.add(s);

			double other_signed_distance = Eigen::Hyperplane<double, 3>(s.normal,
																		s.point).signedDistance(nearest->surflet.point);

			if (other_signed_distance > 0) {
				result.add({(s.point + nearest->surflet.point) * 0.5,
							(s.normal + nearest->surflet.normal).normalized()});
			}


		}

	}
	return result;

}

const std::vector<SurfletVolume::Surflet> &SurfletVolume::getSurflets() const {
	return surflets;
}

double SurfletVolume::signedDistance(const Eigen::Vector3d &p) const {

	if (auto nearest = closest(p)) {
		return (p - nearest->surflet.point).dot(nearest->surflet.normal);
	} else {
		return std::numeric_limits<double>::infinity();
	}

}
//
//void SurfletVolume::recalculateNormals(double radius) {
//
//	// First, build a spatial index for easy lookup of nearby surflets.
//	HashedSpatialIndex<size_t> grid(radius, surflets.size() / 2 + 1);
//
//	for (size_t i = 0; i < surflets.size(); i++) {
//		grid.insert(surflets[i].point, i);
//	}
//
//	// Now, for each surflet, find all surflets within the radius.
//
//	for (auto & surflet : surflets) {
//
//		surflet.original_normal = Eigen::Vector3d::Zero();
//
//		for (auto & neighbor : grid.query(surflet.point)) {
//
//			const auto & neighbor_surflet = surflets[neighbor];
//
//			surflet.smoothed_normal += neighbor_surflet.original_normal;
//
//		}
//
//		surflet.smoothed_normal.normalize();
//
//	}
//
//
//
//
//}
