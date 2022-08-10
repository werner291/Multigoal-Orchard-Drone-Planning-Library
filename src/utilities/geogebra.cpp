
#include "geogebra.h"

void geogebra_dump_named_point(const Eigen::Vector3d &middle_proj_euc, const std::string &name) {
	std::cout << name << " = (" << middle_proj_euc.x() << ", " << middle_proj_euc.y() << ", " << middle_proj_euc.z() << ")" << std::endl;
}

void geogebra_dump_walk(const std::vector<ConvexHullPoint> &walk) {
	std::cout << "walk = Polyline({";

	for (size_t i = 0; i < walk.size(); ++i) {
		std::cout << "(" << walk[i].position.x() << ", " << walk[i].position.y() << ", "
				  << walk[i].position.z() << ")";
		if (i < walk.size() - 1) {
			std::cout << ", ";
		}
	}

	std::cout << "})" << std::endl << std::flush;
}
