
#include "geogebra.h"
//
//void geogebra_dump_named_point(const Eigen::Vector3d &middle_proj_euc, const std::string &name) {
//	std::cout << name << " = (" << middle_proj_euc.x() << ", " << middle_proj_euc.y() << ", " << middle_proj_euc.z() << ")" << std::endl;
//}
//
//void geogebra_dump_walk(const std::vector<ConvexHullPoint> &walk) {
//	std::cout << "Polyline({";
//
//	for (size_t i = 0; i < walk.size(); ++i) {
//		std::cout << "(" << walk[i].position.x() << ", " << walk[i].position.y() << ", "
//				  << walk[i].position.z() << ")";
//		if (i < walk.size() - 1) {
//			std::cout << ", ";
//		}
//	}
//
//	std::cout << "})" << std::endl << std::flush;
//}
//
//void geogebra_dump_named_walk(const std::vector<ConvexHullPoint> &walk, const std::string &name) {
//	std::cout << name << " = Polyline({";
//
//	for (size_t i = 0; i < walk.size(); ++i) {
//		std::cout << "(" << walk[i].position.x() << ", " << walk[i].position.y() << ", "
//				  << walk[i].position.z() << ")";
//		if (i < walk.size() - 1) {
//			std::cout << ", ";
//		}
//	}
//
//	std::cout << "})" << std::endl << std::flush;
//}
//
//void geogebra_dump_named_face(const std::string &name, const Eigen::Vector3d &va, const Eigen::Vector3d &vb, const Eigen::Vector3d &vc) {
//	std::cout << name << " = Polygon({(" << va.x() << ", " << va.y() << ", " << va.z() << "), (" << vb.x() << ", " << vb.y() << ", " << vb.z() << "), (" << vc.x() << ", " << vc.y() << ", " << vc.z() << ")}) " << std::endl;
//}
//
//void geogebra_dump_mesh(const shape_msgs::msg::Mesh &mesh) {
//	for (size_t face_i = 0; face_i < mesh.triangles.size(); ++face_i) {
//
//		Eigen::Vector3d va(mesh.vertices[mesh.triangles[face_i].vertex_indices[0]].x,
//						   mesh.vertices[mesh.triangles[face_i].vertex_indices[0]].y,
//						   mesh.vertices[mesh.triangles[face_i].vertex_indices[0]].z);
//
//		Eigen::Vector3d vb(mesh.vertices[mesh.triangles[face_i].vertex_indices[1]].x,
//						   mesh.vertices[mesh.triangles[face_i].vertex_indices[1]].y,
//						   mesh.vertices[mesh.triangles[face_i].vertex_indices[1]].z);
//
//		Eigen::Vector3d vc(mesh.vertices[mesh.triangles[face_i].vertex_indices[2]].x,
//						   mesh.vertices[mesh.triangles[face_i].vertex_indices[2]].y,
//						   mesh.vertices[mesh.triangles[face_i].vertex_indices[2]].z);
//
//		geogebra_dump_named_face("face_" + std::to_string(face_i), va, vb, vc);
//	}
//}
