#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include "load_mesh.h"

shape_msgs::msg::Mesh meshMsgFromResource(const std::string &resource) {
	std::shared_ptr<shapes::Mesh> mesh_shape(shapes::createMeshFromResource(resource));
	shape_msgs::msg::Mesh mesh;
	shapes::ShapeMsg mesh_msg;
	shapes::constructMsgFromShape(mesh_shape.get(), mesh_msg);
	return boost::get<shape_msgs::msg::Mesh>(mesh_msg);
}

shape_msgs::msg::Mesh loadMesh(const std::string &name) {
	std::stringstream prefix_stream;
	prefix_stream << "file://";
	prefix_stream << MYSOURCE_ROOT;
	prefix_stream << "/3d-models/";
	prefix_stream << name;
	std::string prefix = prefix_stream.str();

	return std::move(meshMsgFromResource(prefix));
}
