//
// Created by werner on 30-9-22.
//

#ifndef NEW_PLANNERS_LOAD_MESH_H
#define NEW_PLANNERS_LOAD_MESH_H

#include <shape_msgs/msg/mesh.hpp>

shape_msgs::msg::Mesh meshMsgFromResource(const std::string &resource);

shape_msgs::msg::Mesh loadMesh(const std::string& name);

shape_msgs::msg::Mesh loadRobotMesh(const std::string& name);

#endif //NEW_PLANNERS_LOAD_MESH_H
