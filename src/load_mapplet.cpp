
#include "load_mapplet.h"
#include "procedural_tree_generation.h"
//#include <jsoncpp/json/json.h>
#include <fstream>
#include <geometric_shapes/check_isometry.h>


std::vector<DetachedTreeNode> load_mapplet_json() {

    Json::Value root;
    std::ifstream file("/home/werner/workspace/mapplet_stuff/MAppleT/share/data/tree_out.json");
    file >> root;

    assert(root.isArray());

    std::vector<DetachedTreeNode> nodes;
    nodes.resize(root.size());

    for (unsigned int i = 0; i < root.size(); ++i) {

        nodes[i].root_at_absolute.setIdentity();
        Json::Value &position = root[i]["position"];
        nodes[i].root_at_absolute.translate(Eigen::Vector3d(
                position[0].asDouble(),
                position[1].asDouble(),
                position[2].asDouble()
        ));

        Json::Value &left = root[i]["left"];
        Json::Value &up = root[i]["up"];
        Json::Value &heading = root[i]["heading"];


        Eigen::Matrix3d rotmat;
        rotmat <<
               left[0].asDouble(), left[0].asDouble(), left[0].asDouble(),
                up[0].asDouble(), up[0].asDouble(), up[0].asDouble(),
                heading[0].asDouble(), heading[0].asDouble(), heading[0].asDouble();

        nodes[i].root_at_absolute.rotate(rotmat);

        assert(checkIsometry(nodes[i].root_at_absolute, std::numeric_limits<double>::epsilon()));

//        nodes[i].root_at_absolute.rotate()

    }

    return nodes;
}
