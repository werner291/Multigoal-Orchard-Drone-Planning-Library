
#include "procedural_tree_generation.h"
#include "json_utils.h"
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

int main(int argc, char **argv) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution num_apples(5, 100);

    Json::Value all_trees;

    for (int i = 0; i < 100; i++) {
        all_trees.append(toJSON(generateTreeScene(num_apples(gen))));
        std::cout << "Tree: " << i << std::endl;

    }

    std::stringstream pathss;

    time_t now;
    time(&now);
    char buf[256];
    strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
    pathss << "test_robots/trees/trees_" << buf << ".json.gzip";

    std::string path = pathss.str();

    jsonToGzipFile(all_trees, path);

}