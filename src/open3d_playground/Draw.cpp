// https://www.wobblyduckstudios.com/Octrees.php
//

#include <json/json.h>

#include <fstream>
#include <string>

#include "open3d/Open3D.h"
#include "open3d/utility/IJsonConvertible.h"

int main(int argc, char* argv[]) {
    int N = 1000;
    auto armadillo = open3d::data::ArmadilloMesh();
    open3d::geometry::TriangleMesh mesh;
    open3d::io::ReadTriangleMesh(armadillo.GetPath(), mesh);
    auto pcd = mesh.SamplePointsPoissonDisk(N);

    // std::shared_ptr<open3d::geometry::TriangleMesh> mesh =
    //         open3d::geometry::TriangleMesh::CreateBox(1, 1, 1);
    // auto pcd = mesh->SamplePointsPoissonDisk(N);

    // std::shared_ptr<open3d::geometry::PointCloud> pcd =
    //         std::make_shared<open3d::geometry::PointCloud>();
    // pcd->points_ = {{0, 0, 0}, {1, 0, 0}, {1.1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    pcd->PaintUniformColor({100, 0, 0});

    std::cout << "HasPoints: " << pcd->HasPoints() << std::endl;
    std::cout << "HasNormals: " << pcd->HasNormals() << std::endl;
    std::cout << "HasColors: " << pcd->HasColors() << std::endl;
    // open3d::visualization::DrawGeometries({pcd});

    std::size_t max_depth = 5;
    auto octree = std::make_shared<open3d::geometry::Octree>(max_depth);

    // size_expand A small expansion size such that the octree is
    // slightly bigger than the original point cloud bounds to accommodate all
    // points.

    double size_expand = 0.1;
    octree->ConvertFromPointCloud(*pcd, size_expand);
    open3d::visualization::DrawGeometries({octree});

    // You can query an existing node
    // If such a node did not originally exist in the plc
    // Then the output is null
    std::shared_ptr<open3d::geometry::OctreeLeafNode> node;
    std::shared_ptr<open3d::geometry::OctreeNodeInfo> node_info;
    std::tie(node, node_info) = octree->LocateLeafNode({1.1, 0, 0});
    if (node_info != nullptr) {
        std::cout << "node_info->origin_\n" << node_info->origin_ << std::endl;
    } else {
        std::cout << "node not found" << std::endl;
    }

    // What does the IsPointInBound function do?
    // origin <= point < origin + size.
    Eigen::Vector3d point{5, 0, 0};
    Eigen::Vector3d origin{5, 0, 0};
    double oc_size = 0.05;  // Size of the Octree.
    bool is_inbound = octree->IsPointInBound(point, origin, oc_size);
    std::cout << "is_inbound: " << is_inbound << std::endl;

    // Callback function for tree traversal
    // How do we integrate this with the collision checker
    // or cost function calculator
    std::vector<Eigen::Vector3d> colors_traversed;
    std::vector<size_t> child_indices_traversed;
    std::vector<size_t> depth_traversed;
    std::vector<Eigen::Vector3d> origin_traversed;
    std::vector<double> size_traversed;

    auto f = [&colors_traversed, &child_indices_traversed, &depth_traversed,
              &origin_traversed, &size_traversed](
                     const std::shared_ptr<open3d::geometry::OctreeNode>& node,
                     const std::shared_ptr<open3d::geometry::OctreeNodeInfo>&
                             node_info) -> bool {
        if (auto leaf_node = std::dynamic_pointer_cast<
                    open3d::geometry::OctreeColorLeafNode>(node)) {
            colors_traversed.push_back(leaf_node->color_);
            child_indices_traversed.push_back(node_info->child_index_);
            depth_traversed.push_back(node_info->depth_);
            origin_traversed.push_back(node_info->origin_);
            size_traversed.push_back(node_info->size_);
        }
        return false;
    };

    colors_traversed.clear();
    child_indices_traversed.clear();
    octree->Traverse(f);

    std::cout << "colors_traversed.size(): " << colors_traversed.size()
              << std::endl;
    std::cout << "child_indices_traversed.size(): "
              << child_indices_traversed.size() << std::endl;

    // for (auto a : origin_traversed) {
    //     std::cout << "a\n" << a << std::endl;
    // }

    // This is a way to write out the octree into a json file
    // we can do this from the luxonis side, in python
    // and then read from the c++ side
    Json::Value json_value;
    octree->ConvertToJsonValue(json_value);

    // Create an ofstream file stream to write to a file
    std::ofstream fileStream("output.json");

    // Check if the file is open
    if (fileStream.is_open()) {
        // Create a Json::StreamWriterBuilder
        Json::StreamWriterBuilder builder;
        // Optional: You can modify the writer's settings here, for example:
        // builder["commentStyle"] = "None";
        // builder["indentation"] = "   ";  // Use 3 spaces for indentation

        // Use the Json::StreamWriterBuilder to write the Json::Value to the
        // file
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(json_value, &fileStream);

        // Close the file stream
        fileStream.close();
    } else {
        std::cerr << "Could not open file for writing.\n";
    }

    return 0;
}
