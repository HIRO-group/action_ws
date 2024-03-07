// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <json/json.h>

#include <fstream>
#include <string>

#include "open3d/Open3D.h"
#include "open3d/utility/IJsonConvertible.h"

int main(int argc, char* argv[]) {
    int N = 2000;
    auto armadillo = open3d::data::ArmadilloMesh();
    open3d::geometry::TriangleMesh mesh;
    open3d::io::ReadTriangleMesh(armadillo.GetPath(), mesh);
    auto pcd = mesh.SamplePointsPoissonDisk(N);
    pcd->PaintUniformColor({100, 0, 0});
    std::cout << "HasPoints: " << pcd->HasPoints() << std::endl;
    std::cout << "HasNormals: " << pcd->HasNormals() << std::endl;
    std::cout << "HasColors: " << pcd->HasColors() << std::endl;
    // open3d::visualization::DrawGeometries({pcd});

    std::size_t max_depth = 4;
    auto octree = std::make_shared<open3d::geometry::Octree>(6);
    octree->ConvertFromPointCloud(*pcd, 0.01);
    // open3d::visualization::DrawGeometries({octree});

    // You can query an existing node
    // If such a node did not originally exist in the plc
    // Then the output is null
    std::shared_ptr<open3d::geometry::OctreeLeafNode> node;
    std::shared_ptr<open3d::geometry::OctreeNodeInfo> node_info;
    std::tie(node, node_info) = octree->LocateLeafNode({0, 0, 0});
    if (node_info != nullptr) {
        std::cout << "node_info->origin_" << node_info->origin_ << std::endl;
    } else {
        std::cout << "node not found" << std::endl;
    }

    // What does the IsPointInBound function do?

    // Callback function for tree traversal
    // How do we integrate this with the collision checker
    // or cost function calculator
    std::vector<Eigen::Vector3d> colors_traversed;
    std::vector<size_t> child_indices_traversed;
    auto f = [&colors_traversed, &child_indices_traversed](
                     const std::shared_ptr<open3d::geometry::OctreeNode>& node,
                     const std::shared_ptr<open3d::geometry::OctreeNodeInfo>&
                             node_info) -> bool {
        if (auto leaf_node = std::dynamic_pointer_cast<
                    open3d::geometry::OctreeColorLeafNode>(node)) {
            colors_traversed.push_back(leaf_node->color_);
            child_indices_traversed.push_back(node_info->child_index_);
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
