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

#include <string>

#include "open3d/Open3D.h"

int main(int argc, char *argv[]) {
    // armadillo = o3d.data.ArmadilloMesh()
    // mesh = o3d.io.read_triangle_mesh(armadillo.path)
    // pcd = mesh.sample_points_poisson_disk(N)
    // # fit to unit cube
    // pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
    //           center=pcd.get_center())
    // pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    // o3d.visualization.draw_geometries([pcd])

    // print('octree division')
    // octree = o3d.geometry.Octree(max_depth=4)
    // octree.convert_from_point_cloud(pcd, size_expand=0.01)
    // o3d.visualization.draw_geometries([octree])


    int N = 2000;
    auto armadillo =  open3d::data::ArmadilloMesh();
    open3d::geometry::TriangleMesh mesh;
    open3d::io::ReadTriangleMesh(armadillo.GetPath(), mesh);
    auto pcd = mesh.SamplePointsPoissonDisk(N);


    // if (argc == 2) {
    //     std::string option(argv[1]);
    //     if (option == "--skip-for-unit-test") {
    //         open3d::utility::LogInfo("Skiped for unit test.");
    //         return 0;
    //     }
    // }

    // auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    // sphere->ComputeVertexNormals();
    // sphere->PaintUniformColor({0.0, 1.0, 0.0});
    // open3d::visualization::DrawGeometries({sphere});
    return 0;
}
