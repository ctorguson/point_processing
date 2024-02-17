#include </Applications/MATLAB_R2023b.app/extern/include/mex.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Surface_mesh.h>
#include <vector>
#include <fstream>
#include <array>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>

#include <string>
#include <CGAL/property_map.h>
#include <map>
#include <iostream>
#include <utility>
#include <CGAL/Point_set_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
	
// Extend the Point type to store RGB color information
namespace SMS = CGAL::Surface_mesh_simplification;

struct ColorPoint {
    Point_3 point;
    Vector_3 normal;
    unsigned char r, g, b;

};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Read the point cloud from the temporary file created by MATLAB
    char *tempFile = mxArrayToString(prhs[0]);
    std::ifstream infile(tempFile);
    std::vector<ColorPoint> colored_points; 
    double x, y, z, nx, ny, nz;
    unsigned char r, g, b;
    // Close the file stream 
    infile.close();

    // Separate points and normals
    std::vector<Point_3> points;
    for (const auto& colored_point : colored_points) {
        points.push_back(colored_point.point);
    }

// Display number of points to validate 3d structure
  std::ofstream debugFile("debug_output.txt");
  debugFile << "Number of points read: " << colored_points.size() << "\n";
  if (!colored_points.empty()) {
    debugFile << "Sample point: (" << colored_points[0].point.x() << ", "
              << colored_points[0].point.y() << ", "
              << colored_points[0].point.z() << ")\n";
  }
  debugFile.close();

    // Poisson Surface Reconstruction
    std::vector<std::array<std::size_t, 3>> output;
    CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(output));

    // Create a mesh from the output
Mesh mesh;
std::map<Point_3, Mesh::Vertex_index> point_to_vertex;
for (const auto& p : points) {
    point_to_vertex[p] = mesh.add_vertex(p);
}
for (const auto& face : output) {
    mesh.add_face(point_to_vertex[points[face[0]]], point_to_vertex[points[face[1]]], point_to_vertex[points[face[2]]]);
}

// Apply Catmull-Clark subdivision to smooth the mesh
int numberOfSubdivisionSteps = 3;  // Here, the subdivision will be applied three times
for (int i = 0; i < numberOfSubdivisionSteps; ++i) {
    CGAL::Subdivision_method_3::CatmullClark_subdivision(mesh);
}

    // Mesh Simplification
    SMS::Edge_count_stop_predicate<Mesh> stop(points.size() / 2);
    SMS::edge_collapse(mesh, stop);

    // Assign colors to mesh vertices
    std::vector<std::array<unsigned char, 3>> vertex_colors(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        double min_distance = std::numeric_limits<double>::max();
        size_t nearest_index = 0;
        for (size_t j = 0; j < colored_points.size(); ++j) {
            double distance = CGAL::squared_distance(points[i], colored_points[j].point);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = j;
            }
        }
        vertex_colors[i] = {colored_points[nearest_index].r, colored_points[nearest_index].g, colored_points[nearest_index].b};
    }

    // Write the results to the OFF file
    std::ofstream out("output_mesh.off");
    out << "OFF\n";
    out << points.size() << " " << output.size() << " 0\n";
    for (size_t i = 0; i < points.size(); ++i) {
        out << points[i].x() << " " << points[i].y() << " " << points[i].z() 
            << " " << int(vertex_colors[i][0]) << " " << int(vertex_colors[i][1]) << " " << int(vertex_colors[i][2]) << "\n";
    }
    for (const auto& triplet : output) {
        out << "3 " << triplet[0] << " " << triplet[1] << " " << triplet[2] << "\n";
    }

    // Return the mesh file path to MATLAB
    plhs[0] = mxCreateString("output_mesh.off");
}
