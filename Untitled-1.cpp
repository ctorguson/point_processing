#include </Applications/MATLAB_R2023b.app/extern/include/mex.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Subdivision_method_3.h>
#include <CGAL/Surface_mesh.h>
#include <iostream>
#include <vector>

#include <mex.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>// for Kernel types
#include <CGAL/IO/PLY/PLY_reader.h>// for reading PLY files
#include <tuple>
#include <array>// for std::tuple and std::array
#include <fstream>// for std::vector and ifstream
#include <CGAL/property_map.h>// for property maps
#include <fstream>// for std::vector and ifstream
#include <sstream> // Include this for std::istringstream

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3_Base;
typedef K::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3_Base> Mesh;
typedef std::array<unsigned char, 3> Color;
typedef std::tuple<Point_3_Base, Vector_3, Color, int> PNCI; // Point, Normal, Color, Intensity	


// Define the property maps
typedef CGAL::Nth_of_tuple_property_map<0, PNCI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNCI> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNCI> Color_map;
typedef CGAL::Nth_of_tuple_property_map<3, PNCI> Intensity_map;


// Extend the Point type to store RGB color information
namespace SMS = CGAL::Surface_mesh_simplification;

struct ColorPoint {
    Point_3_Base point;
    unsigned char r, g, b;
     Vector_3 normal; // Assuming Vector_3 is the correct type for normals in your setup
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Read the point cloud from the temporary file created by MATLAB
    char *tempFile = mxArrayToString(prhs[0]);
    std::ifstream infile(tempFile);
    std::string line;

    // Skip header lines until you reach "end_header"
    while (std::getline(infile, line)) {
    if (line == "end_header") {
        break;
    }
}

     std::vector<ColorPoint> colored_points;
    // Now read the points, colors, and normals
    double x, y, z, nx, ny, nz;
    unsigned char r, g, b;

    while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (!(iss >> x >> y >> z >> r >> g >> b >> nx >> ny >> nz)) {
        break; // Error in parsing the line
    }
    
    // Correctly include normals when adding to colored_points
    colored_points.push_back({Point_3_Base(x, y, z), r, g, b, Vector_3(nx, ny, nz)});
}

    infile.close();

    // Separate points and normals
    std::vector<Point_3_Base> points;
    for (const auto& colored_point : colored_points) {
        points.push_back(colored_point.point);
    }

    // Poisson Surface Reconstruction
    std::vector<std::array<std::size_t, 3>> output;
    CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(output));

    // Create a mesh from the output
    Mesh mesh;
    std::map<Point_3_Base, Mesh::Vertex_index> point_to_vertex;
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
    SMS::Count_stop_predicate<Mesh> stop(points.size() / 2);  // Simplify to half the original number of vertices
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

// Before closing your mexFunction, log the received file path and the number of points read
    mexPrintf("Received file path: %s\n", tempFile); // Correctly logs the file path
    mexPrintf("Number of points read: %lu\n", colored_points.size()); // Correctly logs the number of points read

    // Make sure to deallocate the dynamically allocated memory by MATLAB API
    mxFree(tempFile);


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

