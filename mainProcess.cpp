#include </Applications/MATLAB_R2023b.app/extern/include/mex.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Subdivision_method_3.h>
#include <CGAL/Surface_mesh.h>
#include <vector>
#include <fstream>
#include <array>
#include <cstdlib> // For system()
#include <string>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3_Base;
typedef K::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3_Base> Mesh;
	
// Extend the Point type to store RGB color information
namespace SMS = CGAL::Surface_mesh_simplification;

struct ColorPoint {
    Point_3_Base point;
    unsigned char r, g, b;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Check for the right number of arguments
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:invalidNumInputs",
                          "One input required.");
    }

    // Convert MATLAB input to C-style string
    char *tempFile = mxArrayToString(prhs[0]);
    if (tempFile == nullptr) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:conversionFailed",
                          "Could not convert input to string.");
    }

    // Construct the command to run the Python script with the path to the temp file
    std::string pythonCommand = "python3 /Users/ciaratorguson/point_processing/loadPointCloud.py " + std::string(tempFile);

    // Execute the Python script
    int status = system(pythonCommand.c_str());
    if (status != 0) {
        mexErrMsgIdAndTxt("MATLAB:mexFunction:PythonError",
                          "Failed to execute Python script.");
    }

    // Read the point cloud from the temporary file created by MATLAB
    std::ifstream infile(tempFile);
    std::vector<ColorPoint> colored_points;
    double x, y, z;
    unsigned char r, g, b;
    while (infile >> x >> y >> z >> r >> g >> b) {
        colored_points.push_back({Point_3_Base(x, y, z), r, g, b});
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