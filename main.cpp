#include </Applications/MATLAB_R2023b.app/extern/include/mex.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <vector>
#include <fstream>
#include <array>
#include <map>
#include <limits>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef std::pair<Point, Vector> Point_with_normal;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
// ...other necessary types

// Extend the Point type to store RGB color information
struct ColorPoint {
    Point_3 point;
    unsigned char r, g, b;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    char *tempFile = mxArrayToString(prhs[0]);
    std::ifstream infile(tempFile);
    std::vector<ColorPoint> colored_points;
    double x, y, z;
    unsigned char r, g, b;
    while (infile >> x >> y >> z >> r >> g >> b) {
        colored_points.push_back({Point_3(x, y, z), r, g, b});
    }
    infile.close();


    std::vector<Point_3> points;
    std::vector<Vector_3> normals;
    for (const auto& colored_point : colored_points) {
        points.push_back(colored_point.point);
        // Assuming you also have normals in your PLY
        normals.push_back(Vector_3(normal_x, normal_y, normal_z)); // Replace with actual normal values

    }

    // Poisson Surface Reconstruction
    CGAL::Poisson_reconstruction_function<K> function(points.begin(), points.end(), Point_map(), Normal_map());

if (!function.compute_implicit_function()) {
        mexErrMsgTxt("Poisson surface reconstruction failed.");
        return;
    }

    // Create an empty surface mesh
    Mesh poisson_mesh;
    if (!poisson_reconstruction_function.reconstruct_surface(poisson_mesh)) {
        mexErrMsgTxt("Surface reconstruction failed.");
        return;
    }

    // Advancing Front Surface Reconstruction
    std::vector<std::array<std::size_t, 3>> output;
    CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(output));

    Mesh advancing_front_mesh;
    for (const auto& p : points) {
        point_to_vertex[p] = advancing_front_mesh.add_vertex(p);
    }
    for (const auto& face : output) {
        advancing_front_mesh.add_face(point_to_vertex[points[face[0]]], point_to_vertex[points[face[1]]], point_to_vertex[points[face[2]]]);
    }

    // Catmull-Clark Subdivision
    int numberOfSubdivisionSteps = 3;
    for (int i = 0; i < numberOfSubdivisionSteps; ++i) {
        CGAL::Subdivision_method_3::CatmullClark_subdivision(advancing_front_mesh);
    }

    // Surface Mesh Simplification
    namespace SMS = CGAL::Surface_mesh_simplification;
    SMS::Edge_count_stop_predicate<Mesh> stop(points.size() / 2);
  SMS::edge_collapse(advancing_front_mesh, stop, CGAL::parameters::get_default_edge_is_constrained_map(advancing_front_mesh));
 

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

    mxFree(tempFile);

    plhs[0] = mxCreateDoubleScalar(0.0); // 0 indicates success
}