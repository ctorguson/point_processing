#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/PLY.h>
#include <CGAL/Point_set_3.h>
#include <fstream>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::Point_set_3<K::Point_3> PointSet;

int main() {
    PointSet point_set;

    std::string filename = "/Users/ciaratorguson/point_processing/test.ply";
    if (!CGAL::IO::read_PLY(filename, point_set)) {
        std::cerr << "Error reading PLY file." << std::endl;
        return 1;
    }

    std::cout << "Point set has " << point_set.size() << " points." << std::endl;

    return 0;
}