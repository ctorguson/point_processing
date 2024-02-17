#include </Applications/MATLAB_R2023b.app/extern/include/mex.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Point_set_3.h>
#include <fstream>
#include <CGAL/property_map.h>
#include <iostream>
#include <utility>
#include <vector>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef CGAL::Point_set_3<Point_3> PointSet;
typedef CGAL::Surface_mesh<Point_3> Mesh;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:arguments", "One input argument required.");
        return;
    }
    
    char *tempFile = mxArrayToString(prhs[0]);
    std::ifstream input(tempFile, std::ios::binary); // Assuming binary for broader compatibility
        if (!input) {
        mexErrMsgIdAndTxt("MATLAB:fileReadingError", "Cannot open file.");
        mxFree(tempFile);
        return;
    }

    PointSet points;
    Mesh mesh;
    Point_3 point;
    while (input >> point) {
      points.insert(point);
}

  //return info to MATLAB
    mxFree(tempFile);
}
