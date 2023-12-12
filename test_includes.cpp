#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_3.h>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Point_3<K> Point_3;

int main() {
    Point_3 p(0, 0, 0);
    std::cout << "Point x-coordinate: " << p.x() << std::endl;
    return 0;
}
