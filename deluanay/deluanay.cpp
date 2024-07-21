#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/draw_triangulation_3.h>
#include <iostream>
#include <fstream>
#include <sstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_3<K> DT3;
typedef CGAL::Creator_uniform_3<double, K::Point_3> Creator;
int main()
{
    std::vector<K::Point_3> points;

    std::ifstream file("bunny.pcd");
    std::string line;
    double x, y, z;
    while (getline(file, line))
    {
        std::istringstream iss(line);
        if (iss >> x >> y >> z)
            points.push_back(K::Point_3(x, y, z));
    }

    DT3 dt3(points.begin(), points.end());
    CGAL::draw(dt3);

    std::ofstream out("bunny.off");
    out << dt3;

    return EXIT_SUCCESS;
}