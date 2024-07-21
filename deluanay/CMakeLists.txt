cmake_minimum_required(VERSION 3.10)
project(RDTExample)

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

add_executable(RDTExample deluanay.cpp)
target_link_libraries(RDTExample CGAL::CGAL CGAL::CGAL_Core CGAL::CGAL_Basic_viewer)
