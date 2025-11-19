cmake_minimum_required(VERSION 3.12...3.29)
project(Isotropic_Surface_Remeshing)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)
igl_include(glfw)

# Delaunator
add_subdirectory(dep/delaunator-cpp)
include_directories(dep/delaunator-cpp/include)

# CGAL
find_package(CGAL REQUIRED)

# create a target per cppfile
create_single_source_cgal_program("main.cpp")

target_link_libraries(main PUBLIC igl::glfw delaunator)



