# Isotropic Surface Remeshing

Three libraries are used in this project : `libigl`, `CGAL` and `delaunator`.

## Libreries and Compilation
#### I - CGAL 
The procedure is explained in the [CGAL manual](https://doc.cgal.org/6.0/Manual/windows.html),  This library is essential for performing [Delaunay Tessellations and Voronoi Diagrams](https://inria.hal.science/hal-01421021/) 
Let's start by installing `vcpkg` 

```bash
C:> git clone https://github.com/microsoft/vcpkg
C:> cd vcpkg
C:\vcpkg> .\bootstrap-vcpkg.bat
```

Then install `CGAL` with vcpkg. However, CGAL depends on other libraries such as `GMP` and `MPFR`, which must be managed properly on Windows:

```shell
# Since we are on Windows, this is needed due to issues related to GMP 
C:\dev\vcpkg> .\vcpkg.exe install yasm-tool:x86-windows 
```
```shell
C:\dev\vcpkg> .\vcpkg.exe install cgal
```

#####  This setup is less convenient than I initially expected for three reasons:
**1 -** CMake must be configured and built using the following command lines:
```shell
cmake -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
cmake --build .
```
**2 -** The `libigl` library installed via vcpkg does not work well.

**3 -** In the `CMakeLists.txt` file provided in the `CGAL` example project, the following line is present:

```cmake
create_single_source_cgal_program("main.cpp")
```

This automatically links CGAL headers and creates the `main` target. However, this link is relative, preventing me from relocating my executable as I usually do to easily access `.off` files in the `/data` directory.

To solve this issue, I added the following command:

```cmake
add_custom_command(TARGET main
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:main> ${CMAKE_CURRENT_SOURCE_DIR})
```

As a result, my `main.exe` executable is now always placed in `/build/Debug/main.exe`.

#### II - libigl

For libigl, I used a `libigl.cmake` file placed in the `/cmake` directory, which fetches the project from Git:

```cmake
# libigl.cmake

if(TARGET igl::core)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.5.0
)
FetchContent_MakeAvailable(libigl)
```

To use it, add the following line to the `CMakeLists.txt` file:

```cmake
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(libigl)
```

Then, to make `libigl` modules accessible and link them to the project:

```cmake
igl_include(glfw)
...
target_link_libraries(main PUBLIC igl::glfw)    
```

#### III - Delaunator
Because CGAL is complex to use, I tried an alternative library for computing Delaunay triangulation.

#### IV - Compilation and run

Here is the `CMakeLists.txt` file: 

```cmake
# CMakeLists.txt

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
target_link_libraries(main PUBLIC igl::glfw delaunator) # link libigl and delaunator to the project file
```

To compile the code from the project's root directory:

```shell
C:\path\to\project> mkdir build
C:\path\to\project> cd build
C:\path\to\project\build> cmake -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
C:\path\to\project\build> cmake --build .
C:\path\to\project\build> cd Debug
C:\path\to\project\build\Debug> .\main.exe
```

#### V - Be Aware of the Following Issues
**1 -** The type names `Delauday` and `VoronoiDiagram` are already used, so typedefs should be defined as follows:

```cpp
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
...
typedef CGAL::Voronoi_diagram_2<Delaunay, AT, AP> VoronoiDiagram;
```
but 
```cpp
typedef CGAL::Delaunay_triangulation_2<K> DT;
...
typedef CGAL::Voronoi_diagram_2<DT,AT,AP> VD;
```

**2 -** Declaring variables inside `main()` versus outside it affects behavior, particularly when modifying `viewer.callback_key_down`.

Initially:

```cpp
...
int main(int argc, char *argv[]){
    Eigen::MatrixXd V, V_uv;
    Eigen::MatrixXi F;
    ...
    viewer.callback_key_pressed = 
        [&V, &V_uv, &F](igl::opengl::glfw::Viewer& viewer, unsigned int key, int /*mod*/){}
}
```

Modified version:

```cpp
Eigen::MatrixXd V, V_uv;
Eigen::MatrixXi F;

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier){
    ... //using V, V_uv, F
}

int main(int argc, char *argv[]){
    ...
    viewer.callback_key_pressed = &key_down;
}
```

Now, when a function has a parameter `Eigen::MatrixXd V`, it is unclear whether it refers to the global or local `V`:


```cpp
Eigen::MatrixXd V
...
void rundomFonction(Eigen::MatrixXd V){
    // witch V is used ? 
}
```

#### VI - Visualisation

don't forgot to Toggle filles faces with T and wireframe with L to look at meshes.