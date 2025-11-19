#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <cstdlib>  // Pour rand() et srand()
#include <ctime>    // Pour time()
#include <cmath>

#include <queue>
#include <unordered_set>
#include <unordered_map>

// delaunator library
#include "delaunator.hpp"

// CGAL library
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Voronoi_diagram_2.h>

// libigl library
#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846  // Définition de M_PI si non définie
#endif

// Définition des types CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                                    VD;
typedef DT::Point Point;

typedef AT::Site_2                    Site_2;
typedef AT::Point_2                   Point_2;

typedef VD::Locate_result             Locate_result;
typedef VD::Vertex_handle             Vertex_handle;
typedef VD::Face_handle               Face_handle;
typedef VD::Halfedge_handle           Halfedge_handle;
typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;

typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::MatrixXi MatrixXi;


struct Edge_ {
    int v0, v1;
    bool operator==(const Edge_& other) const;
};

struct Edge {
    unsigned int a, b;
    Edge(unsigned int c, unsigned int d);
    bool operator<(Edge const & o) const;
    bool operator==(Edge const & o) const;
};

struct Triangle {
    int v0, v1, v2;
};

// Function Declarations
int randomInt(int x) {};
double triangleArea(const Eigen::RowVector3d &A, 
    const Eigen::RowVector3d &B, 
    const Eigen::RowVector3d &C) {};
double triangleArea2D(const Eigen::RowVector2d &A, 
    const Eigen::RowVector2d &B, 
    const Eigen::RowVector2d &C) {};
int isInFeat(const MatrixXi &feat, const Edge &edge) {};
void arbreCouvrant(
    std::vector<std::pair<int, int>> &FacesP,       // first index: parent, second: child
    std::vector<std::pair<int, int>> &FeaturesP,    // first index: parent, second: child
    const MatrixXi &F,
    const MatrixXi &feat,
    const std::map<unsigned int, std::set<unsigned int>> &adjacentVertices,
    const std::map<Edge, std::set<unsigned int>> &trianglesOnEdge) {};
void errorPropagation(
    std::vector<int> &nb_v_in_Tri, 
    std::vector<int> &nb_v_in_feat_edge, 
    const std::vector<float> &faceDens, 
    const std::vector<float> &featEdgeDens, 
    const MatrixXd &V_uv, 
    const MatrixXi &F, 
    const std::vector<std::pair<int, int>> &path){};
std::map<Edge, std::set<unsigned int>>  createMapTriangleOnEdge(const MatrixXi &F){};
std::map<unsigned int, std::set<unsigned int>> createMapAdjacentVertices(const MatrixXi &F) {};
void createFeature(
    MatrixXi &feat, 
    Eigen::VectorXi &corn, 
    const MatrixXd &V,  // V.clos() = 3
    const MatrixXi &F, 
    const Eigen::VectorXi &bnd, 
    double threshold = 50.0){};
void uniformeDensity(
    std::vector<float> &faceDens, 
    std::vector<float> &featEdgeDens, 
    const MatrixXd &V, 
    const MatrixXd &V_, 
    const MatrixXi &F, 
    const MatrixXi &feat, 
    const Eigen::VectorXi &corn, 
    const int &budget, 
    int ratio = 1) {};
Eigen::RowVector3d randomBarycentric(std::mt19937 &gen) {};
void initialResample(
    MatrixXd &B,
    const MatrixXd &V, 
    const MatrixXi &F, 
    const std::vector<int> &nb_v_in_Tri,
    const std::vector<int> &nb_v_in_feat_edge,
    const MatrixXi &feat, 
    const Eigen::VectorXi &corn) {};
bool isPointInCircumcircle(const Eigen::Vector2d& p, 
    const Eigen::Vector2d& a, 
    const Eigen::Vector2d& b, 
    const Eigen::Vector2d& c) {};
void computeHandDelaunayTriangulation(const Eigen::MatrixXd& V_uv, Eigen::MatrixXi& F_out) {};
void computeDelaunatorDelaunayTriangulation(MatrixXi& F, const MatrixXd& V, const MatrixXi &feat){};
DT computeCGALDelaunayTriangulation(MatrixXi& F, const MatrixXd& V, const MatrixXi &feat){};
VD computeCGALVoronoiTessellation(const DT &dt, MatrixXd &P, MatrixXi &Fp) {};
bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {};
int main() {};

#endif