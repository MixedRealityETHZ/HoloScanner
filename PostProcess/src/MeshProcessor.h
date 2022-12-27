//
// Created by Noureddine Gueddach on 11/12/2022.
//

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>
#include <igl/barycenter.h>
#include <igl/cotmatrix.h>
#include <igl/write_triangle_mesh.h>
#include <igl/copyleft/marching_cubes.h>
#include <queue>

using PCD = std::vector<Eigen::Vector3d>;

class MeshProcessor {
public:

    //Using default params
    MeshProcessor() {}

    //Run Implicit Laplacian Smoothing
    void smoothMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Eigen::MatrixXd& V_impLap, float delta) {
        V_impLap = V;
        Eigen::SparseMatrix<double> L;
        igl::cotmatrix(V, F, L);

        // Compute Mass Matrix
        Eigen::SparseMatrix<double> M;
        igl::massmatrix(V_impLap, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
        // Solve (M-delta*L) U = M*U
        const auto& S = (M - delta * L);
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver(S);
        assert(solver.info() == Eigen::Success);
        V_impLap = solver.solve(M * V_impLap).eval();
        // Compute centroid and subtract (also important for numerics)
        Eigen::VectorXd dblA;
        igl::doublearea(V, F, dblA);
        double area = 0.5 * dblA.sum();
        Eigen::MatrixXd BC;
        igl::barycenter(V_impLap, F, BC);
        Eigen::RowVector3d centroid(0, 0, 0);
        for(int i = 0; i < BC.rows(); i++) {
            centroid += 0.5 * dblA(i) / area * BC.row(i);
        }
        V_impLap.rowwise() -= centroid;
    }

    void meshify(Eigen::MatrixXd& grid_points, Eigen::VectorXd& grid_values, 
        const Eigen::Vector3d& resolution, Eigen::MatrixXd& V, Eigen::MatrixXi& F) const {
        unsigned int resx = resolution.x();
        unsigned int resy = resolution.y();
        unsigned int resz = resolution.z();
        igl::copyleft::marching_cubes(grid_values, grid_points, resx, resy, resz, V, F);
    }
    

    void saveMesh(const std::string& path, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) const {
        igl::write_triangle_mesh(path, V, F);
    }

    bool flood(const PCD& pcd, const Eigen::Vector3d& resolution, 
        Eigen::MatrixXd& grid_points, Eigen::VectorXd& grid_values, PCD& inner_points_vec) const {
        using std::vector;
        using std::cout;
        using std::endl;
        //#########################################
        //Note: Assumes the point cloud is closed #
        //#########################################

        //Discretize points and create a thick boundary out of them
        Eigen::Vector3d min, max;
        getMinMaxBounds(pcd, min, max);
        Eigen::Vector3d extents = max - min;
        Eigen::Vector3d extents_inv = Eigen::Vector3d(1/extents.x(), 1/extents.y(), 1/extents.z());
        Eigen::Vector3d resolution_inv = Eigen::Vector3d(1/resolution.x(), 1/resolution.y(), 1/resolution.z());
        vector<vector<vector<uint8_t>>>* occupancy = new vector<vector<vector<uint8_t>>>(resolution.x(), 
            vector<vector<uint8_t>>(resolution.y(), vector<uint8_t>(resolution.z(), 0)));
        for(int i = 0; i < pcd.size(); i++) {
            Eigen::Vector3d point = (pcd[i] - min).cwiseProduct(extents_inv).cwiseProduct(resolution);
            int x_ = (int)point.x();
            int y_ = (int)point.y();
            int z_ = (int)point.z();
            //Set each neighboring as a boundary point
            //This thickness is necessary so that we don't accidentally cross the boundary
            //when flooding
            for(int i = -1; i <= 1; i++) {
                for(int j = -1; j <= 1; j++) {
                    for(int k = -1; k <= 1; k++) {
                        int x = x_ + i;
                        int y = y_ + j;
                        int z = z_ + k;
                        if(inBounds(x, y, z, resolution)) {
                            (*occupancy)[x][y][z] = 1;
                        }
                    }
                }
            }
        }
        
        //#######################################################################
        //Note: The key assumption here is that the center point will be inside #
        //the shape. If this does not hold then nothing works.                  #
        //#######################################################################

        //Compute the center of the point cloud.
        Eigen::Vector3d center = (getCenter(pcd) - min)
            .cwiseProduct(extents_inv).cwiseProduct(resolution);
        std::queue<Eigen::Vector3i> q;
        q.push(Eigen::Vector3i((int)center.x(), (int)center.y(), (int)center.z()));
        vector<Eigen::Vector3d> inner_points;
        //Flood the inside of the shape, BFS style
        while(!q.empty()) {
            Eigen::Vector3i p = q.front();
            q.pop();
            if((*occupancy)[p.x()][p.y()][p.z()] != 0)
                continue;
            (*occupancy)[p.x()][p.y()][p.z()] = 2;
            Eigen::Vector3d w_p = Eigen::Vector3d(p.x(), p.y(), p.z())
                .cwiseProduct(resolution_inv).cwiseProduct(extents) + min;
            inner_points.push_back(w_p);
            for(int i = -1; i <= 1; i++) {
                for(int j = -1; j <= 1; j++) {
                    for(int k = -1; k <= 1; k++) {
                        int x = p.x() + i;
                        int y = p.y() + j;
                        int z = p.z() + k;
                        if(inBounds(x, y, z, resolution)) {
                            if(!(i == 0 && j == 0 && k == 0)) {
                                short val = (*occupancy)[x][y][z];
                                if(val == 0) {
                                    q.push(Eigen::Vector3i(x, y, z));
                                }
                            }
                        }
                    }
                }
            }
        }
        if(inner_points.size() == 0) {
            cout << "PCD SIZE: " << inner_points.size() << endl;
            return false;
        }

        //Expand the volume to compensate for the thick boundary
        vector<Eigen::Vector3i> expanded_points;
        for(int x = 0; x < resolution.x(); x++) {
            for(int y = 0; y < resolution.y(); y++) {
                for(int z = 0; z < resolution.z(); z++) {
                    bool found_neighbor = false;
                    if((*occupancy)[x][y][z] == 2)
                        continue;
                    for(int i = -1; i <= 1 && !found_neighbor; i++) {
                        for(int j = -1; j <= 1 && !found_neighbor; j++) {
                            for(int k = -1; k <= 1 && !found_neighbor; k++) {
                                int x_ = x + i;
                                int y_ = y + j;
                                int z_ = z + k;
                                if(inBounds(x_, y_, z_, resolution)) {
                                    if((*occupancy)[x_][y_][z_] == 2) {
                                        expanded_points.push_back(Eigen::Vector3i(x, y, z));
                                        found_neighbor = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        for(auto p : expanded_points) {
            (*occupancy)[p.x()][p.y()][p.z()] = 2;
        }

        bool is_successful = true;
        //Put the result in the appropriate data structures in preparation for marching cubes
        int N = (int)resolution.x() * (int)resolution.y() * (int)resolution.z();
        grid_points.resize(N, 3);
        grid_values.resize(N);
        grid_values.setOnes(); //points are outside the shape by default
        int ii = 0;
        inner_points_vec.clear();
        for(int x = 0; x < resolution.x(); x++) {
            for(int y = 0; y < resolution.y(); y++) {
                for(int z = 0; z < resolution.z(); z++, ii++) {
                    Eigen::Vector3d w_p = Eigen::Vector3d(x, y, z)
                        .cwiseProduct(resolution_inv).cwiseProduct(extents) + min;
                    grid_points.row(ii) = w_p.transpose();

                    bool found_neighbor = false;
                    if((*occupancy)[x][y][z] == 2) {
                        grid_values(ii) = -2; //inner point
                        inner_points_vec.push_back(w_p);
                        if(isCornerPoint(x, y, z, resolution))
                            is_successful = false;
                        continue;
                    }
                    for(int i = -1; i <= 1 && !found_neighbor; i++) {
                        for(int j = -1; j <= 1 && !found_neighbor; j++) {
                            for(int k = -1; k <= 1 && !found_neighbor; k++) {
                                int x_ = x + i;
                                int y_ = y + j;
                                int z_ = z + k;
                                if(inBounds(x_, y_, z_, resolution)) {
                                    if((*occupancy)[x_][y_][z_] == 2) {
                                        found_neighbor = true;
                                    }
                                }
                            }
                        }
                    }
                    if(found_neighbor) {
                        grid_values(ii) = -0.5; //inner point close to the surface
                        inner_points_vec.push_back(w_p);
                        if(isCornerPoint(x, y, z, resolution))
                            is_successful = false;
                    }
                }
            }
        }
        return is_successful;
    }

private:
    void getMinMaxBounds(const PCD& pcd, Eigen::Vector3d& min, Eigen::Vector3d& max) const {
        double minx, miny, minz, maxx, maxy, maxz;
        minx = miny = minz = std::numeric_limits<double>::max() - 1;
        maxx = maxy = maxz = -(std::numeric_limits<double>::max() - 1);
        for(const auto p : pcd) {
            minx = std::min(minx, p.x());
            miny = std::min(miny, p.y());
            minz = std::min(minz, p.z());
            maxx = std::max(maxx, p.x());
            maxy = std::max(maxy, p.y());
            maxz = std::max(maxz, p.z());
        }
        min = Eigen::Vector3d(minx, miny, minz);
        max = Eigen::Vector3d(maxx, maxy, maxz);
    }

    Eigen::Vector3d getCenter(const PCD& pcd) const {
        Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
        if(pcd.size() == 0) 
            return center;
        for(const auto p : pcd) {
            center += p;
        }
        return center / pcd.size();
    }

    inline bool inBounds(int x, int y, int z, const Eigen::Vector3d& resolution) const {
        return  x >= 0 && x < resolution.x() && 
                y >= 0 && y < resolution.y() && 
                z >= 0 && z < resolution.z();
    }

    inline bool isCornerPoint(int x, int y, int z, const Eigen::Vector3d& resolution) const {
        return  x == 0 && y == 0 && z == 0 ||
                x == 0 && y == 0 && z == resolution.z() ||
                x == 0 && y == resolution.y() && z == 0 ||
                x == 0 && y == resolution.y() && z == resolution.z() ||
                x == resolution.x() && y == 0 && z == 0 ||
                x == resolution.x() && y == 0 && z == resolution.z() ||
                x == resolution.x() && y == resolution.y() && z == 0 ||
                x == resolution.x() && y == resolution.y() && z == resolution.z();
    }

};