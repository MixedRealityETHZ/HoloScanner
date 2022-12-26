//
// Created by Noureddine Gueddach on 21/11/2022.
//

#include <memory>
#include <filesystem>
#include "open3d/Open3D.h"
#include "Registrator.h"

using namespace open3d;
using std::cout;
using std::endl;
namespace fs = std::filesystem;

void Registrator::reset() {
    m_pcd = nullptr;
}

bool Registrator::getReconstructedPCDInEigenFormat(Eigen::MatrixXd &mat) {
    if (!m_pcd) return false;
    {
        std::lock_guard guard(pcdMatrixLock);
        mat = pcdMatrix;
    }
    return true;
}

Eigen::Matrix4d Registrator::getTransformation(const geometry::PointCloud &source,
                                               const geometry::PointCloud &target, Eigen::Matrix6d &InfoMat,
                                               const double kernel_param) const {

    int nb_iterations = 600;

    double voxel_size = 0.01;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; ++i) {
        auto source_down = source.VoxelDownSample(voxel_size);
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto target_down = target.VoxelDownSample(voxel_size);
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto loss = pipelines::registration::TukeyLoss(kernel_param);
        auto kernel = loss.k_;
        auto result = pipelines::registration::RegistrationGeneralizedICP(
                *source_down, *target_down, m_max_corr_dist_transformation, T,
                pipelines::registration::TransformationEstimationForGeneralizedICP(kernel),
                pipelines::registration::ICPConvergenceCriteria(1e-7, 1e-7, nb_iterations));
        T = result.transformation_;
        voxel_size /= 2;
    }
    InfoMat = pipelines::registration::GetInformationMatrixFromPointClouds(source, target,
                                                                           m_max_corr_dist_transformation, T);
    return T;
}

bool Registrator::isRegistrationSuccessful(const geometry::PointCloud &pcd, const Eigen::Matrix4d &T) const {
    auto result = pipelines::registration::EvaluateRegistration(pcd, *m_pcd, m_max_corr_dist_evaluation, T);
    auto correspondance_set = result.correspondence_set_;
    auto fitness = result.fitness_;  // corresponds to: correspondance_set.size() / pcd.points_.size()
    auto rmse = result.inlier_rmse_;
    // bool most_of_pcd_is_inlier = correspondance_set.size() >= 0.8 * pcd.points_.size(); //same as fitness
    cout << fitness << " " << rmse << " " << endl;
    bool high_fitness = fitness > m_min_fitness;
    bool low_rmse = rmse < m_max_rmse;
    return high_fitness && low_rmse;
}

#ifdef USE_DBSCAN
bool Registrator::mergePCD(const PCD &pcd_, std::vector<DirectX::XMVECTOR> handMesh[2])
#else

bool Registrator::mergePCD(const PCD &pcd_)
#endif
{
    if (pcd_.size() < 1500) return false;
    auto pcd = geometry::PointCloud(pcd_);
    if (m_pcd == nullptr) {  // first registration is always successful as it initializes the point cloud
        m_pcd = std::make_shared<geometry::PointCloud>(pcd);
        return true;
    }

    // DBSCAN
    // bool dbscan = false; // Note: DBSCAN is too slow for real-time (could be use) for a final pass though
    // if(dbscan) {
    //     std::vector<int> indices = pcd.ClusterDBSCAN(0.1, 0.7 * pcd.points_.size());
    //     PCD valid_points;
    //     for(int i = 0; i < indices.size(); i++) {
    //         if(indices[i] != -1)
    //             valid_points.push_back(pcd.points_[i]);
    //     }
    //     pcd.points_ = valid_points;
    //}
#ifdef USE_DBSCAN
    std::vector<size_t> index = std::get<1>(pcd.RemoveStatisticalOutliers(16, 0.8));
    pcd = *pcd.SelectByIndex(index);
    std::vector<int> labels = pcd.ClusterDBSCAN(0.013, 64);
    std::set<int> labels_unique;
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            labels_unique.insert(labels[i]);
        }
    }
    // mean of the hand points
    Eigen::Vector3d hand_center(0.0, 0.0, 0.0);
    for(size_t i=0; i<2; ++i){
        for(size_t j=0; j<handMesh[i].size(); ++j){
            hand_center[0] -= handMesh[i][j][1];
            hand_center[1] -= handMesh[i][j][0];
            hand_center[2] += handMesh[i][j][2];
        }
    }

    hand_center = hand_center / (2*handMesh[0].size());
    // some recording for each cluster
    std::vector<size_t> labels_num(labels_unique.size(), 0);
    std::vector<double> distances(labels_unique.size(), std::numeric_limits<double>::max());
    std::vector<std::vector<size_t>> labels_index;
    for (int i = 0; i < labels_unique.size(); ++i) {
        labels_index.push_back(std::vector<size_t>());
    }
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0){
            double distance = (pcd_[i] - hand_center).squaredNorm();
            if(distance < distances[labels[i]]){
                distances[labels[i]] = distance;
            }
            labels_num[labels[i]]++;
            labels_index[labels[i]].push_back(i);
        }
    }
    size_t argclose = std::distance(distances.begin(), std::min_element(distances.begin(),distances.end()));
    size_t argmax = std::distance(labels_num.begin(), std::max_element(labels_num.begin(), labels_num.end()));
    // usually argclose == argmax,  but argclose now
    pcd = *pcd.SelectByIndex(labels_index[argclose]);
#endif

    // Compute the transformation between the current and global point cloud
    Eigen::Matrix6d InfoMat;
    double kernel_param = 0.1;
    Eigen::Matrix4d T = getTransformation(pcd, *m_pcd, InfoMat, kernel_param);

    // Evaluate the registration
    bool success = isRegistrationSuccessful(pcd, T);
    // If not successful, keep the global point cloud as is, wait for the user to realign
    if (!success) return false;

    *m_pcd = m_pcd->Transform(T.inverse());  // bring the global point cloud into the reference of the current frame
    *m_pcd += pcd;  // merge the current frame to the global point cloud
    m_pcd = m_pcd->VoxelDownSample(0.0025);  // downsample for performance

    updatePCDMatrixFromPCD();

    return true;
}

void Registrator::updatePCDMatrixFromPCD() {// Convert to Eigen matrix format
    Eigen::MatrixXd mat;
    mat.resize((long) m_pcd->points_.size(), 3);
    for (int i = 0; i < m_pcd->points_.size(); i++) {
        mat.row(i) = m_pcd->points_[i].transpose();
    }

    {
        std::lock_guard guard(pcdMatrixLock);
        pcdMatrix = std::move(mat);
    }
}

void Registrator::manualUpdatePCD(const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                                  std::vector<long unsigned int> &index) const {
    PCD points = pcd->points_;
    PCD new_points;
    for (int i = 0; i < index.size(); ++i) {
        new_points.push_back(points[index[i]]);
    }
    pcd->points_ = new_points;
}

void Registrator::saveReconstructedMesh() {
    if (!m_pcd) return;

    // DATA_FOLDER defined in CMakeLists.txt
    const fs::path dataFolder = fs::path(DATA_FOLDER) / currentTimeString();
    if (!fs::exists(dataFolder)) {
        fs::create_directories(dataFolder);
    }

    // Save PCD before possibly FINAL_DBSCAN
    auto pcdFilename = dataFolder / "Final.xyz";
    std::cout << "Writing PCD to " << pcdFilename << std::endl;
    open3d::io::WritePointCloudToXYZ(pcdFilename.string(), *m_pcd, {});

#ifdef FINAL_DBSCAN
    std::vector<size_t> index = std::get<1>(m_pcd->RemoveStatisticalOutliers(16, 0.8));
    manualUpdatePCD(m_pcd, index);
    std::vector<int> labels = m_pcd->ClusterDBSCAN(0.013, 64);
    std::set<int> labels_unique;
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            labels_unique.insert(labels[i]);
        }
    }
    // some recording for each cluster
    std::vector<size_t> labels_num(labels_unique.size(), 0);
    std::vector<std::vector<size_t>> labels_index;
    for (int i = 0; i < labels_unique.size(); ++i) {
        labels_index.push_back(std::vector<size_t>());
    }
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            labels_num[labels[i]]++;
            labels_index[labels[i]].push_back(i);
        }
    }
    size_t argmax = std::distance(labels_num.begin(), std::max_element(labels_num.begin(), labels_num.end()));
    // auto m_pcd = m_pcd->SelectByIndex(labels_index[argmax]);
    manualUpdatePCD(m_pcd, labels_index[argmax]);
    // m_pcd = std::make_shared<open3d::geometry::PointCloud>(pcd);

    updatePCDMatrixFromPCD();

    // Save PCD before possibly FINAL_DBSCAN
    auto afterSBScanPCDFilename = dataFolder / "AfterDBScan.xyz";
    std::cout << "Writing post-DBScan PCD to " << afterSBScanPCDFilename << std::endl;
    open3d::io::WritePointCloudToXYZ(afterSBScanPCDFilename.string(), *m_pcd, {});
#endif

    /*std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    std::vector<double> densities;

    float scale = 3;
    m_pcd->EstimateNormals();
    std::tie(mesh, densities) = geometry::TriangleMesh::CreateFromPointCloudPoisson(*m_pcd, 8UL, 0, scale);

    // Save final mesh
    auto meshFilename = dataFolder / "Poisson.ply";
    std::cout << "Writing mesh to " << meshFilename << std::endl;
    io::WriteTriangleMesh(meshFilename.string(), *mesh);*/
}

std::string Registrator::currentTimeString() {
    // Reference: https://stackoverflow.com/questions/24686846/get-current-time-in-milliseconds-or-hhmmssmmm-format

    using namespace std::chrono;

    // Get current time
    auto now = system_clock::now();

    // Get number of milliseconds for the current second
    // (remainder after division into seconds)
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    // Convert to std::time_t in order to convert to std::tm (broken time)
    auto timer = system_clock::to_time_t(now);

    // Convert to broken time
    std::tm bt = *std::localtime(&timer);

    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y%m%d_%H%M%S");

    return oss.str();
}