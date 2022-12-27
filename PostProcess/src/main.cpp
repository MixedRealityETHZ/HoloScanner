#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/slice.h>
#include <imgui/imgui.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <limits.h>
#include <math.h>
#include <map>

#include "utils.h"
#include "MeshProcessor.h"

#include <open3d/geometry/PointCloud.h>

using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;

Viewer viewer;
MeshProcessor meshProcessor;

Eigen::MatrixXd NoisyPoints;
Eigen::MatrixXd OutlierPoints;
Eigen::MatrixXd FloodPoints;

Eigen::MatrixXd MC_V;
Eigen::MatrixXd SmoothV;
Eigen::MatrixXi F;
Eigen::MatrixXd N;

Eigen::MatrixXd GridPoints;
Eigen::VectorXd GridValues;

PCD noisy_points;
PCD outlier_points;
PCD flood_points;

bool redraw = false;
std::string input_file;
int min_res = 10, max_res = 100;

enum DisplayMode { NOISY_POINTS, POINT_WITHOUT_OUTLIERS, GRID_POINTS, FLOOD_POINTS, MC_MESH, SMOOTH_MESH };
DisplayMode display_mode = SMOOTH_MESH;

//Tweakable parameters
int res = max_res; //grid resolution
int resx = res, resy = res, resz = res;
float delta = 0.00005; //Controls the smoothing amount
bool score_denoise = false;
int nb_neighbors = 16;
double std_ratio = 0.8;

///////////////////////////////

void setPointsToVisualize(const Eigen::MatrixXd& points, const Eigen::Vector3d color);
void setMeshToShow(const Eigen::MatrixXd& V);
void showNoisyPoints();
void showPointsWithoutOutliers();
void showGridPoints();
void showFloodPoints();
void showMCMesh();
void showSmoothMesh();

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers) {
  if (key == '1') {
    
  }
  return false;
}

bool callback_pre_draw(Viewer& viewer) {
  if(!redraw)
    return false;
  switch(display_mode) {
    case NOISY_POINTS:
      showNoisyPoints();
    break;
    case POINT_WITHOUT_OUTLIERS:
      showPointsWithoutOutliers();
    break;
    case GRID_POINTS:
      showGridPoints();
    break;
    case FLOOD_POINTS:
      showFloodPoints();
    break;
    case MC_MESH:
      showMCMesh();
    break;
    case SMOOTH_MESH:
      showSmoothMesh();
    break;
    default:
      cout << "Unknown display mode" << endl; 
  }
  redraw = false;
  return false;
}

void read_data(const std::string& path, PCD& points) {
  try {
    //Read the point cloud
    std::ifstream istream(path, std::ifstream::in);
    points.clear();
    for(std::string line; std::getline(istream, line); ) { //read stream line by line
      std::istringstream in(line); //make a stream for the line itself
      float x, y, z;
      in >> x >> y >> z;
      points.push_back(Eigen::Vector3d(x, y, z));
    }
    cout << "Finished reading the pcd from disk" << endl;
  } catch (const char* msg) {
    std::cerr << msg << endl;
    exit(-1);
  }
}

// void denoise(const std::string& path_to_denoise, PCD& denoised_points) {
//   std::string denoised_file_name = "denoised.xyz";
//   try {
//     cout << "Calling Python-based score-denoise script..." << endl;
//     std::string command = std::string("cd ../ext/score-denoise && python test_single.py --input_xyz ../") 
//       + path_to_denoise + " --output_xyz ../../" + denoised_file_name;
//     int error_code = system(command.c_str());
//     cout << "Finished denoising!" << endl;

//     //Read back the denoised point cloud
//     cout << "C++ takes over. Reading the denoised point cloud..." << endl;
//     denoised_points.clear();
//     read_data("../" + denoised_file_name, denoised_points);
//   } catch (const char* msg) {
//       std::cerr << msg << endl;
//   }
// }

void slicePCD(open3d::geometry::PointCloud& pcd,
                     std::vector<long unsigned int>& index) {
    PCD new_points;
    for (int i = 0; i < index.size(); ++i) {
        new_points.push_back(pcd.points_[index[i]]);
    }
    pcd.points_ = new_points;
}

void remove_outliers(const PCD& points_with_outliers, PCD& points_without_outliers) {
  open3d::geometry::PointCloud pcd(points_with_outliers);

  if(nb_neighbors <= 0 || std_ratio <= 0) {
    std::cout << "Nb neighbors and std_ratio must be positive" << endl;
  } else {
    std::vector<size_t> index = std::get<1>(pcd.RemoveStatisticalOutliers(nb_neighbors, std_ratio));
    slicePCD(pcd, index);
  }

  points_without_outliers = pcd.points_;

  // std::vector<int> labels = pcd.ClusterDBSCAN(0.013, 64);
  // std::set<int> labels_unique;
  // for (int i = 0; i < labels.size(); ++i) {
  //   if (labels[i] >= 0) {
  //     labels_unique.insert(labels[i]);
  //   }
  // }
  // // some recording for each cluster
  // std::vector<size_t> labels_num(labels_unique.size(), 0);
  // std::vector<std::vector<size_t>> labels_index;
  // for (int i = 0; i < labels_unique.size(); ++i) {
  //   labels_index.push_back(std::vector<size_t>());
  // }
  // for (size_t i = 0; i < labels.size(); ++i) {
  //   if (labels[i] >= 0) {
  //     labels_num[labels[i]]++;
  //     labels_index[labels[i]].push_back(i);
  //   }
  // }
  // size_t argmax = std::distance(labels_num.begin(), std::max_element(labels_num.begin(), labels_num.end()));
  // slicePCD(pcd, labels_index[argmax]);
    
}

void setPointsToVisualize(const Eigen::MatrixXd& points, const Eigen::Vector3d color) {
  viewer.data().clear();
  viewer.data().add_points(points, color.transpose());
  viewer.core().align_camera_center(points);
}

void showNoisyPoints() {
  setPointsToVisualize(NoisyPoints, Eigen::Vector3d(0.3, 0, 0.5));
}

void showPointsWithoutOutliers() {
  setPointsToVisualize(OutlierPoints, Eigen::Vector3d(0.15, 0.15, 0.4));
}

void showGridPoints() {
  Eigen::MatrixXd Color(GridPoints.rows(), 3);
  for(int i = 0; i < Color.rows(); i++) {
    if(GridValues(i) < 0) {
      Color.row(i) = Eigen::RowVector3d(0, 1, 0);
    } else {
      Color.row(i) = Eigen::RowVector3d(1, 0, 0);
    }
  }
  viewer.data().clear();
  viewer.data().add_points(GridPoints, Color);
  viewer.core().align_camera_center(GridPoints);
}

void showFloodPoints() {
  setPointsToVisualize(FloodPoints, Eigen::Vector3d(0.2, 1.0, 0.8));
}

void setMeshToShow(const Eigen::MatrixXd& V) {
  igl::per_face_normals(V, F, N);

  viewer.data().clear();
  viewer.data().set_mesh(V, F);
  viewer.data().show_lines = true;
  viewer.data().show_faces = true;
  viewer.data().set_normals(-N);
  viewer.core().align_camera_center(V);
}

void showMCMesh() {
  setMeshToShow(MC_V);
}

void showSmoothMesh() {
  setMeshToShow(SmoothV);
}

int nb_runs = 0;
void runPipeline(const DisplayMode from) {
  cout << nb_runs << "==================== RUNNING PIPELINE (" << nb_runs << ") ====================" << endl;
  nb_runs++;
  redraw = true;

  Eigen::Vector3d resolution(resx, resy, resz);

  if(from <= NOISY_POINTS) {
    read_data(input_file, noisy_points);
    NoisyPoints = vec_to_eigen(noisy_points);
    cout << "Reading Points: Successful." << endl;
  }

  if(from <= POINT_WITHOUT_OUTLIERS) {
    remove_outliers(noisy_points, outlier_points);
    OutlierPoints = vec_to_eigen(outlier_points);
    cout << "Outlier removal: Successful." << endl;
  }
  
  if(from <= FLOOD_POINTS) {
    int ri = resolution.x();
    bool success = false;
    while(!success && ri > min_res) {
      Eigen::Vector3d desired_resolution = Eigen::Vector3d(ri, ri, ri);
      success = meshProcessor.flood(outlier_points, desired_resolution, GridPoints, GridValues, flood_points);
      if(!success) {
        ri--;
      }
    }
    FloodPoints = vec_to_eigen(flood_points);
    if(success) {
      if(ri != (int)resolution.x()) {
        cout << "Failed with desired resolution " << (int)resolution.x() << 
          " but succeeded with resolution " << ri << ". Using " << ri << " instead..." << endl;
        resolution = Eigen::Vector3d(ri, ri, ri);
        res = resx = resy = resz = ri;
      }
      cout << "Flooding: Successful." << endl;
    } else {
      cout << "Flooding: Failed." << endl;
      cout << "Aborting pipeline." << endl;
      return;
    }
  }
  
  if(from <= MC_MESH) {
    meshProcessor.meshify(GridPoints, GridValues, resolution, MC_V, F);
    meshProcessor.saveMesh("../out/mc_mesh.ply", MC_V, F);
    cout << "Marching Cubes: Successful." << endl;
  }
  
  if(from <= SMOOTH_MESH) {
    meshProcessor.smoothMesh(MC_V, F, SmoothV, delta);
    meshProcessor.saveMesh("../out/smooth_mesh.ply", SmoothV, F);
    cout << "Smoothing: Successful." << endl;
  }
}

int main(int argc, char *argv[]) {
  if(argc < 2) {
    std::cout << "You must specify a path to an .xyz file" << std::endl;
    std::cout << "For example, run: ./postprocess ../finaldenoisedboxdb.xyz" << std::endl;
    return 0;
  }
  input_file = argv[1];

  //Setup visualizer
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);

  menu.callback_draw_viewer_menu = [&]() {
    menu.draw_viewer_menu();
    if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
      if(ImGui::Combo("Display mode", (int*)(&display_mode), "NOISY_POINTS\0POINT_WITHOUT_OUTLIERS\0GRID_POINTS\0FLOOD_POINTS\0MC_MESH\0SMOOTH_MESH\0\0")) {
        runPipeline((DisplayMode)0);
      }
    }
    if (ImGui::CollapsingHeader("Hyper parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
      if(ImGui::SliderInt("resolution XYZ", &res, min_res, max_res, "%d", 0)) {
        resx = resy = resz = res;
        runPipeline(FLOOD_POINTS);
      }
      // if(ImGui::SliderInt("resolution X", &resx, min_res, max_res, "%d", 0)) {
      //   runPipeline(FLOOD_POINTS);
      // }
      // if(ImGui::SliderInt("resolution Y", &resy, min_res, max_res, "%d", 0)) {
      //   runPipeline(FLOOD_POINTS);
      // }
      // if(ImGui::SliderInt("resolution Z", &resz, min_res, max_res, "%d", 0)) {
      //   runPipeline(FLOOD_POINTS);
      // }
      if(ImGui::InputFloat("delta", &delta, 0.0f, 0.0f, "%.8f")) {
        runPipeline(SMOOTH_MESH);
      }
      if(ImGui::SliderInt("nb neighbors", &nb_neighbors, 3, 80, "%d", 0)) {
        runPipeline(POINT_WITHOUT_OUTLIERS);
      }
      if(ImGui::InputDouble("sd ratio", &std_ratio, 0.0f, 0.0f, "%.5f")) {
        runPipeline(POINT_WITHOUT_OUTLIERS);
      }
    }
  };

  runPipeline((DisplayMode)0);

  viewer.callback_key_down = callback_key_down;
  viewer.callback_pre_draw = callback_pre_draw;
  viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
  viewer.data().point_size = 2;
  viewer.core().is_animating = true;
  Vector4f color(1, 1, 1, 1);
  viewer.core().background_color = color * 0.349f;
  viewer.launch();
}