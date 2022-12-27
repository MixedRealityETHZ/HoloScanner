#include <filesystem>
#include "utils.h"
#include <iostream>

bool string_contains(const std::string s, const char c) {
  return s.find(c) != std::string::npos;
}

bool ends_with(std::string& s, const std::string& ending) {
    if (s.length() >= ending.length()) {
        return s.compare(s.length() - ending.length(), ending.length(), ending) == 0;
    } else {
        return false;
    }
}

std::string find_file_ending_with(const std::string& folder, const std::string& ending) {
  for (const auto & entry : std::filesystem::directory_iterator(folder)) {
    std::string path = entry.path();
    if(ends_with(path, ending)) {
        return path;
    }
  }
  throw std::runtime_error("Did not find any file ending with '" + ending + "'.");
}

std::vector<Eigen::Vector3d> eigen_to_vec(const Eigen::MatrixXd& mat) {
  std::vector<Eigen::Vector3d> vec;
  for(int i = 0; i < mat.rows(); i++) {
    vec.push_back(mat.row(i));
  }
  return vec;
}

Eigen::MatrixXd vec_to_eigen(const std::vector<Eigen::Vector3d>& vec) {
  Eigen::MatrixXd Mat(vec.size(), 3);
  for(int i = 0; i < vec.size(); i++) {
    Mat.row(i) = vec[i];
  }
  return Mat;
}