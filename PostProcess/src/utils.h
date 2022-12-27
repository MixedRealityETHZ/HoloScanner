#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <Eigen/Dense>
#include <vector>

bool string_contains(const std::string s, const char c);

bool ends_with(std::string& s, const std::string& ending);

std::string find_file_ending_with(const std::string& folder, const std::string& ending);

std::vector<Eigen::Vector3d> eigen_to_vec(const Eigen::MatrixXd& mat);

Eigen::MatrixXd vec_to_eigen(const std::vector<Eigen::Vector3d>& vec);

inline void endian_swap(uint16_t& x) { x = (x>>8) | (x<<8); }

#endif