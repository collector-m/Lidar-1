#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <fstream>

struct PipelineParams {

  // filtering parameters
  float filter_resolution;
  Eigen::Vector4f crop_min_point;
  Eigen::Vector4f crop_max_point;

  // RANSAC parameters
  int max_iterations;
  float distance_threshold;

  // clustering parameters
  float cluster_tolerance;
  int cluster_min_size;
  int cluster_max_size;

  // visualization options
  bool render_input_cloud;
  bool render_filtered_cloud;
  bool render_obstacles;
  bool render_plane;
  bool render_clusters;
  bool render_boxes;

  bool fromFile(const std::string &filePath) {
    std::ifstream file_(filePath);

    if (!file_.is_open()) {
      std::cerr << "Params file not found!" << std::endl;
      return false;
    }

    std::string line_;
    int i = 0;
    while (getline(file_, line_)) {
      if (line_[0] == '#') continue;
      if (line_.empty()) continue;

      std::stringstream check1(line_);
      std::string paramName;

      check1 >> paramName;
      if (paramName == "filter_resolution:") {
        check1 >> filter_resolution;
      } else if (paramName == "crop_min_point:") {
        double x, y, z, i;
        check1 >> x >> y >> z >> i;
        crop_min_point = Eigen::Vector4f(x, y, z, i);
      } else if (paramName == "crop_max_point:") {
        double x, y, z, i;
        check1 >> x >> y >> z >> i;
        crop_max_point = Eigen::Vector4f(x, y, z, i);
      } else if (paramName == "max_iterations:") {
        check1 >> max_iterations;
      } else if (paramName == "distance_threshold:") {
        check1 >> distance_threshold;
      } else if (paramName == "cluster_tolerance:") {
        check1 >> cluster_tolerance;
      } else if (paramName == "cluster_min_size:") {
        check1 >> cluster_min_size;
      } else if (paramName == "cluster_max_size:") {
        check1 >> cluster_max_size;
      } else if (paramName == "render_input_cloud:") {
        check1 >> render_input_cloud;
      } else if (paramName == "render_filtered_cloud:") {
        check1 >> render_filtered_cloud;
      } else if (paramName == "render_obstacles:") {
        check1 >> render_obstacles;
      } else if (paramName == "render_plane:") {
        check1 >> render_plane;
      } else if (paramName == "render_clusters:") {
        check1 >> render_clusters;
      } else if (paramName == "render_boxes:") {
        check1 >> render_boxes;
      } else {
        std::cerr << "Unrecognized pipeline parameter: " << paramName << std::endl;
        assert(0);
      }
    }
    file_.close();
    return true;
  }

};

#endif