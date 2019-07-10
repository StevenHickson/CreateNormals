#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

bool ReadParameters(const std::string &filename,
                    std::vector<float> *camera_params,
                    std::vector<float> *normal_params,
                    std::vector<bool> *flat_labels);
void CreateNormals(const std::vector<float> &camera_params,
                   const std::vector<float> &normal_params,
                   const std::vector<bool> &flat_labels,
                   const cv::Mat &depth,
                   const cv::Mat &labels,
                   cv::Mat *output);
bool CreateNormals(const std::vector<float> &camera_params,
                   const std::vector<float> &normal_params,
                   const std::vector<bool> &flat_labels,
                   const std::string &depth_file,
                   const std::string &labels_file,
                   const std::string &output_file);
