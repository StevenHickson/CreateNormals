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
void CreateNormalsPython(float *camera_params,
                   int camera_params_length,
                   float *normal_params,
                   int normal_params_length,
                   int* flat_labels,
                   int flat_labels_length,
                   int width,
                   int height,
                   unsigned short int *depth,
                   unsigned short int *labels,
                   float *output);
bool CreateNormals(const std::vector<float> &camera_params,
                   const std::vector<float> &normal_params,
                   const std::vector<bool> &flat_labels,
                   const std::string &depth_file,
                   const std::string &labels_file,
                   const std::string &output_file);
extern "C" {
  void CalculateNormals(float *camera_params,
                   int camera_params_length,
                   float *normal_params,
                   int normal_params_length,
                   int* flat_labels,
                   int flat_labels_length,
                   int width,
                   int height,
                   unsigned short int *depth,
                   unsigned short int *labels,
                   float *output) {
    return CreateNormalsPython(camera_params,
                               camera_params_length,
                               normal_params,
                               normal_params_length,
                               flat_labels,
                               flat_labels_length,
                               width,
                               height,
                               depth,
                               labels,
                               output);
  }
}
