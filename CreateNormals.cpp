#include "CreateNormals.h"
#include "ConnectedComponents.h"
#include <sys/time.h>
#include <ctime>
#include <iostream>
#include <sstream>

using namespace std;
using namespace pcl;
using namespace cv;

bool ReadParameters(const string &filename, vector<float> *params, vector<bool> *flat_labels) {
  std::ifstream file(filename);
  string line, value;
  if(!getline(file, line))
    return false;
  istringstream ss(line);
  while(getline(ss, value, ',')) {
    params->push_back(stof(value));
  }
  if(!getline(file, line))
    return false;
  istringstream ss2(line);
  while(getline(ss2, value, ',')) {
    flat_labels->push_back(value[0] == '1');
  }
  return true;
}

void GetMatFromCloud(const PointCloud<PointNormal> &cloud, Mat *img) {
	*img = Mat(cloud.height, cloud.width, CV_32FC3);
	Mat_<Vec3f>::iterator pI = img->begin<Vec3f>();
	PointCloud<PointNormal>::const_iterator pC = cloud.begin();
	while(pC != cloud.end()) {
		*pI = Vec3f(pC->normal_x, pC->normal_y, pC->normal_z);
		++pI; ++pC;
	}
}

void MakeCloudDense(PointCloud<PointXYZ> &cloud, const vector<float> &params) {
	PointCloud<PointXYZ>::iterator p = cloud.begin();
	cloud.is_dense = true;
	for(int j = 0; j < cloud.height; j++) {
		for(int i = 0; i < cloud.width; i++) {
			if(isnan(p->z)) {
				p->x = float(((float)i - params[2]) / params[0]);
				p->y = float(((float)j - params[5]) / params[4]);
				p->z = 0;
			}
			++p;
		}
	}
}

void CreatePointCloud(const Mat& input_depth,
                      const vector<float>& params,
                      PointCloud<PointXYZ>* cloud) {
  if (cloud == NULL) {
    std::cout << "cloud cannot be NULL in CreatePointCloud" << std::endl;
    return;
  }
  cloud->header.frame_id = "/cloud_frame";
  cloud->height = input_depth.rows;
  cloud->width = input_depth.cols;
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);

  PointCloud<PointXYZ>::iterator pCloud = cloud->begin();
  Mat_<float>::const_iterator pDepth = input_depth.begin<float>();
  for (int j = 0; j < input_depth.rows; j++) {
    for (int i = 0; i < input_depth.cols; i++, pCloud++, pDepth++) {
      pCloud->z = *pDepth;
      pCloud->x = static_cast<float>(i - params[2]) *
                  *pDepth / params[0];
      pCloud->y = static_cast<float>(params[5] - j) *
                  *pDepth / params[4];
    }
  }
  cloud->sensor_origin_.setZero();
  cloud->sensor_orientation_.w() = 1.0;
  cloud->sensor_orientation_.x() = 0;
  cloud->sensor_orientation_.y() = 0;
  cloud->sensor_orientation_.z() = 0;
}

void EstimateNormals(const PointCloud<PointXYZ> &cloud,
                     PointCloud<PointNormal> *normals,
                     bool fill) {
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(30.0f);
	ne.setInputCloud(cloud.makeShared());
	ne.compute(*normals);
	if(fill) {
		PointCloud<PointNormal>::iterator p = normals->begin();
		while(p != normals->end()) {
			if(isnan(p->normal_x))
				p->normal_x = 0;
			if(isnan(p->normal_y))
				p->normal_y = 0;
			if(isnan(p->normal_z))
				p->normal_z = 0;
			++p;
		}
	}
}

bool OpenImage(const string &filename, Mat *output) {
  *output = imread(filename, IMREAD_ANYDEPTH);
  if(output->rows == 0) {
      std::cout << "Error, File: " << filename << " doesn't exist" << std::endl;
      return false;
  }
  return true;
}

bool CreateNormals(const vector<float> &params,
                   const vector<bool> &flat_labels,
                   const string &depth_file,
                   const string &labels_file,
                   const string &output_file) {
  PointCloud<PointXYZ> cloud;
  PointCloud<PointNormal> normals;
  // Open the depth file.
  Mat depth, labels;
  if(!OpenImage(depth_file, &depth))
    return false;
  // Open the labels file
  if(!OpenImage(labels_file, &labels))
    return false;  
  // Convert nyu depth to float
  depth.convertTo(depth, CV_32FC1);
  depth = depth / 1000.0f;

  // Inpaint the depth holes
  Mat mask, mask_8bit, filled_depth;
  threshold(depth, mask, 0, 1, THRESH_BINARY_INV);
  mask.convertTo(mask_8bit, CV_8U);
  inpaint(depth, mask_8bit, filled_depth, 5, INPAINT_NS);

  // Create cloud and estimate normals
  CreatePointCloud(filled_depth, params, &cloud);
  EstimateNormals(cloud, &normals, true);

  // Let's extract that data
  Mat normal_mat;
  GetMatFromCloud(normals, &normal_mat);
  imwrite(output_file, normal_mat);

  // Let's smooth out the flat surfaces
  ConnectedComponents(labels, flat_labels, true, &normal_mat);
  imwrite(labels_file + ".exr", normal_mat);
  return true;
}

int main (int argc, char** argv) {  
  vector<float> params;
  vector<bool> flat_labels;
  if (!ReadParameters(argv[1], &params, &flat_labels)) {
    cout << "Error reading parameters" << endl;
    return -1;
  }
  
  if (argc == 3) {
    std::ifstream file(argv[2]);
    std::string str; 
    while (std::getline(file, str)) {
      int pos = str.find_last_of(",");
      string remainder = str.substr(0, pos);
      string output_file = str.substr(pos+1);
      pos = remainder.find_last_of(",");
      string depth_file = remainder.substr(0, pos);
      string labels_file = remainder.substr(pos+1);
      if (!CreateNormals(params, flat_labels, depth_file, labels_file, output_file))
        return -1;
    }
  } else if (argc == 5) {
      if (!CreateNormals(params, flat_labels, string(argv[2]), string(argv[3]), string(argv[4])))
        return -1;
  } else {
    cout << "Improper usage. Must be CreateNormals params_file text_file" << endl;
    cout << "Or CreateNormals params_file depth_file labels_file output_file" << endl;
    return -1;
  }
  return 0;
}
