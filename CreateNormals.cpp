#include "CreateNormals.h"
#include "ConnectedComponents.h"

using namespace std;
using namespace pcl;
using namespace cv;

#define KINECT_CX_D 3.1304475870804731e+02
#define KINECT_CY_D 2.4273913761751615e+02
#define KINECT_FX_D 2.3844389626620386e+02
#define KINECT_FY_D 5.8269103270988637e+02

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
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

void MakeCloudDense(PointCloud<PointXYZ> &cloud) {
	PointCloud<PointXYZ>::iterator p = cloud.begin();
	cloud.is_dense = true;
	for(int j = 0; j < cloud.height; j++) {
		for(int i = 0; i < cloud.width; i++) {
			if(isnan(p->z)) {
				p->x = float(((float)i - KINECT_CX_D) / KINECT_FX_D);
				p->y = float(((float)j - KINECT_CY_D) / KINECT_FY_D);
				p->z = 0;
			}
			++p;
		}
	}
}

void CreatePointCloud(const Mat& input_depth,
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
      pCloud->x = static_cast<float>(i - KINECT_CX_D) *
                  *pDepth / KINECT_FX_D;
      pCloud->y = static_cast<float>(KINECT_CY_D - j) *
                  *pDepth / KINECT_FY_D;
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
	ne.setMaxDepthChangeFactor(0.04f);
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

bool OpenImage(string &filename, Mat *output) {
  *output = imread(filename, IMREAD_ANYDEPTH);
  if(output->rows == 0) {
      std::cout << "Error, File: " << filename << " doesn't exist" << std::endl;
      return false;
  }
  return true;
}

int main (int argc, char** argv) {  
  std::ifstream file(argv[1]);
  std::string str; 
  while (std::getline(file, str)) {
    int pos = str.find_last_of(",");
    string depth_file = str.substr(0, pos);
    string labels_file = str.substr(pos+1);
    PointCloud<PointXYZ> cloud;
    PointCloud<PointNormal> normals;

    // Open the depth file.
    Mat depth, labels;
    if(!OpenImage(depth_file, &depth))
      return -1;
    // Open the labels file
    if(!OpenImage(labels_file, &labels))
      return -1;  
    // Convert nyu depth to float
    depth.convertTo(depth, CV_32FC1);
    depth = depth / 1000.0f;

    // Create cloud and estimate normals
    CreatePointCloud(depth, &cloud);
    EstimateNormals(cloud, &normals, true);

    // Let's extract that data
    Mat normal_mat;
    GetMatFromCloud(normals, &normal_mat);
    imwrite(depth_file + ".exr", normal_mat);

    cout << "Labels type: " << type2str(labels.type()) << endl;
    // Let's smooth out the flat surfaces
    ConnectedComponents(labels, &normal_mat);
    imwrite(labels_file + ".exr", normal_mat);
  }
  return 0;
}
