#include "CreateNormals.h"

using namespace std;
using namespace pcl;
using namespace cv;

#define KINECT_CX_D 3.2330780975300314e+02
#define KINECT_CY_D 2.4273913761751615e+02
#define KINECT_FX_D 1.6828944189289601e-03
#define KINECT_FY_D 1.6919313269589566e-03


void MakeCloudDense(PointCloud<PointXYZRGBA> &cloud) {
	PointCloud<PointXYZRGBA>::iterator p = cloud.begin();
	cloud.is_dense = true;
	for(int j = 0; j < cloud.height; j++) {
		for(int i = 0; i < cloud.width; i++) {
			if(isnan(p->z)) {
				p->x = float(((float)i - KINECT_CX_D) * KINECT_FX_D);
				p->y = float(((float)j - KINECT_CY_D) * KINECT_FY_D);
				p->z = 0;
			}
			//p->a = 255;
			++p;
		}
	}
}

void CreatePointCloud(const Mat& input_image, const Mat& input_depth,
                             PointCloud<PointXYZRGBA>* cloud, float focal) {
  if (cloud == NULL) {
    std::cout << "cloud cannot be NULL in CreatePointCloud" << std::endl;
    return;
  }
  if (input_image.cols != input_depth.cols || input_image.rows != input_depth.rows) {
    std::cout << "Image and depth must have equal dimensons" << std::endl;
    return;
  }
  cloud->header.frame_id = "/cityscapes_frame";
  cloud->height = input_image.rows;
  cloud->width = input_image.cols;
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);

  PointCloud<PointXYZRGBA>::iterator pCloud = cloud->begin();
  Mat_<Vec3b>::const_iterator pImg = input_image.begin<Vec3b>();
  Mat_<float>::const_iterator pDepth = input_depth.begin<float>();
  for (int j = 0; j < input_image.rows; j++) {
    for (int i = 0; i < input_image.cols; i++, pCloud++, pImg++, pDepth++) {
      pCloud->z = *pDepth;
      pCloud->x = static_cast<float>(i - 320) *
                  *pDepth / focal;
      pCloud->y = static_cast<float>(j - 240) *
                  *pDepth / focal;
      pCloud->r = (*pImg)[2];
      pCloud->g = (*pImg)[1];
      pCloud->b = (*pImg)[0];
    }
  }
  cloud->sensor_origin_.setZero();
  cloud->sensor_orientation_.w() = 1.0;
  cloud->sensor_orientation_.x() = 0;
  cloud->sensor_orientation_.y() = 0;
  cloud->sensor_orientation_.z() = 0;
}

void EstimateNormals(const PointCloud<PointXYZRGBA>::ConstPtr &cloud, PointCloud<PointNormal>::Ptr &normals, bool fill) {
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
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

int main (int argc, char** argv) {
    return 0;
}