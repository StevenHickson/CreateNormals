#include "ConnectedComponents.h"
#include <math.h>

using namespace std;
using namespace cv;

//#define ANGLE_MAX 0.523599
#define ANGLE_MAX 0.0472665

// 4, 11, 21 all flat surfaces.
vector<bool> flat_labels = {0,0,0,0,1,
                            0,0,0,0,0,
                            0,1,0,0,0,
                            1,0,0,0,1,
                            0,1,0,0,0,
                            0,0,0,1,0,
                            0,0,0,0,1,
                            0,1,1,0,0,
                            0,0,0,0,0,
                            1,0,0,0,0};


bool isnan(const Vec3f& vec) {
  return (isnan(vec[0]) || isnan(vec[1]) || isnan(vec[2]));
}

float MeasureAngle(const Vec3f &a, const Vec3f &b, int a_label, int b_label) {
  if(a_label != b_label || isnan(a) || isnan(b) || !flat_labels[a_label])
    return 100.0;
  float angle = acos(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
  //if(angle < 0 || angle > 5.75959)
  //  cout << "Angle is: " << angle << endl;
  return angle;
}

int BuildGraph(const Mat &labels, const Mat &normals, bool reduce_edges, vector<Edge> *edges) {
	int width = labels.cols;
	int height = labels.rows;
	int num = 0;
	int x, y, xp, ym, yp;
	int safeWidth = width - 1, safeHeight = height - 1;
  Mat_<uint16_t>::const_iterator pLabels = labels.begin<uint16_t>();
  Mat_<Vec3f>::const_iterator pNormals = normals.begin<Vec3f>();
  edges->reserve(width * height * 2);
  for ( y = 0, ym = -1, yp = 1; y < height; y++, ym++, yp++) {
    for ( x = 0, xp = 1; x < width; x++, xp++) {
      if (!reduce_edges || flat_labels[*pLabels]) {
        if (x < safeWidth) {
          Edge edge;
          edge.a = y * width + x;
          edge.b = y * width + xp;
          edge.weight = MeasureAngle(*pNormals, *(pNormals + 1), *pLabels, *(pLabels + 1));
          if (!reduce_edges || (edge.weight < ANGLE_MAX && *pLabels == *(pLabels + 1))) {
            edges->push_back(edge);
            num++;
          }
        }
        if (y < safeHeight) {
          Edge edge;
          edge.a = y * width + x;
          edge.b = yp * width + x;
          edge.weight = MeasureAngle(*pNormals, *(pNormals + width), *pLabels, *(pLabels + width));
          if (!reduce_edges || (edge.weight < ANGLE_MAX && *pLabels == *(pLabels + 1))) {
            edges->push_back(edge);
            num++;
          }
        }
        /*if (x < safeWidth && y < safeHeight) {
          Edge edge;
          edge.a = y * width + x;
          edge.b = yp * width + xp;
          edge.weight = MeasureAngle(*pNormals, *(pNormals + width + 1), *pLabels, *(pLabels + width + 1));
          edges->push_back(edge);
          num++;
        }
        if (x < safeWidth && y > 0) {
          Edge edge;
          edge.a  = y * width + x;
          edge.b  = ym * width + xp;
          edge.weight = MeasureAngle(*pNormals, *(pNormals - width + 1), *pLabels, *(pLabels - width + 1));
          edges->push_back(edge);
          num++;
        }*/
      }
      pLabels++;
      pNormals++;
		}
	}
  return num;
}

void SegmentGraph(const vector<Edge> &edges, Universe *universe) {
  vector<Edge>::const_iterator pEdge = edges.begin();
  while(pEdge != edges.end()) {
	  int a = universe->find(pEdge->a);
		int b = universe->find(pEdge->b);
		if (a != b && pEdge->weight < ANGLE_MAX)
		  universe->join(a, b);
    pEdge++;
  }
}

void ConnectedComponents(const Mat &labels, Mat *normals) {
  vector<Edge> edges;
  
  // Segment the normals based on labels
  int num_edges = BuildGraph(labels, *normals, false, &edges);
  Universe uni(num_edges);
  SegmentGraph(edges, &uni);

  // Set up vector with normal values
  vector<ComponentInfo> info;
  info.resize(uni.num_sets());
  //cout << "Number of sets: " << uni.num_sets() << endl;
  map<int, int> mapping;

  // Let's grab the normals for each segment
  int i = 0;
  int segment_count = 0;
  Mat segments = Mat::zeros(labels.rows, labels.cols, CV_32S) - 1;
  Mat_<Vec3f>::iterator pNormals = normals->begin<Vec3f>();
  Mat_<int>::iterator pSegments = segments.begin<int>();
  Mat_<uint16_t>::const_iterator pLabels = labels.begin<uint16_t>();
  while(pNormals != normals->end<Vec3f>()) {
    //if(find(flat_labels.begin(), flat_labels.end(), *pLabels) != flat_labels.end()) {
      int segment = uni.find(i);
      //cout << segment <<  ", " << uni.size(segment) << endl;
      //if(uni.size(segment) > 1) { 
        if(mapping.find(segment) == mapping.end()) {
          mapping[segment] = segment_count;
          segment_count++;
        }
        int s = mapping[segment];
        info[s].AddValue(*pNormals);
        *pSegments = s;
      //}
    //}
    i++; pNormals++; pSegments++; pLabels++;
  }
  //Mat segments_write;
  //segments.convertTo(segments_write, CV_16UC1);
  //imwrite("/home/steve/datasets/nyu_dataset/nyu_dataset/depth/1_segments.png", segments_write);
  //cout << "Number of segments: " << segment_count << endl;
  // Let's calculate the averages of the info vector into a new vector
  vector<Vec3f> info_averages;
  info_averages.resize(info.size());
  for(unsigned int j = 0; j < info.size(); j++) {
    info_averages[j] = info[j].GetAverage();
  }

  // Now lets reset the normals to be the average across its segments
  pNormals = normals->begin<Vec3f>();
  pSegments = segments.begin<int>();
  pLabels = labels.begin<uint16_t>();
  while(pNormals != normals->end<Vec3f>()) {
    if(*pSegments >= 0 && flat_labels[*pLabels]) {
      //cout << *pLabels << endl;
      *pNormals = info_averages[*pSegments];
    }
    pSegments++; pNormals++; pLabels++;
  }
}

void ConnectedComponents2(const Mat &labels, Mat *normals) {
  vector<Edge> edges;
  
  // Segment the normals based on labels
  int num_edges = BuildGraph(labels, *normals, true, &edges);
  Universe uni(labels.rows * labels.cols);
  SegmentGraph(edges, &uni);

  // Set up vector with normal values
  vector<ComponentInfo> info;
  info.resize(uni.num_sets());
  //cout << "Number of sets: " << uni.num_sets() << endl;
  map<int, int> mapping;

  // Let's grab the normals for each segment
  int i = 0;
  int segment_count = 0;
  Mat segments = Mat::zeros(labels.rows, labels.cols, CV_32S) - 1;
  Mat_<Vec3f>::iterator pNormals = normals->begin<Vec3f>();
  Mat_<int>::iterator pSegments = segments.begin<int>();
  Mat_<uint16_t>::const_iterator pLabels = labels.begin<uint16_t>();
  while(pNormals != normals->end<Vec3f>()) {
    if (flat_labels[*pLabels]) {
      int segment = uni.find(i);
      if(uni.size(segment) > 1) { 
        if(mapping.find(segment) == mapping.end()) {
          mapping[segment] = segment_count;
          segment_count++;
        }
        int s = mapping[segment];
        info[s].AddValue(*pNormals);
        *pSegments = s;
      }
    }
    i++; pNormals++; pSegments++; pLabels++;
  }
  //Mat segments_write;
  //segments.convertTo(segments_write, CV_16UC1);
  //imwrite("/home/steve/datasets/nyu_dataset/nyu_dataset/depth/1_segments.png", segments_write);
  //cout << "Number of segments: " << segment_count << endl;
  // Let's calculate the averages of the info vector into a new vector
  vector<Vec3f> info_averages;
  info_averages.resize(info.size());
  for(unsigned int j = 0; j < info.size(); j++) {
    info_averages[j] = info[j].GetAverage();
  }

  // Now lets reset the normals to be the average across its segments
  pNormals = normals->begin<Vec3f>();
  pSegments = segments.begin<int>();
  pLabels = labels.begin<uint16_t>();
  while(pNormals != normals->end<Vec3f>()) {
    if(*pSegments >= 0 && flat_labels[*pLabels]) {
      //cout << *pLabels << endl;
      *pNormals = info_averages[*pSegments];
    }
    pSegments++; pNormals++; pLabels++;
  }
}


