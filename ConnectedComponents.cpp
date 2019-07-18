#include "ConnectedComponents.h"
#include <math.h>

using namespace std;
using namespace cv;

bool isnan(const Vec3f& vec) {
  return (isnan(vec[0]) || isnan(vec[1]) || isnan(vec[2]));
}

float MeasureAngle(const vector<bool> &flat_labels, const Vec3f &a, const Vec3f &b, int a_label, int b_label) {
  if(a_label != b_label || isnan(a) || isnan(b) || !flat_labels[a_label])
    return 100.0;
  float angle = acos(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
  return angle;
}

int BuildGraph(const Mat &labels, const Mat &normals, const vector<bool> &flat_labels, float planar_thresh, bool reduce_edges, vector<Edge> *edges) {
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
          edge.weight = MeasureAngle(flat_labels, *pNormals, *(pNormals + 1), *pLabels, *(pLabels + 1));
          if (!reduce_edges || (edge.weight < planar_thresh && *pLabels == *(pLabels + 1))) {
            edges->push_back(edge);
            num++;
          }
        }
        if (y < safeHeight) {
          Edge edge;
          edge.a = y * width + x;
          edge.b = yp * width + x;
          edge.weight = MeasureAngle(flat_labels, *pNormals, *(pNormals + width), *pLabels, *(pLabels + width));
          if (!reduce_edges || (edge.weight < planar_thresh && *pLabels == *(pLabels + 1))) {
            edges->push_back(edge);
            num++;
          }
        }
      }
      pLabels++;
      pNormals++;
		}
	}
  return num;
}

void SegmentGraph(const vector<Edge> &edges, float planar_thresh, Universe *universe) {
  vector<Edge>::const_iterator pEdge = edges.begin();
  while(pEdge != edges.end()) {
	  int a = universe->find(pEdge->a);
		int b = universe->find(pEdge->b);
		if (a != b && pEdge->weight < planar_thresh)
		  universe->join(a, b);
    pEdge++;
  }
}

void ComputeAndSetNormalAverages(Universe &uni, const vector<bool> &flat_labels, const Mat &labels, Mat *normals) {
  // Set up vector with normal values
  map<int, ComponentInfo> info;

  // Let's grab the normals for each segment
  int i = 0;
  Mat segments = Mat::zeros(labels.rows, labels.cols, CV_32S) - 1;
  Mat_<Vec3f>::iterator pNormals = normals->begin<Vec3f>();
  Mat_<int>::iterator pSegments = segments.begin<int>();
  Mat_<uint16_t>::const_iterator pLabels = labels.begin<uint16_t>();
  while(pNormals != normals->end<Vec3f>()) {
    if (flat_labels[*pLabels]) {
      int segment = uni.find(i);
      if(uni.size(segment) > 1) { 
        info[segment].AddValue(*pNormals);
        *pSegments = segment;
      }
    }
    i++; pNormals++; pSegments++; pLabels++;
  }
  // Let's calculate the averages of the info vector into a new vector
  for(map<int, ComponentInfo>::iterator p = info.begin(); p != info.end(); p++) {
    p->second.ComputeAverage();
  }

  // Now lets reset the normals to be the average across its segments
  pNormals = normals->begin<Vec3f>();
  pSegments = segments.begin<int>();
  pLabels = labels.begin<uint16_t>();
  while(pNormals != normals->end<Vec3f>()) {
    if(*pSegments >= 0 && flat_labels[*pLabels]) {
      *pNormals = info[*pSegments].GetAverage();
    }
    pSegments++; pNormals++; pLabels++;
  }
}

void ConnectedComponents(const Mat &labels, const vector<bool> &flat_labels, float planar_thresh, bool fast_method, Mat *normals) {
  vector<Edge> edges;
  
  // Segment the normals based on labels
  BuildGraph(labels, *normals, flat_labels, planar_thresh, fast_method, &edges);
  Universe uni(labels.rows * labels.cols * 2);
  SegmentGraph(edges, planar_thresh, &uni);

  // Compute and fix normal averages
  ComputeAndSetNormalAverages(uni, flat_labels, labels, normals);
}

