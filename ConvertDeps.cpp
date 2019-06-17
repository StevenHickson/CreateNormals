#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

string PathFindExtension(string fname) {
    int pos = fname.find_last_of(".");
    return fname.substr(pos);
}

Mat imread_depth(const char* fname, bool binary) {
	string ext = PathFindExtension(fname);
	const char char_dep[] = ".dep";
	const char char_png[] = ".png";
	Mat out;
    if(strncasecmp(ext.c_str(),char_dep,strlen(char_dep))==0) {
		FILE *fp;
		if(binary)
			fp = fopen(fname,"rb");
		else
			fp = fopen(fname,"r");
		int width = 640, height = 480; //If messed up, just assume
		if(binary) {
			fread(&width,sizeof(int),1,fp);
			fread(&height,sizeof(int),1,fp);
			out = Mat(height,width,CV_32S);
			int *p = (int*)out.data;
			fread(p,sizeof(int),width*height,fp);
		} else {
			//fscanf(fp,"%i,%i,",&width,&height);
			out = Mat(height,width,CV_32S);
			int *p = (int*)out.data, *end = ((int*)out.data) + out.rows*out.cols;
			while(p != end) {
				fscanf(fp,"%i",p);
				p++;
			}
		}
		fclose(fp);
	} else if(strncasecmp(ext.c_str(),char_png,strlen(char_png))==0) {
		out = imread(fname, IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
		out.convertTo(out, CV_32S);
		int* pi = (int*)out.data;
		for (int y=0; y < out.rows; y++) {
			for (int x=0; x < out.cols; x++) {
				*pi = round(*pi * 0.2f);
				pi++;
			}
		}
	} else {
		throw exception();
	}
	return out;
}

int main (int argc, char** argv) {  
  Mat depth = imread_depth(argv[1], true);
  string file_string = String(argv[1]);
  int pos = file_string.find_last_of(".");
  string file_root = file_string.substr(0, pos);
  depth.convertTo(depth, CV_16UC1);
  imwrite(file_root + ".png", depth);
  return 0;
}
