#include "CreateNormals.h"

using namespace std;

int main (int argc, char** argv) {  
  vector<float> camera_params, normal_params;
  vector<bool> flat_labels;
  if (!ReadParameters(argv[1], &camera_params, &normal_params, &flat_labels)) {
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
      if (!CreateNormals(camera_params, normal_params, flat_labels, depth_file, labels_file, output_file))
        return -1;
    }
  } else if (argc == 5) {
      if (!CreateNormals(camera_params, normal_params, flat_labels, string(argv[2]), string(argv[3]), string(argv[4])))
        return -1;
  } else {
    cout << "Improper usage. Must be CreateNormalsMain params_file text_file" << endl;
    cout << "Or CreateNormalsMain params_file depth_file labels_file output_file" << endl;
    return -1;
  }
  return 0;
}
