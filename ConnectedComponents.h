#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class ComponentInfo {
    double sum_x, sum_y, sum_z;
    int count;
public:
    ComponentInfo() : sum_x(0), sum_y(0), sum_z(0), count(0) {}
    void AddValue(cv::Vec3f val) {
        sum_x += val[0];
        sum_y += val[1];
        sum_z += val[2];
        count++;
    }
    cv::Vec3f GetAverage() {
        return cv::Vec3f(sum_x / double(count), sum_y / double(count), sum_z / double(count));
    }
};

class Edge {
public:
    float weight;
    int a, b;
    Edge() : weight(0), a(0), b(0) {};
};

class UniverseMember {
public:
    int rank;
    int size;
    int parent;
};

class Universe {
    std::vector<UniverseMember> elements;
	int num;
public:
    Universe() : num(0) { }
    Universe(int num_elements)
	{
		num = num_elements;
		elements.resize(num);
		std::vector<UniverseMember>::iterator p = elements.begin();
		int i = 0;
		while(p != elements.end()) {
			p->rank = 0;
			p->size = 1;
			p->parent = i;
			p++;
			i++;
		}
	}
	~Universe(){};
	int find(int a)
	{
		int b = a;
    std::vector<int> update_parents_list;
		while (b != elements[b].parent) {
      update_parents_list.push_back(b);
			b = elements[b].parent;
    }
    // Path compression
    std::vector<int>::iterator p = update_parents_list.begin();
    while (p != update_parents_list.end()) {
      elements[*p].parent = b;
      p++;
    }
		return b;
	};  
	void join(int a, int b)
	{
		if (elements[a].rank > elements[b].rank)
		{
			elements[b].parent = a;
			elements[a].size += elements[b].size;
		} 
		else
		{
			elements[a].parent = b;
			elements[b].size += elements[a].size;
			if (elements[a].rank == elements[b].rank)
				elements[b].rank++;
		}
		num--;
	}
	void release() {
		elements.clear();
	}
	int size(int a) const { return elements[a].size; }
	int num_sets() const { return num; }
};

void ConnectedComponents(const cv::Mat &parents, cv::Mat *normals);
void ConnectedComponents2(const cv::Mat &parents, cv::Mat *normals);
