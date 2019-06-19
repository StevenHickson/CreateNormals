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
		while (b != elements[b].parent)
			b = elements[b].parent;
		elements[a].parent = b;
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