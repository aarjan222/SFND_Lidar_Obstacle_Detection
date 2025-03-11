#include <chrono>
#include <string>
#include "kdtree_3d.h"

int main()
{
	// Create data
	std::vector<std::vector<float>> points = {{3.0, 1.0, 4.0},	// 0 d = sqrt(9)
											  {2.0, 3.0, 7.0},	// 1
											  {4.0, 3.0, 4.0},	// 2 d = sqrt(3)
											  {2.0, 1.0, 3.0},	// 3
											  {2.0, 2.0, 3.0},	// 4
											  {2.0, 2.0, 4.0},	// 5
											  {2.0, 4.0, 5.0},	// 6
											  {6.0, 1.0, 4.0},	// 7 d = sqrt(3)
											  {1.0, 4.0, 4.0},	// 8
											  {0.0, 5.0, 7.0},	// 9
											  {5.0, 2.0, 5.0},	// 10 d = 0
											  {4.0, 0.0, 6.0},	// 11 d = sqrt(6)
											  {7.0, 1.0, 6.0}}; // 12 d = sqrt(6)

	KdTree *tree = new KdTree;

	for (int i = 0; i < points.size(); i++)
		tree->insert(points[i], i);

	int it = 0;

	std::cout << "Test Search" << std::endl;
	std::vector<int> nearby = tree->search({5.0, 2.0, 5.0}, 3.0);

	for (int index : nearby)
		std::cout << index << ",";
	std::cout << std::endl;
}
