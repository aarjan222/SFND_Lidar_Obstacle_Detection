/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	// return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
	// return pointProcessor.loadPcd("../../../sensors/data/pcd/kitti_data.pcd");
	return pointProcessor.loadPcd("../../../sensors/data/pcd/logictronix_env.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();

	// For max iterations
	while (maxIterations--)
	{
		// Randomly pick 2 points
		std::unordered_set<int> inliers;
		while (inliers.size() < 2)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		float x1, y1, x2, y2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1 * y2 - x2 * y1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			// if that point is already inside inliers, dont include twice.
			if (inliers.count(index) > 0)
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			// Measure distance between every point and fitted line
			float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		// record the line with the highest inliers
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();

	// For max iterations
	while (maxIterations--)
	{
		// Randomly pick 3 points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		auto itr = inliers.begin();
		pcl::PointXYZ p1 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ p2 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ p3 = cloud->points[*itr];

		// Define two vectors in the plane
		Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
		Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

		// Compute the normal using cross product
		Eigen::Vector3f normal = v1.cross(v2);
		float A = normal[0];
		float B = normal[1];
		float C = normal[2];
		float D = -(A * p1.x + B * p1.y + C * p1.z);

		// Ensure the normal is not zero
		if (A == 0 && B == 0 && C == 0)
		{
			continue;
		}

		for (int index = 0; index < cloud->points.size(); index++)
		{
			// if that point is already inside inliers, dont include twice.
			if (inliers.count(index) > 0)
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[index];
			float x = point.x, y = point.y, z = point.z;

			// Measure distance between every point and fitted line
            float d = fabs(A * x + B * y + C * z + D) / sqrt(A * A + B * B + C * C);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		// record the line with the highest inliers
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RansacPlane Segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));	 // road is green
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0)); // obstacles are red
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spin();
	}
}
