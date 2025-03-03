/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // instantaniate a pointer to a lidar object. create lidar pointer object on the heap using new keyword
    // lidar object is going to hold pcd which could be very large.
    // hence instantaniating on heap, we have more memory to work with than the 2MB on the stack.
    // hence it takes longer time to look objects on the heap while on stack lookup is very fast.
    Lidar* lidar = new Lidar(cars, 0.0);
    // ptr type from pointcloud indicates that the object is a 32bit integer that contains the memory 
    // address of your point cloud object.
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();    
    // pcl::PointCloud<PointT>::ptr cloud = lidar->scan(); 
    // Vect3 origin(0.0, 0.0, 0.5);
    // renderRays(viewer, origin, cloud);
    // renderRays(viewer, lidar->position, cloud);
    Color red = Color(255,0,0);
    Color green = Color(0,255,0);
    Color blue = Color(0,0,255);

    // turn off coming new point clouds as it may overlap
    // renderPointCloud(viewer, inputCloud, "LidarPCD", green);
    // road points = green, other obstacles = red



    // TODO:: Create point processor
    // stack intitialization
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    // heap initialization
    // ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();


    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.4);
    // renderPointCloud(viewer, segmentCloud.first, "Obstacles", red);
    // renderPointCloud(viewer, segmentCloud.second, "Road", green);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    bool render_box = true, render_clusters = true;
    std::vector<Color> colors = {Color(1,0,1), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters){
        if(render_clusters){
            std::cout<<"cluster size ";
            pointProcessor.numPoints(cluster);
            // std::cout<<""<<cluster->size();
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        }
        if(render_box)
        {
            // Box box = pointProcessor.BoundingBox(cluster);
            // renderBox(viewer, box, clusterId);

            // bounding box with smaller size
            BoxQ boxq = pointProcessor.BoundingBoxQ(cluster);
            renderBox(viewer, boxq, clusterId);
        }
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spin();
    } 
}