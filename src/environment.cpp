/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/ransac/ransacPlane.h"
#include "quiz/cluster/cluster.cpp"

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
/*
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    float xLimit = 25.0;
    float yLimit = 6.0;
    float zLimit = 4.0;

    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, .5 , Eigen::Vector4f (-xLimit, -yLimit, -zLimit, 1), Eigen::Vector4f ( xLimit, yLimit, zLimit, 1));
    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    auto obstacleCloud = segmentedCloud.first;
    auto planeCloud = segmentedCloud.second;

    auto obstacleClusters = pointProcessorI->Clustering(obstacleCloud, 1.0, 5, 200);

    renderPointCloud(viewer,filterCloud,"inputCloud");
    renderPointCloud(viewer,planeCloud,"planeCloud",Color(0,1,0));

    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};
    int ix = 0;

    for(auto cluster : obstacleClusters){
        //renderPointCloud(viewer,cluster,"obstacleCloud " + std::to_string(ix),colors[ix % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,ix);
        ix++;
    }

}
*/

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    float xLimit = 25.0;
    float yLimit = 6.0;
    float zLimit = 4.0;

    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, .5 , Eigen::Vector4f (-xLimit, -yLimit, -zLimit, 1), Eigen::Vector4f ( xLimit, yLimit, zLimit, 1));

    //const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = RansacPlane(filterCloud, 100, 0.2);

    auto obstacleCloud = segmentedCloud.second;
    auto planeCloud = segmentedCloud.first;

    //auto obstacleClusters = pointProcessorI->Clustering(obstacleCloud, 1.0, 5, 200);
    auto obstacleClusters = clusterPoints(*obstacleCloud, 1.0, 5.0);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = {};

    //renderPointCloud(viewer,obstacleCloud,"inputCloud", Color(1, 0, 0));
    renderPointCloud(viewer,planeCloud,"planeCloud",Color(0,1,0));

    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};
    int ix = 0;

    for(auto cluster : obstacleClusters){
        renderPointCloud(viewer,cluster,"obstacleCloud " + std::to_string(ix),colors[ix % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,ix);
        ix++;
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    auto* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    float xLimit = 25.0;
    float yLimit = 6.0;
    float zLimit = 4.0;


    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, .5 , Eigen::Vector4f (-xLimit, -yLimit, -zLimit, 1), Eigen::Vector4f ( xLimit, yLimit, zLimit, 1));
    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    auto obstacleCloud = segmentedCloud.first;
    auto planeCloud = segmentedCloud.second;

    auto obstacleClusters = pointProcessorI->Clustering(obstacleCloud, 1.0, 5, 200);

    renderPointCloud(viewer,filterCloud,"inputCloud");
    renderPointCloud(viewer,planeCloud,"planeCloud",Color(0,1,0));

    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};
    int ix = 0;

    for(auto &cluster : obstacleClusters){
        //renderPointCloud(viewer,cluster,"obstacleCloud " + std::to_string(ix),colors[ix % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,ix);
        ix++;
    }

    //renderPointCloud(viewer,inputCloud,"ogCloud");
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);

    auto scanRes = lidar->scan();

    //renderRays(viewer, lidar->position, scanRes);

    //renderPointCloud(viewer, scanRes, "Hi");
    // TODO:: Create point processor
    auto pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(scanRes, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    auto box = pointProcessor->BoundingBox(scanRes);
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
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}