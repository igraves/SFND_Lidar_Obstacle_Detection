/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
// #include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
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
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for(int i = 0; i < maxIterations; i++){
        // Randomly sample subset and fit line
        int points = cloud->points.size();
        auto pt1 = &cloud->points[rand() % points];
        auto pt2 = &cloud->points[rand() % points];

        while(pt1 == pt2){
            pt2 = &cloud->points[rand() % points];
        }

        float polyA = pt1->y - pt2->y;
        float polyB = pt2->x - pt1->x;
        float polyC = (pt1->x * pt2->y - pt2->x * pt1->y);

        float abSqrt = sqrt(pow(polyA,2) + pow(polyB,2));

        std::unordered_set<int> iter_inliers;

        for(int point_index = 0; i < cloud->points.size(); i++){
            auto point = &cloud->points[i];
            float ptx = point->x;
            float pty = point->y;

            float dist = fabs(((polyA * ptx) + (polyB * pty) + polyC))/abSqrt;
            if(dist < distanceTol){
                std::cout << dist << std::endl;
                iter_inliers.insert(i);
            }
        }

        if(iter_inliers.size() > inliersResult.size()){
            inliersResult = iter_inliers;
        }
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        // Return indicies of inliers from fitted line with most inliers

	}

	return inliersResult;

}

std::pair<std::unordered_set<int>,std::unordered_set<int>> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> outliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations
    for(int i = 0; i < maxIterations; i++){
        // Randomly sample subset and fit line
        int points = cloud->points.size();
        auto pt1 = &cloud->points[rand() % points];
        auto pt2 = &cloud->points[rand() % points];
        auto pt3 = &cloud->points[rand() % points];

        while(pt1 == pt2){
            pt2 = &cloud->points[rand() % points];
        }

        while(pt2 == pt3){
            pt3 = &cloud->points[rand() % points];
        }

        float x1 = pt1->x;
        float y1 = pt1->y;
        float z1 = pt1->z;
        float x2 = pt2->x;
        float y2 = pt2->y;
        float z2 = pt2->z;
        float x3 = pt3->x;
        float y3 = pt3->y;
        float z3 = pt3->z;

        //(y2−y1)(z3−z1)−(z2−z1)(y3−y1)
        float iv = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
        //(z2−z1)(x3−x1)−(x2−x1)(z3−z1)
        float jv = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
        //(x2−x1)(y3−y1)−(y2−y1)(x3−x1)
        float kv = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);


        //OG
        float polyA = iv;
        float polyB = jv;
        float polyC = kv;
        float polyD = -1 * (iv * x1 + jv * y1 + kv * z1);

        float abSqrt = sqrtf(powf(polyA,2) + powf(polyB,2) + powf(polyC, 2));

        std::unordered_set<int> iter_inliers;

        for(int point_index = 0; i < cloud->points.size(); i++){
            auto point = &cloud->points[i];
            float ptx = point->x;
            float pty = point->y;
            float ptz = point->z;

            float dist = fabs(((polyA * ptx) + (polyB * pty) + (polyC * ptz) + polyD))/abSqrt;
            if(dist < distanceTol){
                std::cout << dist << std::endl;
                iter_inliers.insert(i);
            }
        }

        if(iter_inliers.size() > inliersResult.size()){
            inliersResult = iter_inliers;
        }
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        // Return indicies of inliers from fitted line with most inliers

    }
    return std::pair<std::unordered_set<int>,std::unordered_set<int>>(inliersResult, outliersResult);

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 1000, 1.0);
    std::pair<std::unordered_set<int>,std::unordered_set<int>> res = RansacPlane(cloud, 1000, 0.45);
    auto inliers = res.first;
    //auto outliers = res.second;

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(!inliers.empty())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
    else
  	{
        renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
