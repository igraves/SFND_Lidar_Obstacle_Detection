//
// Created by Ian Graves on 3/17/20.
//

#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// std::pair<std::unordered_set<int>,std::unordered_set<int>> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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

        for(int point_index = 0; point_index < cloud->points.size(); point_index++){
            auto point = &cloud->points[point_index];
            float ptx = point->x;
            float pty = point->y;
            float ptz = point->z;

            float dist = fabs(((polyA * ptx) + (polyB * pty) + (polyC * ptz) + polyD))/abSqrt;
            if(dist < distanceTol){
                //std::cout << dist << std::endl;
                iter_inliers.insert(point_index);
            }
        }

        if(iter_inliers.size() > inliersResult.size()){
            inliersResult = iter_inliers;
        }
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        }

    // Return indicies of inliers from fitted line with most inliers
    pcl::PointCloud<pcl::PointXYZI>::Ptr inliers (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZI>());

    for(int pi = 0; pi < cloud->points.size(); pi++){
        if(inliersResult.count(pi) > 0){
            inliers->push_back(cloud->points[pi]);
        } else {
            outliers->push_back(cloud->points[pi]);
        }
    }
    return std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>(inliers, outliers);

}

