//
// Created by Ian Graves on 3/17/20.
//

#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// std::pair<std::unordered_set<int>,std::unordered_set<int>>
const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

