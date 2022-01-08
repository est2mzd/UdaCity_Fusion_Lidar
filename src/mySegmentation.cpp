/* \author Takuya Kobayashi */
#ifndef MYSEGMENTATION_H
#define MYSEGMENTATION_H
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
#include <chrono>
#include <unordered_set>
#include "myRansac3D.cpp"

//--------------------------------------------------------------------------------------//
// MySegmentPlane
//--------------------------------------------------------------------------------------//
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliers = myRansac3D(cloud, maxIterations, distanceThreshold); // 100, 0.2

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>()); // Plane
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>()); // Obstacles

    // Check All points.
    for(int index = 0; index < cloud->points.size(); index++)
    {
        std::cout << "myRansac3D, index = " << index << std::endl;
        typename pcl::PointCloud<PointT>::Ptr point = cloud->points[index]; // pick up each point

        if(inliers.count(index)) // check if this point is in inliers.
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Create function output
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

#endif