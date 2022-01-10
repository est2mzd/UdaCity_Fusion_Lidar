/* \author Takuya Kobayashi */
#ifndef MYCLUSTER_H
#define MYCLUSTER_H
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include "myKdtree.h"

//-----------------------------------------------------------------------------------//
// clusterHelper
//-----------------------------------------------------------------------------------//
template<typename PointT>
void clusterHelper(int id, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    // process the point of "input id"
    processed[id] = true;
    cluster.push_back(id);

    // Find the nearest_point_ids point list of points[id] --> nearest_point_ids
    std::vector<float> point = {cloud->points[id].x, cloud->points[id].y, cloud->points[id].z};
    std::vector<int> nearest_point_ids = tree->search(point, distanceTol);

    // process the points in "ids of nearest_point_ids"
    for(int i : nearest_point_ids)
    {
        if(!processed[i])
        {
            clusterHelper<PointT>(i, cloud, cluster, processed, tree, distanceTol);
        }
    }
}

//-----------------------------------------------------------------------------------//
// myEuclideanCluster
//-----------------------------------------------------------------------------------//
template<typename PointT>
std::vector<std::vector<int>> myEuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance)
{
	//std::vector<std::vector<int>> clusters;
    std::vector<std::vector<int>> clusters;
 
	// init processed with "false". The size of Array is points.size()
    const int pointNum = cloud->points.size();
	std::vector<bool> processed(cloud->points.size(), false);

	int i=0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

        std::vector<int> cluster;
		clusterHelper<PointT>(i, cloud, cluster, processed, tree, clusterTolerance);
		clusters.push_back(cluster);
		i++;
	}

	return clusters;
}


//-----------------------------------------------------------------------------------//
// myClustering
//-----------------------------------------------------------------------------------//
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // KdTree
    KdTree* tree = new KdTree;

    for (int i=0; i < cloud->points.size(); i++){
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i);
    }

    // Euclidean Clustering
    std::vector<std::vector<int>> multiClusterIds = myEuclideanCluster<PointT>(cloud, tree, clusterTolerance);

    for(std::vector<int> singleClusterIds : multiClusterIds)
    {
        if((singleClusterIds.size() < minSize) || (singleClusterIds.size() > maxSize)){
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index: singleClusterIds){
            cloudCluster->points.push_back(cloud->points[index]);
        }
        clusters.push_back(cloudCluster);
    }
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "myClustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

#endif