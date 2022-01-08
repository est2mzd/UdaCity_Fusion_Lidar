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
void clusterHelper(int id, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& clusters, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    // process the point of "input id"
    processed[id] = true;
    clusters.push_back(id);

    // Find the nearest point list of points[id] --> nearest
    std::vector<float> point = {cloud->points[id].x, cloud->points[id].y, cloud->points[id].z};
    std::vector<int> nearest = tree->search(point, distanceTol);

    // process the points in "ids of nearest"
    for(int i : nearest)
    {
        if(!processed[i])
        {
            clusterHelper(i, cloud, clusters, processed, tree, distanceTol);
        }
    }
}

//-----------------------------------------------------------------------------------//
// myEuclideanCluster
//-----------------------------------------------------------------------------------//
template<typename PointT>
std::vector<std::vector<int>> myEuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize)
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
		clusterHelper(i, cloud->points, cluster, processed, tree, clusterTolerance);
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
    std::vector<std::vector<int>> clusterIndices = myEuclideanCluster(cloud, tree, 3.0, minSize, maxSize);

    for(std::vector<int> getIndices : clusterIndices)
    {
        //pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index: getIndices){
            //clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
            cloudCluster->points.push_back(cloud->points[index]);
        }
        clusters.push_back(cloudCluster);
    }
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

#endif