/* \author Takuya Kobayashi */
#ifndef MYRANSAC3D_H
#define MYRANSAC3D_H
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
#include <chrono>
#include <unordered_set>

//--------------------------------------------------------------------------------------//
// Ransac3D
//--------------------------------------------------------------------------------------//
template<typename PointT>
std::unordered_set<int> myRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    for(int i=0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        while(inliers.size() < 3)
        {
            // If inliers has the same value, below will fail.
            // so, inliers.sise is not incremented.
            inliers.insert(rand()%(cloud->points.size()));
        }

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        Eigen::Vector3f P1, P2, P3, Q, Vec1, Vec2, Vec3,VecN;
        auto itr = inliers.begin();
        P1.x() = cloud->points[*itr].x;
        P1.y() = cloud->points[*itr].y;
        P1.z() = cloud->points[*itr].z;
        itr++;
        P2.x() = cloud->points[*itr].x;
        P2.y() = cloud->points[*itr].y;
        P2.z() = cloud->points[*itr].z;
        itr++;
        P3.x() = cloud->points[*itr].x;
        P3.y() = cloud->points[*itr].y;
        P3.z() = cloud->points[*itr].z;
        //
        Vec1 = P2 - P1;
        Vec2 = P3 - P1;
        Vec3 = Vec1.cross(Vec2);
        VecN = Vec3.normalized();

        for(int index=0; index<cloud->points.size(); index++)
        {
            // inliers has already 3 points.
            // To avoid to set the same value, I check it below.
            if(inliers.count(index) > 0)
            {
                continue;
            }

            pcl::PointXYZ point = cloud->points[index];
            Q.x() = point.x;
            Q.y() = point.y;
            Q.z() = point.z;
            //
            Eigen::Vector3f VecPQ = (Q-P1);
            float d = fabs(VecPQ.dot(VecN));

            if(d <= distanceTol)
            {
                inliers.insert(index);
                //std::cout << "d = " << d << std::endl;
            }
        }

        // Update inliersResult
        if(inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
    std::cout << "myRansac3D took " << elapsedTime.count() << " milliseconds" << std::endl;
    return inliersResult;
}

#endif