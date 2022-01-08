/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData2D()
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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
    for(int i=0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        while(inliers.size() < 2)
        {
            // If inliers has the same value, below will fail.
            // so, inliers.sise is not incremented.
            inliers.insert(rand()%(cloud->points.size()));
        }

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        float x1, y1, x2, y2;

        // get the 1st pointer of inliers
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;

        // move to the 2nd pointer of inliers
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        // ax + by + c = 0
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1*y2 - x2*y1;

        for(int index=0; index<cloud->points.size(); index++)
        {
            // inliers has already 2 points.
            // To avoid to set the same value, I check it below.
            if(inliers.count(index) > 0)
            {
                continue;
            }

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;
            float d  = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);

            if(d <= distanceTol)
            {
                inliers.insert(index);
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
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

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
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData2D();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac2D(cloud, 10, 1.0);
    std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>()); // Plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>()); // Obstacles

    // Check All points.
	for(int index = 0; index < cloud->points.size(); index++)
	{
        std::cout << "Ransac3D, index = " << index << std::endl;
		pcl::PointXYZ point = cloud->points[index]; // pick up each point

		if(inliers.count(index)) // check if this point is in inliers.
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
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
