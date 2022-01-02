/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "myUtility.cpp"

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
    // create Lidar instance
    Lidar* lidar = new Lidar(cars, 0.0); // get dynamic memory on heap

    // get All PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); // inputCloud is a shared pointer

    // render something
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    // create pointProcessor instance
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // on stack ()
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); // on heap (pointer)

    // Separate PointClouds to Plane and Obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    // render something
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    //-----------------------------------------------------------------------------------------
    // Lesson : Lidar-3-3. Euclidean Clustering with PCL
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        //-----------------------------------------------------------------------------------------
        // Lesson : Lidar-3-9. Bounding Boxes 
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        //-----------------------------------------------------------------------------------------

        ++clusterId;
    }
}


//-----------------------------------------------------------------------------------------
// Lesson : Lidar-4-2. Load PCD
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    //-----------------------------------------------------------------------------------------
    // Settings
    bool renderRaw           = false;
    bool renderFilteredCloud = false;
    bool renderObstacles     = false;
    bool renderPlane         = true;
    bool renderCluster       = true;
    bool renderClusterBox    = true;
    float boxLength          = 20.0;
    float boxWidth           = 6.5;

    //-----------------------------------------------------------------------------------------
    // Step-1 : Load PCD, Open 3D viewer and Display City Block
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  
    if(renderRaw){
        renderPointCloud(viewer, inputCloud, "InputCloud");
    }
    
    //-----------------------------------------------------------------------------------------
    // Step-2 : Apply filter : Voxel Grid , Region of Interest , Remove roof poitns
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    float filterResolution       = 0.1f;
    Eigen::Vector4f boxMinPoint  = Eigen::Vector4f(-boxLength, -boxWidth, -10.0, 1);
    Eigen::Vector4f boxMaxPoint  = Eigen::Vector4f( boxLength,  boxWidth,  10.0, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor.FilterCloud(inputCloud, filterResolution, boxMinPoint, boxMaxPoint);
 
    if(renderFilteredCloud){
        renderPointCloud(viewer, filteredCloud, "FilteredCloud");
    }
    
    //-----------------------------------------------------------------------------------------
    // Step-3 : Separate PointClouds to Plane and Obstacles
    int maxIterations       = 10;
    float distanceThreshold = 0.15;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filteredCloud, maxIterations, distanceThreshold);
   
    if(renderObstacles){
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    }
    if(renderPlane){
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    }

    //-----------------------------------------------------------------------------------------
    // Step-4 : Euclidean Clustering with PCL
    float clusterTolerance = 1.0;
    int minSize = 5;
    int maxSize = 1500;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);

        if(renderCluster){
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }

        //-----------------------------------------------------------------------------------------
        // Lesson : Lidar-3-9. Bounding Boxes
        if(renderClusterBox){ 
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        //-----------------------------------------------------------------------------------------

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
    CameraAngle setAngle = TopDown; // XY, TopDown, Side, FPS
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    
    //-----------------------------------------------------------------------------------------
    // Lesson : Lidar-4-2. Load PCD
    cityBlock(viewer);

    std::string folderPath  = "/home/workspace/Udacity_Fusion_Lidar";
    std::string fileNamePng = folderPath + "/png/L4_6_" + getCurrentTime("") + ".png";
    std:cout << fileNamePng << std::endl;
    //pcl::visualization::PCLVisualizer pngObj("PNG OBJ");
    

    /*
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
    */ 

    viewer->saveScreenshot(fileNamePng);
    for(int i=0; i<1000; i++)
    {
        viewer->spinOnce ();
    } 
    //viewer->saveScreenshot(fileNamePng);
    pcl::io::savePNGFile(fileNamePng, *viewer, "rgb");
}