/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

//
#include <unistd.h>
#include "myUtility.cpp"
#include "mySegmentation.cpp"
#include "myCluster.cpp"

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

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer, const int distance=16)
{
    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    //int distance = 16;

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
// Lesson : Lidar-4-2. Load PCD && Lidar-4-6, Steps For Obstacle Detection
void cityBlockVer1(pcl::visualization::PCLVisualizer::Ptr& viewer)
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

//-----------------------------------------------------------------------------------------
// Lesson : Lidar-4-6, Steps For Obstacle Detection
void cityBlockVer1Solution(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    //-----------------------------------------------------------------------------------------
    // Step-1 : Load PCD, Open 3D viewer and Display City Block
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //-----------------------------------------------------------------------------------------
    // Step-2 : Apply filter : Voxel Grid , Region of Interest , Remove roof poitns
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    float filterResolution       = 0.3; // mine is 0.1
    Eigen::Vector4f boxMinPoint  = Eigen::Vector4f(-10.0, -5.0, -2.0, 1);
    Eigen::Vector4f boxMaxPoint  = Eigen::Vector4f(  30.0, 8.0,  1.0, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor.FilterCloud(inputCloud, filterResolution, boxMinPoint, boxMaxPoint);

    //-----------------------------------------------------------------------------------------
    // Step-3 : Separate PointClouds to Plane and Obstacles
    int maxIterations       = 25; // mine is 10;
    float distanceThreshold = 0.3; // mine is 0.15;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filteredCloud, maxIterations, distanceThreshold);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));


    //-----------------------------------------------------------------------------------------
    // Step-4 : Euclidean Clustering with PCL
    float clusterTolerance = 0.53; // mine is 1.0;
    int minSize = 10; // mine is 5;
    int maxSize = 500; // mine is 1500;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
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
// Lesson : Lidar-4-7
void cityBlockVer2(pcl::visualization::PCLVisualizer::Ptr& viewer,
                   ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //-----------------------------------------------------------------------------------------
    // Step-1 : Load PCD, Open 3D viewer and Display City Block
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //-----------------------------------------------------------------------------------------
    // Step-2 : Apply filter : Voxel Grid , Region of Interest , Remove roof poitns
    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    float filterResolution       = 0.3; // mine is 0.1
    Eigen::Vector4f boxMinPoint  = Eigen::Vector4f(-10.0, -5.0, -2.0, 1);
    Eigen::Vector4f boxMaxPoint  = Eigen::Vector4f(  30.0, 8.0,  1.0, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterResolution, boxMinPoint, boxMaxPoint);

    //-----------------------------------------------------------------------------------------
    // Step-3 : Separate PointClouds to Plane and Obstacles
    int maxIterations       = 25; // mine is 10;
    float distanceThreshold = 0.3; // mine is 0.15;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, maxIterations, distanceThreshold);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    //-----------------------------------------------------------------------------------------
    // Step-4 : Euclidean Clustering with PCL
    float clusterTolerance = 0.53; // mine is 1.0;
    int minSize = 10; // mine is 5;
    int maxSize = 500; // mine is 1500;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        //-----------------------------------------------------------------------------------------
        // Lesson : Lidar-3-9. Bounding Boxes
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        //-----------------------------------------------------------------------------------------

        ++clusterId;
    }
}


//-----------------------------------------------------------------------------------------
// for Project
void cityBlockVer3(pcl::visualization::PCLVisualizer::Ptr& viewer,
                   ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //-----------------------------------------------------------------------------------------
    // Step-1 : Load PCD, Open 3D viewer and Display City Block
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //-----------------------------------------------------------------------------------------
    // Step-2 : Apply filter : Voxel Grid , Region of Interest , Remove roof poitns
    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    float filterResolution       = 0.3; // mine is 0.1
    Eigen::Vector4f boxMinPoint  = Eigen::Vector4f(-10.0, -5.0, -2.0, 1);
    Eigen::Vector4f boxMaxPoint  = Eigen::Vector4f(  30.0, 8.0,  1.0, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterResolution, boxMinPoint, boxMaxPoint);

    //-----------------------------------------------------------------------------------------
    // Step-3 : Separate PointClouds to Plane and Obstacles
    int maxIterations       = 25; // mine is 10;
    float distanceThreshold = 0.3; // mine is 0.15;
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, maxIterations, distanceThreshold);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = mySegmentPlane(filteredCloud, maxIterations, distanceThreshold);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    //-----------------------------------------------------------------------------------------
    // Step-4 : Euclidean Clustering with PCL
    float clusterTolerance = 0.53; // mine is 1.0;
    int minSize = 10; // mine is 5;
    int maxSize = 500; // mine is 1500;
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = myClustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        //-----------------------------------------------------------------------------------------
        // Lesson : Lidar-3-9. Bounding Boxes
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        //-----------------------------------------------------------------------------------------

        ++clusterId;
    }
}





//-------------------------------------------------------------------------//
void mainSingleShot()
{
    std::cout << "starting environment : mainSingleShot()" << std::endl;

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // Set Camera
    CameraAngle setAngle = TopDown; // XY, TopDown, Side, FPS
    const int cameraDistance = 30;
    initCamera(setAngle, viewer, cameraDistance);

    // Set Environment
    //simpleHighway(viewer);
    //cityBlockVer1(viewer);
    cityBlockVer1Solution(viewer);

    // Run Simulation
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}

//-------------------------------------------------------------------------//
void mainStream()
{
    std::cout << "starting environment : mainStream()" << std::endl;

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // Set Camera
    CameraAngle setAngle = TopDown; // XY, TopDown, Side, FPS
    const int cameraDistance = 30;
    initCamera(setAngle, viewer, cameraDistance);

    // Create instances
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // Create Stream Instance
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    // Run Simulation
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        // Set Environment
        cityBlockVer2(viewer,pointProcessorI,inputCloudI);

        // Update Iterator
        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
            usleep(1000.0*1000); // micro seconds
        }

        // Run
        viewer->spinOnce ();
        usleep(100.0*1000); // micro seconds
        //viewer->saveScreenshot("/home/takuya/HDD1/UdaCity/Fusion/SFND_Lidar_Obstacle_Detection/png/test2.png");
        //break;
    }
}

int main (int argc, char** argv)
{
    const int runType = 0;

    switch(runType)
    {
        case 0:
            mainSingleShot();
            break;
        case 1:
            mainStream();
            break;
        default:
            std::cout << "runType is wrong!" << std::endl;
            break;
    }
}