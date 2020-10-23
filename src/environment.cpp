/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <memory>

#define ENABLE_LIDAR_OBSTACLE_DETECTION_WITH_PCL_BUILTIN 1
#define ENABLE_LIDAR_OBSTACLE_DETECTION_WITH_UD_ROUTINES 0
#define ENABLE_SIMPLE_HIGHWAY_SIMULATION 0

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

#if ENABLE_LIDAR_OBSTACLE_DETECTION_WITH_UD_ROUTINES
template<typename PointT>
void cityBlockUD(pcl::visualization::PCLVisualizer::Ptr& viewer, std::unique_ptr<ProcessPointClouds<PointT>>& process_pointcloud, typename pcl::PointCloud<PointT>::Ptr& traffic_cloud)
{
    const Eigen::Vector4f min_vector(-25, -6, -2.5, 1);
    const Eigen::Vector4f max_vector(25, 6, 2.5, 1);
    auto filtered_cloud = process_pointcloud->FilterCloud(traffic_cloud, 0.2f, min_vector, max_vector);
    auto segmented_pointcloud = process_pointcloud->RansacPlane(filtered_cloud, 100, 0.2);
    renderPointCloud(viewer, segmented_pointcloud.first, "PlaneCloud", Color(1, 0.8, 0.2));
    auto obstacle_clusters = process_pointcloud->ClusteringUD(segmented_pointcloud.second, 0.5f, 15, 1500);
    size_t clusterId, cluster_color {0};
    std::vector<Color> colors = {Color(1,0.6,0.2), Color(0,1,0), Color(0,0,1)};

    for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : obstacle_clusters)
    {
        std::cout << "cluster size ";
        process_pointcloud->numPoints(cluster);
        cluster_color = clusterId % colors.size();
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[cluster_color]);
        Box box = process_pointcloud->BoundingBox(cluster);
//        BoxQ box = process_pointcloud->SBoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}
#endif

#if ENABLE_LIDAR_OBSTACLE_DETECTION_WITH_PCL_BUILTIN

template<typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, std::unique_ptr<ProcessPointClouds<PointT>>& process_pointcloud, typename pcl::PointCloud<PointT>::Ptr& traffic_cloud)
{
//    auto process_pointcloud = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
//    auto traffic_cloud = process_pointcloud->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    auto filtered_cloud = process_pointcloud->FilterCloud(traffic_cloud, 0.1f, Eigen::Vector4f(-25, -6.5, -2.5, 1), Eigen::Vector4f(25, 6.5, 2.5, 1));
//    renderPointCloud(viewer, filtered_cloud, "trafficCloud");
    auto segmented_pointcloud = process_pointcloud->SegmentPlane(filtered_cloud, 100, 0.2);
    renderPointCloud(viewer, segmented_pointcloud.first, "PlaneCloud", Color(1, 0.8, 0.2));
//    renderPointCloud(viewer, segmented_pointcloud.second, "ObstacleCloud", Color(0, 1, 0));
    auto obstacle_clusters = process_pointcloud->Clustering(segmented_pointcloud.second, 0.5f, 25, 1800);
    size_t clusterId, cluster_color {0};
    std::vector<Color> colors = {Color(1,0.6,0.2), Color(0,1,0), Color(0,0,1)};

    for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : obstacle_clusters)
    {
        std::cout << "cluster size ";
        process_pointcloud->numPoints(cluster);
        cluster_color = clusterId % colors.size();
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[cluster_color]);
        Box box = process_pointcloud->BoundingBox(cluster);
//        BoxQ box = process_pointcloud->SBoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }


}
#endif


#if ENABLE_SIMPLE_HIGHWAY_SIMULATION
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    auto ego_lidar = std::make_unique<Lidar>(cars, 0);
    auto traffic_pointcloud = ego_lidar->scan();
    renderRays(viewer,  ego_lidar->position, traffic_pointcloud);
    renderPointCloud(viewer, traffic_pointcloud, "lidar scatter");
    auto process_pointcloud = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();
    auto segmented_pointcloud = process_pointcloud->SegmentPlane(traffic_pointcloud, 100, 0.2);
    renderPointCloud(viewer, segmented_pointcloud.first, "PlaneCloud", Color(1, 0.8, 0.6));
    renderPointCloud(viewer, segmented_pointcloud.second, "ObstacleCloud", Color(0, 1, 0));
    auto obstacle_clusters = process_pointcloud->Clustering(segmented_pointcloud.second, 1.0f, 3, 50);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(0,1,0), Color(0,0,1)};

    for(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster : obstacle_clusters)
    {
        std::cout << "cluster size ";
        process_pointcloud->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        Box box = process_pointcloud->BoundingBox(cluster);
//        BoxQ box = process_pointcloud->SBoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}
#endif

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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
#if ENABLE_SIMPLE_HIGHWAY_SIMULATION
    simpleHighway(viewer);
#endif
    auto point_processor = std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>>();
    std::vector<boost::filesystem::path> pcl_stream = point_processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto pcl_stream_iter = pcl_stream.begin();
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_frames;
    for (const auto& pcl_stream_frame: pcl_stream)
    {
        cloud = point_processor->loadPcd((pcl_stream_frame).string());
        cloud_frames.emplace_back(std::move(cloud));
    }

    auto cloud_frames_iter = cloud_frames.begin();



    while (!viewer->wasStopped ())
    {
#if ENABLE_LIDAR_OBSTACLE_DETECTION_WITH_PCL_BUILTIN
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        cloud = point_processor->loadPcd((*pcl_stream_iter).string());
        cityBlock<pcl::PointXYZI> (viewer, point_processor, cloud);
        pcl_stream_iter++;
        if(pcl_stream_iter == pcl_stream.end())
            pcl_stream_iter = pcl_stream.begin();
#endif

#if ENABLE_LIDAR_OBSTACLE_DETECTION_WITH_UD_ROUTINES
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        cityBlockUD<pcl::PointXYZI> (viewer, point_processor, *cloud_frames_iter);
        cloud_frames_iter++;
        if(cloud_frames_iter == cloud_frames.end())
            cloud_frames_iter = cloud_frames.begin();
#endif
        viewer->spinOnce ();
    } 
}