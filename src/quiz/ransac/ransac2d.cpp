/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
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

template<typename PointT>
std::unordered_set<int> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliers_result;
    srand(time(NULL));

    while(maxIterations--)
    {
        std::unordered_set<int> inlier_idx;
        while (inlier_idx.size() < 3)
        {
              inlier_idx.insert(rand()% cloud->points.size()); //select the idx_value below the max cloud size
        }
        //Sample 3 points from the cloud to define a plane
        auto point_idx_iter = inlier_idx.begin();
        float x1 = cloud->points.at(*point_idx_iter).x;
        float y1 = cloud->points.at(*point_idx_iter).y;
        float z1 = cloud->points.at(*point_idx_iter).z;
        point_idx_iter++;
        float x2 = cloud->points.at(*point_idx_iter).x;
        float y2 = cloud->points.at(*point_idx_iter).y;
        float z2 = cloud->points.at(*point_idx_iter).z;
        point_idx_iter++;
        float x3 = cloud->points.at(*point_idx_iter).x;
        float y3 = cloud->points.at(*point_idx_iter).y;
        float z3 = cloud->points.at(*point_idx_iter).z;

        //define co-efficients of the plane eq. ax + by + cz + d = 0
        float a = (y2-y1)*(z3-z1) - (y3-y1)*(z2-z1);
        float b = (x2-x1)*(z3-z1) - (x3-x1)*(z2-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = -a * x1 - b * y1 - c * z1;


        //measure the distance btw point and the current plane
        for(auto curr_point_idx = 0; curr_point_idx < cloud->points.size(); curr_point_idx++)
        {
            if(inlier_idx.count(curr_point_idx) > 0)
                continue;
            float curr_x = cloud->points.at(curr_point_idx).x;
            float curr_y = cloud->points.at(curr_point_idx).y;
            float curr_z = cloud->points.at(curr_point_idx).z;

            float point_plane_distance = fabs(a*curr_x + b*curr_y + c*curr_z + d)/sqrt(a*a + b*b + c*c);
            if(point_plane_distance < distanceThreshold)
                inlier_idx.insert(curr_point_idx);

        }
        if (inlier_idx.size() > inliers_result.size())
            inliers_result = inlier_idx;

    }

    return inliers_result;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for RansacLine function
	//std::unordered_set<int> inliers = RansacLine(cloud, 0, 0);
	auto inliers_plane = RansacPlane<pcl::PointXYZ>(cloud3D, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud3D->points.size(); index++)
	{
		pcl::PointXYZ point = cloud3D->points[index];
		if(inliers_plane.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers_plane.size())
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
