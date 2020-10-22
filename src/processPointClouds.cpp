// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg_filter;
    auto voxel_cloud_filtered = std::make_shared<pcl::PointCloud<PointT>>();
    vg_filter.setInputCloud(cloud);
    vg_filter.setLeafSize(filterRes, filterRes, filterRes);
    vg_filter.filter(*voxel_cloud_filtered);

    auto cloud_region = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::CropBox<PointT> crop_filter(true); //true means extract indices of points being removed
    crop_filter.setMin(minPoint);
    crop_filter.setMax(maxPoint);
    crop_filter.setInputCloud(voxel_cloud_filtered);
    crop_filter.filter(*cloud_region);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof_region(true);
    roof_region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof_region.setMin(Eigen::Vector4f(2.6, 1.7, -4, 1));
    roof_region.setInputCloud(cloud_region);
    roof_region.filter(indices);

    auto inliers_indices = std::make_shared<pcl::PointIndices>();
    for(const int point_idx: indices)
        inliers_indices->indices.push_back(point_idx);
    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(cloud_region);
    extract_indices.setIndices(inliers_indices);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers_point_indices, typename pcl::PointCloud<PointT>::Ptr segmented_cloud)
{

    auto plane_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    auto obstacle_cloud = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(segmented_cloud);
    extract_indices.setIndices(inliers_point_indices);
    extract_indices.setNegative(false);
    extract_indices.filter(*plane_cloud);

    extract_indices.setNegative(true);
    extract_indices.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane_cloud, obstacle_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers_indices = std::make_shared<pcl::PointIndices>();
    pcl::SACSegmentation<PointT> seg_plane;
    pcl::ModelCoefficients::Ptr coefficients = std::make_shared<pcl::ModelCoefficients>();
    seg_plane.setOptimizeCoefficients(true);
    seg_plane.setModelType(pcl::SACMODEL_PLANE);
    seg_plane.setMethodType(pcl::SAC_RANSAC);
    seg_plane.setMaxIterations(maxIterations);
    seg_plane.setDistanceThreshold(distanceThreshold);
    seg_plane.setInputCloud(cloud);
    seg_plane.segment(*inliers_indices, *coefficients);
    if (inliers_indices->indices.size() == 0 )
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers_indices, cloud);
    std::cout << "segmentation and Separating Point Cloud took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    auto kdtree = std::make_shared<pcl::search::KdTree<PointT>>();
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); //l2 norm/distance between the points
    ec.setMaxClusterSize(maxSize);
    ec.setMinClusterSize(minSize);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    auto cloud_cluster = std::make_shared<pcl::PointCloud<PointT>>();
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    for(auto& point_indices: cluster_indices)
    {
        std::cout<< "size of obstacale cluster :"<< point_indices.indices.size()<<std::endl;
        for(const auto& idx: point_indices.indices)
            cloud_cluster->push_back(cloud->at(idx));
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(std::move(cloud_cluster));
        cloud_cluster.reset(new pcl::PointCloud<PointT>);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::getProximityCluster(uint point_index, std::vector<int>& single_cluster_indices, std::vector<bool>& idx_mask, const std::vector<std::vector<float>>& points, std::unique_ptr<KdTree>& kdtree, const float distanceTol)
{
    idx_mask[point_index] = true;
    single_cluster_indices.push_back(point_index);

    std::vector<int> nearest_points_indices = kdtree->search(points[point_index], distanceTol);
    for (const auto& idx: nearest_points_indices) //terminating condition of recursion
    {
        if(!idx_mask[idx])
            getProximityCluster(idx, single_cluster_indices, idx_mask, points, kdtree, distanceTol);
    }

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, std::unique_ptr<KdTree>& tree, const float distanceTol)
{

	std::vector<std::vector<int>> clusters_indices;
	std::vector<bool> cluster_index_mask(points.size(), false); //true if point is part of the cluster else false

	for (uint point_idx{0}; point_idx < points.size(); point_idx++)
    {

	    if (cluster_index_mask[point_idx])
            continue;
        std::vector<int> single_cluster;
	    getProximityCluster(point_idx, single_cluster, cluster_index_mask, points, tree, distanceTol);
	    clusters_indices.push_back(single_cluster);
    }

	return clusters_indices;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringUD(typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // kd tree
    auto tree = std::make_unique<KdTree>();
    std::vector<std::vector<float>> points;
    for (int i=0; i<cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        
        std::vector<float> point_vector;
        point_vector.push_back(point.x);
        point_vector.push_back(point.y);
        point_vector.push_back(point.z);

        tree->insert(point_vector, i); 
        points.push_back(point_vector);
    }

    // cluster
    std::vector<std::vector<int>> clusters_indices = euclideanCluster(points, tree, clusterTolerance);

    for(const std::vector<int>& cluster_idx : clusters_indices)
    {
        auto clusterCloud = std::make_shared<pcl::PointCloud<PointT>>();
        for (const int& idx: cluster_idx)
        {
            clusterCloud->points.push_back(cloud->points[idx]);
        }
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        if ((clusterCloud->width >= minSize) and (clusterCloud->width <= maxSize))
            clusters.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
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
        float x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c, d, curr_x, curr_y, curr_z;
        auto point_idx_iter = inlier_idx.begin();
        x1 = cloud->points.at(*point_idx_iter).x;
        y1 = cloud->points.at(*point_idx_iter).y;
        z1 = cloud->points.at(*point_idx_iter).z;
        point_idx_iter++;
        x2 = cloud->points.at(*point_idx_iter).x;
        y2 = cloud->points.at(*point_idx_iter).y;
        z2 = cloud->points.at(*point_idx_iter).z;
        point_idx_iter++;
        x3 = cloud->points.at(*point_idx_iter).x;
        y3 = cloud->points.at(*point_idx_iter).y;
        z3 = cloud->points.at(*point_idx_iter).z;

        //define co-efficients of the plane eq. ax + by + cz + d = 0
        a = (y2-y1)*(z3-z1) - (y3-y1)*(z2-z1);
        b = (x2-x1)*(z3-z1) - (x3-x1)*(z2-z1);
        c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        d = -a * x1 - b * y1 - c * z1;


        //measure the distance btw point and the current plane
        for(auto curr_point_idx = 0; curr_point_idx < cloud->points.size(); curr_point_idx++)
        {
            if(inlier_idx.count(curr_point_idx) > 0)
                continue;
            PointT curr_point = cloud->points.at(curr_point_idx);
            curr_x = curr_point.x;
            curr_y = curr_point.y;
            curr_z = curr_point.z;


            float point_plane_distance = fabs(a*curr_x + b*curr_y + c*curr_z + d)/sqrt(a*a + b*b + c*c);
            if(point_plane_distance <= distanceThreshold)
                inlier_idx.insert(curr_point_idx);

        }
        if (inlier_idx.size() > inliers_result.size())
            inliers_result = inlier_idx;

    }
//    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
//    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
    auto cloudInliers = std::make_shared<pcl::PointCloud<PointT>>();
    auto cloudOutliers = std::make_shared<pcl::PointCloud<PointT>>();

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers_result.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;

}




template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

//TODO: correct volume and orientation
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::SBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    Eigen::Vector4f pca_centeroid;
    Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(*cluster, pca_centeroid);
    pcl::computeCovarianceMatrixNormalized(*cluster, pca_centeroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors_pca = eigen_solver.eigenvectors();
    Eigen::Vector3f eigen_values_pca = eigen_solver.eigenvalues();
    eigen_vectors_pca.col(2) = eigen_vectors_pca.col(0).cross(eigen_vectors_pca.col(1));
//    eigen_vectors_pca.col(1) = eigen_vectors_pca.col(2).cross(eigen_vectors_pca.col(0));
//    eigen_vectors_pca.col(0) = eigen_vectors_pca.col(1).cross(eigen_vectors_pca.col(2));

//    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection(new pcl::PointCloud<PointT>);
//    pcl::PCA<PointT> pca;
//    pca.setInputCloud(cluster);
//    pca.project(*cluster, *cloudPCAprojection);
//    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
//    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;



    Eigen::Matrix4f cloud_projection_tm(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f cloud_projection_tm_inv(Eigen::Matrix4f::Identity());

    cloud_projection_tm.block<3,3>(0,0) = eigen_vectors_pca.transpose();
    cloud_projection_tm.block<3,1>(0,3) = -1.f * ((cloud_projection_tm.block<3,3>(0,0)) * pca_centeroid.head<3>());

    auto cloud_points_projected = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::transformPointCloud(*cluster, *cloud_points_projected, cloud_projection_tm);

    PointT min_point, max_point;
    pcl::getMinMax3D(*cloud_points_projected, min_point, max_point);

    const Eigen::Vector3f mean_diagonal = 0.5f *(max_point.getVector3fMap() + min_point.getVector3fMap());
    const Eigen::Quaternionf bbox_quaternion(eigen_vectors_pca);
    const Eigen::Vector3f bbox_transform = eigen_vectors_pca * mean_diagonal + pca_centeroid.head<3>();

    BoxQ boundingboxq;
    boundingboxq.bboxQuaternion = bbox_quaternion;
    boundingboxq.bboxTransform = bbox_transform;
    boundingboxq.cube_width = max_point.x - min_point.x;
    boundingboxq.cube_height = max_point.y - min_point.y;
    boundingboxq.cube_length = max_point.z - min_point.z;

    return boundingboxq;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}