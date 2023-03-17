// octree_voxel.cpp
#include "octree_voxel.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace goodsalm
{

    // OctreeVoxel::OctreeVoxel(double resolution)
    //     : resolution_(resolution),
    //       octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZINormal>(resolution)),
    //       next_key_(0) {}

    OctreeVoxel::OctreeVoxel(double resolution)
        : resolution_(resolution),
          octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution)) {}

    OctreeVoxel::~OctreeVoxel() {}

    int OctreeVoxel::insertPointCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int key)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_key = convertCloudWithKey(cloud, key);
        voxelGridFilter(cloud_key);

        {
            std::unique_lock<std::mutex> lock(octree_mutex_);
            insertPointCloudToOctree(cloud_key);
            point_cloud_map_[key] = cloud_key;
        }

        return key;
    }

    bool OctreeVoxel::deletePointCloud(int key)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        auto iter = point_cloud_map_.find(key);
        if (iter == point_cloud_map_.end())
        {
            return false; // Key not found
        }

        deletePointCloudFromOctree(iter->second);
        point_cloud_map_.erase(iter);

        return true;
    }

    bool OctreeVoxel::updatePointCloud(int key, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        auto iter = point_cloud_map_.find(key);
        if (iter == point_cloud_map_.end())
        {
            return false; // Key not found
        }

        deletePointCloudFromOctree(iter->second);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_key = convertCloudWithKey(cloud, key);
        voxelGridFilter(cloud_key);
        insertPointCloudToOctree(cloud_key);

        iter->second = cloud_key;

        return true;
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr OctreeVoxel::convertCloudWithKey(
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int key)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_key(new pcl::PointCloud<pcl::PointXYZINormal>());

        // Reserve space for the new cloud
        cloud_with_key->reserve(cloud->size());

        for (const auto &point : *cloud)
        {
            pcl::PointXYZINormal point_with_key;

            point_with_key.x = point.x;
            point_with_key.y = point.y;
            point_with_key.z = point.z;
            point_with_key.intensity = point.intensity;
            point_with_key.normal_x = point.normal_x;
            point_with_key.normal_y = point.normal_y;
            point_with_key.normal_z = point.normal_z;
            // point_with_key.key = key;

            cloud_with_key->push_back(point_with_key);
        }

        return cloud_with_key;
    }

    void OctreeVoxel::voxelGridFilter(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        pcl::VoxelGrid<pcl::PointXYZINormal> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(resolution_, resolution_, resolution_);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
        voxel_grid.filter(*filtered_cloud);

        cloud->swap(*filtered_cloud);
    }

    void OctreeVoxel::insertPointCloudToOctree(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);
        // for (const auto &point : cloud->points)
        // {
        //     octree_->addPoint(point);
        // }
        octree_->setInputCloud(cloud);
        octree_->addPointsFromInputCloud();
    }

    void OctreeVoxel::deletePointCloudFromOctree(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        for (const auto &point : cloud->points)
        {
            octree_->deleteVoxelAtPoint(point);
        }
    }

    bool OctreeVoxel::kNearestSearch(const pcl::PointXYZINormal &search_point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        if (octree_->getInputCloud() == nullptr || octree_->getInputCloud()->points.empty())
        {
            return false;
        }

        return octree_->nearestKSearch(search_point, k, k_indices, k_sqr_distances) > 0;
    }

    bool OctreeVoxel::radiusSearch(const pcl::PointXYZINormal &search_point, double radius, std::vector<int> &indices, std::vector<float> &sqr_distances)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        if (octree_->getInputCloud() == nullptr || octree_->getInputCloud()->points.empty())
        {
            return false;
        }

        return octree_->radiusSearch(search_point, radius, indices, sqr_distances) > 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeVoxel::getLocalMap(const pcl::PointXYZ &search_point, double radius)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        if (octree_->getInputCloud() == nullptr || octree_->getInputCloud()->points.empty())
        {
            return nullptr;
        }

        std::vector<int> indices;
        std::vector<float> sqr_distances;
        if (octree_->radiusSearch(search_point, radius, indices, sqr_distances) > 0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*octree_->getInputCloud(), indices, *local_map);
            return local_map;
        }

        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr OctreeVoxel::mergePointClouds(const std::vector<int> &keys)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

        for (const int key : keys)
        {
            auto it = point_cloud_map_.find(key);
            if (it != point_cloud_map_.end())
            {
                *merged_cloud += *(it->second);
            }
            else
            {
                // Handle the case when the key is not found in the point_cloud_map_
                std::cerr << "Key " << key << " not found in the point_cloud_map_" << std::endl;
            }
        }

        return merged_cloud;
    }

    bool OctreeVoxel::loadPointCloud(int key, const std::string &cloud_path)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        // Check if the key already exists
        if (point_cloud_map_.find(key) != point_cloud_map_.end())
        {
            std::cerr << "Error: Point cloud with key " << key << " already exists." << std::endl;
            return false;
        }

        // Load point cloud from PCD file
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        if (pcl::io::loadPCDFile(cloud_path, *cloud) == -1)
        {
            std::cerr << "Error: Couldn't read point cloud file " << cloud_path << std::endl;
            return false;
        }

        // Convert the point cloud to pcl::PointXYZ type
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_with_key = convertCloudWithKey(cloud, key);

        // Voxel grid filter
        voxelGridFilter(cloud_with_key);

        // Insert point cloud into octree
        insertPointCloudToOctree(cloud_with_key);

        // Add the point cloud to the map
        point_cloud_map_[key] = cloud_with_key;

        return true;
    }

    bool OctreeVoxel::unloadPointCloud(int key)
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        // Check if the key exists
        auto it = point_cloud_map_.find(key);
        if (it == point_cloud_map_.end())
        {
            std::cerr << "Error: Point cloud with key " << key << " not found." << std::endl;
            return false;
        }

        // Remove point cloud from octree
        deletePointCloudFromOctree(it->second);

        // Remove point cloud from map
        point_cloud_map_.erase(it);

        return true;
    }

    void OctreeVoxel::adjustVoxelSize()
    {
        std::unique_lock<std::mutex> lock(octree_mutex_);

        // Iterate through all point clouds in point_cloud_map_ and compute density
        double total_density = 0;
        int num_clouds = 0;

        for (const auto &kv : point_cloud_map_)
        {
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud = kv.second;
            double density = cloud->size() / (cloud->sensor_origin_.norm() * cloud->sensor_origin_.norm() * M_PI);
            total_density += density;
            num_clouds++;
        }
        if (num_clouds == 0)
        {
            return;
        }
        // Compute average density
        double avg_density = total_density / num_clouds;

        // Set new resolution based on average density
        // double new_resolution = std::clamp(1.0 / std::sqrt(avg_density), 0.1, 10.0);
        double new_resolution = std::max(std::min(1.0 / std::sqrt(avg_density), 10.0), 0.1);

        // Update octree resolution if it's different from the current resolution
        if (std::abs(new_resolution - resolution_) > 1e-5)
        {
            resolution_ = new_resolution;
            octree_->setResolution(resolution_);
        }
    }

    // Implement other custom features or algorithms as needed.

} // namespace goodsalm