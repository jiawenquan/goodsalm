// octree_voxel.h
#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree.h>

// #include "point_xyzi_normal_key.h"
#include <unordered_map>
#include <mutex> // Add this include

namespace goodsalm
{

    // Define PointXYZINormalKey class
    struct PointXYZINormalKey : public pcl::PointXYZINormal
    {
        int key;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

} // namespace goodsalm

POINT_CLOUD_REGISTER_POINT_STRUCT(goodsalm::PointXYZINormalKey,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(int, key, key))

namespace goodsalm
{

    class OctreeVoxel
    {
    public:
        OctreeVoxel(double resolution);
        ~OctreeVoxel();

        // Insert a new point cloud frame with specified key
        int insertPointCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int key);

        // Delete a point cloud frame
        bool deletePointCloud(int key);

        // Update an existing point cloud frame with specified key
        bool updatePointCloud(int key, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        pcl::PointCloud<PointXYZINormalKey>::Ptr convertCloudWithKey(
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int key);

        // Perform voxel the point cloud
        void voxelGridFilter(pcl::PointCloud<PointXYZINormalKey>::Ptr cloud);

        // K-Nearest Neighbors search
        bool kNearestSearch(const PointXYZINormalKey &search_point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances);

        // Fixed-radius neighbors search
        bool radiusSearch(const PointXYZINormalKey &search_point, double radius, std::vector<int> &indices, std::vector<float> &sqr_distances);

        // Get local map
        pcl::PointCloud<PointXYZINormalKey>::Ptr getLocalMap(const PointXYZINormalKey &search_point, double radius);

        // Merge point cloud frames
        pcl::PointCloud<PointXYZINormalKey>::Ptr mergePointClouds(const std::vector<int> &keys);

        // Load point cloud frame
        bool loadPointCloud(int key, const std::string &cloud_path);

        // Unload point cloud frame
        bool unloadPointCloud(int key);

        // Adjust voxel size automatically
        void adjustVoxelSize();

        // Visualize the octree structure
        void visualizeOctree();

        // features or algorithms

    private:
        std::mutex octree_mutex_; // Add this member variable

        pcl::octree::OctreePointCloudSearch<PointXYZINormalKey>::Ptr octree_;
        // int next_key_;

        // New member variable to store point clouds by key
        std::unordered_map<int, pcl::PointCloud<PointXYZINormalKey>::Ptr> point_cloud_map_;

        double resolution_;

        // Helper functions
        void insertPointCloudToOctree(pcl::PointCloud<PointXYZINormalKey>::Ptr cloud);
        void deletePointCloudFromOctree(pcl::PointCloud<PointXYZINormalKey>::Ptr cloud);
    };

} // namespace goodsalm
