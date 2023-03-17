#include "octree_voxel.h"
#include <gtest/gtest.h>

TEST(OctreeVoxelTest, InsertPointCloud)
{
    double resolution = 0.1;
    goodsalm::OctreeVoxel octree(resolution);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    // Add points
    // to the point cloud (you can also load a point cloud from a file)

    pcl::PointXYZINormal pt1;
    pt1.x = 1.0f;
    pt1.y = 1.0f;
    pt1.z = 1.0f;
    pt1.intensity = 0.5f;
    pt1.normal[0] = 0.0f;
    pt1.normal[1] = 0.0f;
    pt1.normal[2] = 1.0f;
    pcl::PointXYZINormal pt2;
    pt2.x = 4.0f;
    pt2.y = 4.0f;
    pt2.z = 4.0f;
    pt2.intensity = 0.5f;
    pt2.normal[0] = 0.0f;
    pt2.normal[1] = 0.0f;
    pt2.normal[2] = 1.0f;

    cloud->push_back(pt1);
    cloud->push_back(pt2);

    int key = octree.insertPointCloud(cloud, 0);
    ASSERT_EQ(key, 0); // Check if the assigned key is 0
}

TEST(OctreeVoxelTest, DeletePointCloud)
{
    double resolution = 0.1;
    goodsalm::OctreeVoxel octree(resolution);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::PointXYZINormal pt11;
    pt11.x = 1.0f;
    pt11.y = 1.0f;
    pt11.z = 1.0f;
    pt11.intensity = 0.5f;
    pt11.normal[0] = 0.0f;
    pt11.normal[1] = 0.0f;
    pt11.normal[2] = 1.0f;
    pcl::PointXYZINormal pt22;
    pt22.x = 4.0f;
    pt22.y = 4.0f;
    pt22.z = 4.0f;
    pt22.intensity = 0.5f;
    pt22.normal[0] = 0.0f;
    pt22.normal[1] = 0.0f;
    pt22.normal[2] = 1.0f;

    cloud->push_back(pt11);
    cloud->push_back(pt22);

    int key = octree.insertPointCloud(cloud, 0);

    bool deleted = octree.deletePointCloud(key);
    ASSERT_TRUE(deleted); // Check if the point cloud was successfully deleted
}

TEST(OctreeVoxelTest, UpdatePointCloud)
{
    double resolution = 0.1;
    goodsalm::OctreeVoxel octree(resolution);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointXYZINormal pt111;
    pt111.x = 1.0f;
    pt111.y = 1.0f;
    pt111.z = 1.0f;
    pt111.intensity = 0.5f;
    pt111.normal[0] = 0.0f;
    pt111.normal[1] = 0.0f;
    pt111.normal[2] = 1.0f;
    pcl::PointXYZINormal pt222;
    pt222.x = 4.0f;
    pt222.y = 4.0f;
    pt222.z = 4.0f;
    pt222.intensity = 0.5f;
    pt222.normal[0] = 0.0f;
    pt222.normal[1] = 0.0f;
    pt222.normal[2] = 1.0f;

    cloud->push_back(pt111);
    cloud->push_back(pt222);

    int key = octree.insertPointCloud(cloud, 0);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr updated_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::PointXYZINormal pt1111;
    pt1111.x = 1.0f;
    pt1111.y = 1.0f;
    pt1111.z = 1.0f;
    pt1111.intensity = 0.5f;
    pt1111.normal[0] = 0.0f;
    pt1111.normal[1] = 0.0f;
    pt1111.normal[2] = 1.0f;
    pcl::PointXYZINormal pt2222;
    pt2222.x = 4.0f;
    pt2222.y = 4.0f;
    pt2222.z = 4.0f;
    pt2222.intensity = 0.5f;
    pt2222.normal[0] = 0.0f;
    pt2222.normal[1] = 0.0f;
    pt2222.normal[2] = 1.0f;

    updated_cloud->push_back(pt1111);
    updated_cloud->push_back(pt2222);

    bool updated = octree.updatePointCloud(key, updated_cloud);
    ASSERT_TRUE(updated); // Check if the point cloud was successfully updated
}

// main function to run the tests
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
