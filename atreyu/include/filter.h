#ifndef FILTER_H
#define FILTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "single_consumer_queue.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>  // include this header for avoiding segmentation fault when mls.process()
#include <pcl/features/normal_3d.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointNormalT;

class Filter
{
  private:
    Filter(const Filter &other) = delete;
    Filter &operator=(Filter other) = delete;

    single_consumer_queue<pcl::PointCloud<PointT>::Ptr> input_clouds;
    size_t max_num_input_clouds;

    // filters
    pcl::VoxelGrid<PointT> voxel_filter;
    pcl::RadiusOutlierRemoval<PointT> radius_filter;

#if USINGOPENMP
    /**
     * @brief mls. Object for computing surface smoothing and normals. For Using its OpenMP version you will need find_package(OpenMP) in the CMakeLists.txt file
     */
    pcl::MovingLeastSquaresOMP <PointT, PointNormalT> mls;  //<input type, output type>
#else
    /**
     * @brief mls. Object for computing surface smoothing and normals.
     */
    pcl::MovingLeastSquares <PointT, PointNormalT> mls;  //<input type, output type>
#endif

  public:
    /**
     * @brief Filter class constructor for enqueing pointclouds and applying consecutive filters. It configures and checks the filters values.
     * @param voxel_size_x is the leaf voxel size in x (meters) of the voxel-grid for downsampling.
     * @param voxel_size_y is the leaf voxel size in y (meters) of the voxel-grid for downsampling
     * @param voxel_size_z is the lead voxel size in z (meters) of the voxel-grid for downsampling. Use 0.1 for reducing the depth camera noise in z.
     * @param search_radius is the neighborhood radius for each point. It should be greater than voxel_size. Use voxel_size_x*2 for good results.
     * @param min_neighbors_in_radius is the number of neighbors to find within the search_radius.
     * @param max_num_input_clouds is the maximum number of pointclouds to be enqueued. When maximum, it discards the oldest.
     */
    Filter(float voxel_size_x = 0.037, float voxel_size_y = 0.037, float voxel_size_z = 0.1,
           float search_radius = 0.037*2, int min_neighbors_in_radius = 8,
           int max_num_input_clouds = 100);

    // pointclouds queues
    void enqueuePcl(pcl::PointCloud<PointT>::Ptr input);
    size_t getInputQueueSize();
    bool dataReady();
    void clearQueues();

    /**
     * @brief getFilteredCloud provides a filtered cloud using the enqueued clouds.
     * @param filtered is the cloud in which results will be saved.
     * @return true if a filtered pointcloud has been computed, false otherwise.
     */
    bool getFilteredCloud(pcl::PointCloud<PointT>::Ptr filtered);
    bool getFilteredCloud(pcl::PointCloud<PointNormalT>::Ptr withnormals);

};

#endif
