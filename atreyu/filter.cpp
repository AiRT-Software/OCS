#include "filter.h"

#include <log.h>
#include <iostream>
#include <sstream>

using namespace std;
using airt::Log;

// Constructor
Filter::Filter(float voxel_size_x, float voxel_size_y, float voxel_size_z,
               float search_radius, int min_neighbors_in_radius,
               int max_num_input_clouds)
{
    //checking values
    if (voxel_size_x < 0.03 || voxel_size_x > 0.1)
    {
        Log::error("voxel_size has to be in [0.03, 0.1] to avoid creating a big or small amount of points");
    }
    if (search_radius <= voxel_size_x)
    {
        Log::error("search_radius has to be greater than voxel_size");
    }
    if (min_neighbors_in_radius < 8 || min_neighbors_in_radius > 16)
    {
        Log::error("min_neighbors_in_radius has to be in [8, 16] to avoid removing all points or not removing any of them");
    }

    this->max_num_input_clouds = max_num_input_clouds;

    // configure voxel-filter
    voxel_filter.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);

    // configure radius-filter
    radius_filter.setRadiusSearch(search_radius);
    radius_filter.setMinNeighborsInRadius(min_neighbors_in_radius);

    // configure movingLeastSquares filter
#if USINGOPENMP
    mls.setNumberOfThreads(3);  //3 threads gives good performance.
#endif
    float mls_search_radius = search_radius * 2;
    mls.setSearchRadius(mls_search_radius); //m
    mls.setSqrGaussParam(mls_search_radius * mls_search_radius);  //recommended: search_radius^2
    pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>());  //same type as mls input type
    mls.setSearchMethod(kdtree);

    // If true: the surface and normal will be approximated using a polynomial estimation (if false, only a tangent one).
    mls.setPolynomialFit(false);
    mls.setPolynomialOrder(2);

    // we can use these methods: RANDOM_UNIFORM_DENSITY, SAMPLE_LOCAL_PLANE, VOXEL_GRID_DILATION
    // this is fastest and has good results
    mls.setUpsamplingMethod(mls.RANDOM_UNIFORM_DENSITY);
    mls.setPointDensity (50);  //desired num of points in radius

//    msl.setUpsamplingMethod(msl.SAMPLE_LOCAL_PLANE);
//    msl.setUpsamplingRadius (mls_search_radius*0.5);
//    msl.setUpsamplingStepSize (mls_search_radius*0.25);

//    msl.setUpsamplingMethod(msl.VOXEL_GRID_DILATION);
//    msl.setDilationIterations (2);
//    msl.setDilationVoxelSize (0.01);
}

void Filter::enqueuePcl(pcl::PointCloud<PointT>::Ptr input)
{
    if (input_clouds.size() < max_num_input_clouds)
    {
        input_clouds.enqueue(input);
    }
    else
    {
        // remove the oldest at the beginning of the queue
        input_clouds.dequeue();
        // add the newest at the end of the queue
        input_clouds.enqueue(input);
    }
}

void Filter::clearQueues()
{
    input_clouds.clear();
}

size_t Filter::getInputQueueSize()
{
    return input_clouds.size();
}

bool Filter::dataReady()
{
    constexpr int FilterSize = 1;

    return input_clouds.size() >= FilterSize;
}

bool Filter::getFilteredCloud(pcl::PointCloud<PointT>::Ptr filtered)
{
    /*
    pcl::PointCloud<PointT>::Ptr concat(new pcl::PointCloud<PointT>());
    while (input_clouds.size() > 0)
    {
        *concat += *input_clouds.dequeue();
    }
    */

    // voxel
//    voxel_filter.setInputCloud(concat);
    voxel_filter.setInputCloud(input_clouds.dequeue());
    pcl::PointCloud<PointT>::Ptr vf(new pcl::PointCloud<PointT>());
    voxel_filter.filter(* vf);

    // radius
    radius_filter.setInputCloud(vf);
    pcl::PointCloud<PointT>::Ptr rf(new pcl::PointCloud<PointT>());
    radius_filter.filter(* rf);

    // mls smoothing & normals
    if(rf->points.size() < 100)  //mls could fail when very few points
    {
        Log::error("MLS filter has received very few points. Smoothing was not applied.");
        pcl::copyPointCloud(* rf, * filtered);
    }
    else {
        mls.setComputeNormals(false);

        mls.setInputCloud(rf);
        pcl::PointCloud<PointNormalT>::Ptr smoothed(new pcl::PointCloud<PointNormalT>());  // same type as mls output type
        mls.process(* smoothed);

        // save xyzrgb
        pcl::copyPointCloud(* smoothed, * filtered);  // copies only attributes required by outputcloud
    }

    return true;
}

bool Filter::getFilteredCloud(pcl::PointCloud<PointNormalT>::Ptr withnormals)
{
    /*
    pcl::PointCloud<PointT>::Ptr concat(new pcl::PointCloud<PointT>());
    while (input_clouds.size() > 0)
    {
        *concat += *input_clouds.dequeue();
    }
    */

    // voxel
//    voxel_filter.setInputCloud(concat);
    voxel_filter.setInputCloud(input_clouds.dequeue());
    pcl::PointCloud<PointT>::Ptr vf(new pcl::PointCloud<PointT>());
    voxel_filter.filter(* vf);

    // radius
    radius_filter.setInputCloud(vf);
    pcl::PointCloud<PointT>::Ptr rf(new pcl::PointCloud<PointT>());
    radius_filter.filter(* rf);

    // mls smoothing & normals
    if(rf->points.size() < 100)  //mls could fail when very few points
    {
        Log::error("MLS filter has received very few points. Normals were not computed.");
        pcl::copyPointCloud(* rf, * withnormals);
    }
    else {
        mls.setComputeNormals(true);

        mls.setInputCloud(rf);
        pcl::PointCloud<PointNormalT>::Ptr smoothed(new pcl::PointCloud<PointNormalT>());  // same type as mls output type
        mls.process(* smoothed);

        // #pragma omp parallel for schedule(static)
        #pragma omp parallel for num_threads(2)  // Be careful with change of context. 2 threads is recommended.
        for(pcl::PointCloud<PointNormalT>::iterator it = smoothed->begin(); it < smoothed->end(); it++)
        {
            pcl::flipNormalTowardsViewpoint(* it, 0.0, 0.0, 0.0, it->normal_x, it->normal_y, it->normal_z);
        }

        // save with xyzrgbnormal
        * withnormals = * smoothed;
    }

    return true;
}
