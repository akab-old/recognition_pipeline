#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

//DEBUG
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

typedef PointXYZRGBA PointT;

class PlaneEstimator{
public:

    PointCloud<PointT>::Ptr source_cloud;
    PointCloud<PointT>::Ptr scene_noplanes;
    ModelCoefficients::Ptr plane_coeff;
    PointIndices::Ptr plane_indices;
    vector<PointCloud<PointT>::Ptr > planes;
    vector< PointCloud<PointNormal>::Ptr> clusters;

    PlaneEstimator(){

    }

    PlaneEstimator(PointCloud<PointT>::Ptr scene){
      source_cloud = scene;
    }

    ~PlaneEstimator(){

    }

    vector<PointCloud<PointNormal>::Ptr> getClusters(){
      return clusters;
    }
 

    void subPlanes(){
    PointCloud<PointT>::Ptr cloud_p (new PointCloud<PointT>), cloud_f (new PointCloud<PointT>);

    ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
    PointIndices::Ptr inliers (new PointIndices ());

    // Create the segmentation object
    SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    ExtractIndices<PointT> extract;

    int i = 0, nr_points = (int) source_cloud->points.size ();
    // While 30% of the original cloud is still there
    while (source_cloud->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (source_cloud);
      seg.segment (*inliers, *coefficients);

      plane_coeff = coefficients;
      plane_indices = inliers;

      if (inliers->indices.size () == 0)
      {
        cerr << "Could not estimate a planar model for the given dataset." << endl;
        break;
      }

      // Extract the inliers
      extract.setInputCloud (source_cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);

      double avg_height = 0;
      double zs = 0;
      for(unsigned int j = 0; j < cloud_p->points.size(); j++)
        zs += cloud_p->points[j].z;
      avg_height = zs/cloud_p->points.size();
      // cout << "avg_height " << avg_height << endl; 

      if(avg_height < 1){
        planes.push_back(cloud_p);
    }

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      source_cloud.swap (cloud_f);
      i++;
    }

    scene_noplanes = source_cloud;
  }

void convexHullClustering(){
  PointCloud<PointT>::Ptr convexHull(new PointCloud<PointT>);
  PointCloud<PointT>::Ptr objs(new PointCloud<PointT>);
  ConvexHull<PointT> hull;

  ExtractIndices<PointT> extract;
  extract.setInputCloud(source_cloud);
  extract.setIndices(plane_indices);

  PointCloud<PointT>::Ptr p = planes.at(0);
  hull.setInputCloud(p);
  hull.setDimension(2);
  hull.reconstruct(*convexHull);

  ExtractPolygonalPrismData<PointT> prism;
  prism.setInputCloud(source_cloud);
  prism.setInputPlanarHull(convexHull);
  prism.setHeightLimits(0.0f,0.1f);
  PointIndices::Ptr objectIndices(new PointIndices);
  prism.segment(*objectIndices);

  extract.setIndices(objectIndices);
  extract.filter(*objs);


}

void diffOfNormals(){

  double scale1 = 0.02;
  double scale2 = 0.05;
  double threshold = 0.03;
  double segradius = 0.05;
    // Create a search tree, use KDTreee for non-organized data.
  search::Search<PointT>::Ptr tree;
  if (source_cloud->isOrganized ())
  {
    tree.reset (new search::OrganizedNeighbor<PointT> ());
  }
  else
  {
    tree.reset (new search::KdTree<PointT> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (source_cloud);

  // Compute normals using both small and large scales at each point
  NormalEstimationOMP<PointT, PointNormal> ne;
  ne.setInputCloud (source_cloud);
  ne.setSearchMethod (tree);
  ne.setViewPoint (numeric_limits<float>::max (), numeric_limits<float>::max (), numeric_limits<float>::max ());

  // calculate normals with the small scale
  PointCloud<PointNormal>::Ptr normals_small_scale (new PointCloud<PointNormal>);
  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  PointCloud<PointNormal>::Ptr normals_large_scale (new PointCloud<PointNormal>);
  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new PointCloud<PointNormal>);
  copyPointCloud<PointT, PointNormal>(*source_cloud, *doncloud);

  // Create DoN operator
  DifferenceOfNormalsEstimation<PointT, PointNormal, PointNormal> don;
  don.setInputCloud(source_cloud);
  don.setNormalScaleLarge(normals_large_scale);
  don.setNormalScaleSmall(normals_small_scale);

  if (!don.initCompute ())
  {
    cerr << "Error: Could not intialize DoN feature operator" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);


  // Save DoN features
  // PCDWriter writer;
  // writer.write<PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  // cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  ConditionOr<PointNormal>::Ptr range_cond (new ConditionOr<PointNormal> ());
  range_cond->addComparison (FieldComparison<PointNormal>::ConstPtr (
                               new FieldComparison<PointNormal> ("curvature", ComparisonOps::GT, threshold)) );
  // Build the filter
  ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  PointCloud<PointNormal>::Ptr doncloud_filtered (new PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);
  doncloud = doncloud_filtered;

  // Save filtered output
  // cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << endl;

  // writer.write<PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  // cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  search::KdTree<PointNormal>::Ptr segtree (new search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  int j = 0;
  vector< PointCloud<PointNormal>::Ptr> don_clusters;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    PointCloud<PointNormal>::Ptr cloud_cluster_don (new PointCloud<PointNormal>);
    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    //Save cluster
    don_clusters.push_back(cloud_cluster_don);

    // cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << endl;
    // stringstream ss;
    // ss << "cluster_" << j << ".pcd";
    // writer.write<PointNormal> (ss.str (), *cloud_cluster_don, false);
  }

  clusters = don_clusters;
}



};