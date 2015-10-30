#include "plane_estimator.cpp"
#include "matcher.cpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

using namespace std;
using namespace pcl;

typedef PointXYZRGBA Point;

int main(int argc, char** argv){

	//Define variables
	PointCloud<Point>::Ptr source_cloud(new PointCloud<Point>);
	PointCloud<Point>::Ptr model1(new PointCloud<Point>);
	PointCloud<Point>::Ptr model2(new PointCloud<Point>);
	vector<PointCloud<Point>::Ptr> models;

	string scene = "scene.pcd";
	io::loadPCDFile (scene, *source_cloud);

	//Load models
	string nmodel1 = "EM-01_aid_tray_inliers.pcd";
	string nmodel2 = "AX-01b_bearing_box_inliers.pcd";
	io::loadPCDFile(nmodel1, *model1);
	io::loadPCDFile(nmodel2, *model2);
	models.push_back(model1);
	models.push_back(model2);


	//Filter out distant points
	PassThrough<Point> pass;
	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.0,1.0);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0.0,1.0);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0,1.0);
	pass.filter(*source_cloud);


	//Subtract plans from scene
	PlaneEstimator est(source_cloud);
	est.subPlanes();

	//Extract clusters
	est.diffOfNormals();
	vector<PointCloud<PointNormal>::Ptr> clusters = est.getClusters();
	PointCloud<PointNormal>::Ptr cloud_cluster_don;

	//Compute Features
	Matcher match(clusters, models);
	match.computeClustersFPFH();
	match.computeModsFPFH();
	match.computeHistograms();


	// PointCloud<PointXYZ>::Ptr object(new PointCloud<PointXYZ>);
	// for(vector<PointCloud<PointNormal>::Ptr>::const_iterator it = clusters.begin(); it != clusters.end(); ++it){
	// 	PointCloud<PointNormal>::Ptr cluster = *it;
	// 	object->points.resize(cluster->points.size());
	// 	for(int i = 0; i < cluster->points.size(); i++){
	// 		object->points[i].x = cluster->points[i].x;
	// 		object->points[i].y = cluster->points[i].y;
	// 		object->points[i].z = cluster->points[i].z;
	// 	}

	// 	writeHist(object);
	// }

	// run();


}