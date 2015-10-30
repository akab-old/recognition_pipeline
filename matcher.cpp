#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/histogram_visualizer.h>
#include <string>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>


using namespace std;
using namespace pcl;

typedef PointXYZRGBA PointT;
typedef visualization::PointCloudColorHandlerCustom<PointXYZ> ColorHandlerT;

class Matcher{

public:

	vector<PointCloud<VFHSignature308>::Ptr> clusters_histogram;
	vector<PointCloud<VFHSignature308>::Ptr> models_histogram;
	vector<PointCloud<FPFHSignature33>::Ptr> clusters_signatures;
	vector<PointCloud<FPFHSignature33>::Ptr> models_signatures;
	vector<PointCloud<PointNormal>::Ptr> clusters;
	vector<PointCloud<PointT>::Ptr> models;

	Matcher(){
	}

	Matcher(vector<PointCloud<PointNormal>::Ptr> objs, vector<PointCloud<PointT>::Ptr> mods){
		clusters = objs;
		clusters_signatures.resize(clusters.size());
		models = mods;
		models_signatures.resize(mods.size());

		cout << "MATCH: clusters initialized with " << clusters.size() << " elements" << endl;
		cout << endl;
		cout << "MATCH: models initialized with " << models.size() << " elements" << endl;
		cout << endl;
	}

	~Matcher(){

	}

	void computeHistograms(){

	cout << "MATCH: Computing Clusters histograms " << endl;
	cout << endl;
	int i = 1;
	for(vector<PointCloud<PointNormal>::Ptr>::iterator it = clusters.begin(); it != clusters.end(); ++it, ++i){
		PointCloud<PointNormal>::Ptr cloud = *it;

		// Compute normals
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
		search::KdTree<PointNormal>::Ptr pKdTree(new search::KdTree<PointNormal>());

		NormalEstimation<PointNormal, Normal> ne;
		pKdTree->setInputCloud(cloud);
		ne.setInputCloud(cloud);
		ne.setSearchMethod(pKdTree);
		ne.setKSearch(20);
		ne.compute(*normals);

		// Create the VFH estimation class, and pass the input dataset+normals to it
		VFHEstimation<PointNormal, Normal, VFHSignature308> vfh;
		vfh.setInputCloud (cloud);
		vfh.setInputNormals(normals);
		vfh.setSearchMethod(pKdTree);

		// Output datasets
	  	PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

		// Compute the features
		vfh.compute (*vfhs);

		clusters_histogram.push_back(vfhs);

		if(vfhs->points.size() == 1) cout << "VFH computed for the " << i << "th cluster; " << endl;


	}

	cout << "MATCH: Computing models histograms " << endl;
	cout << endl;
	i = 1;
	for(vector<PointCloud<PointT>::Ptr>::iterator it = models.begin(); it != models.end(); ++it, ++i){
		PointCloud<PointT>::Ptr cloud = *it;

		// Compute normals
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
		search::KdTree<PointT>::Ptr pKdTree(new search::KdTree<PointT>());

		NormalEstimation<PointT, Normal> ne;
		pKdTree->setInputCloud(cloud);
		ne.setInputCloud(cloud);
		ne.setSearchMethod(pKdTree);
		ne.setKSearch(20);
		ne.compute(*normals);

		// Create the VFH estimation class, and pass the input dataset+normals to it
		VFHEstimation<PointT, Normal, VFHSignature308> vfh;
		vfh.setInputCloud (cloud);
		vfh.setInputNormals(normals);
		vfh.setSearchMethod(pKdTree);

		// Output datasets
	  	PointCloud<VFHSignature308>::Ptr vfhs (new PointCloud<VFHSignature308> ());

		// Compute the features
		vfh.compute (*vfhs);

		models_histogram.push_back(vfhs);

		if(vfhs->points.size() == 1) cout << "VFH computed for the " << i << "th model; " << endl;

	}


	// cout << "Write VFH histogram to a file... " << endl;
	// float num = rand() % 100;
	// string id = boost::lexical_cast<string>(num) + ".pcd";
	// io::savePCDFileASCII (id, *vfhs);
	// cout << "...DONE " << endl;

	}

	PointCloud<FPFHSignature33>::Ptr computeFPFH(PointCloud<PointNormal>::Ptr cluster){

		PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
		object->points.resize(cluster->points.size());
		for(int i = 0; i < cluster->points.size(); i++){
			object->points[i].x = cluster->points[i].x;
			object->points[i].y = cluster->points[i].y;
			object->points[i].z = cluster->points[i].z;
		}

		//Compute normals
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
		search::KdTree<PointT>::Ptr pKdTree(new search::KdTree<PointT>());

		NormalEstimation<PointT, Normal> ne;
		pKdTree->setInputCloud(object);
		ne.setInputCloud(object);
		ne.setSearchMethod(pKdTree);
		ne.setKSearch(20);
		ne.compute(*normals);

	    FPFHEstimation<PointT, Normal, FPFHSignature33> fpfh;
	    fpfh.setInputCloud(object);
	    fpfh.setInputNormals(normals);
	    fpfh.setSearchMethod(pKdTree);

	    //Output datasets
	    PointCloud<FPFHSignature33>::Ptr fpfhs(new PointCloud<FPFHSignature33>());
	    fpfh.setRadiusSearch(0.05); //5 cm (must be > radius_normal)
	    fpfh.compute(*fpfhs);

	    return fpfhs;

	    // float num = 1024 * rand () / (RAND_MAX + 1.0f);
	    // string id = boost::lexical_cast<string>(num) + ".pcd";
	    //   io::savePCDFileASCII (id, *fpfhs);

	}

		PointCloud<FPFHSignature33>::Ptr computeFPFH(PointCloud<PointXYZ>::Ptr object){


		//Compute normals
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
		search::KdTree<PointXYZ>::Ptr pKdTree(new search::KdTree<PointXYZ>());

		NormalEstimation<PointXYZ, Normal> ne;
		pKdTree->setInputCloud(object);
		ne.setInputCloud(object);
		ne.setSearchMethod(pKdTree);
		ne.setKSearch(20);
		ne.compute(*normals);

	    FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
	    fpfh.setInputCloud(object);
	    fpfh.setInputNormals(normals);
	    fpfh.setSearchMethod(pKdTree);

	    //Output datasets
	    PointCloud<FPFHSignature33>::Ptr fpfhs(new PointCloud<FPFHSignature33>());
	    fpfh.setRadiusSearch(0.05); //5 cm (must be > radius_normal)
	    fpfh.compute(*fpfhs);

	    return fpfhs;

	    // float num = 1024 * rand () / (RAND_MAX + 1.0f);
	    // string id = boost::lexical_cast<string>(num) + ".pcd";
	    //   io::savePCDFileASCII (id, *fpfhs);

	}

	PointCloud<FPFHSignature33>::Ptr computeFPFH(PointCloud<PointT>::Ptr object){


		//Compute normals
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
		search::KdTree<PointT>::Ptr pKdTree(new search::KdTree<PointT>());

		NormalEstimation<PointT, Normal> ne;
		pKdTree->setInputCloud(object);
		ne.setInputCloud(object);
		ne.setSearchMethod(pKdTree);
		ne.setKSearch(20);
		ne.compute(*normals);

	    FPFHEstimation<PointT, Normal, FPFHSignature33> fpfh;
	    fpfh.setInputCloud(object);
	    fpfh.setInputNormals(normals);
	    fpfh.setSearchMethod(pKdTree);

	    //Output datasets
	    PointCloud<FPFHSignature33>::Ptr fpfhs(new PointCloud<FPFHSignature33>());
	    fpfh.setRadiusSearch(0.05); //5 cm (must be > radius_normal)
	    fpfh.compute(*fpfhs);

	    return fpfhs;

	    // float num = 1024 * rand () / (RAND_MAX + 1.0f);
	    // string id = boost::lexical_cast<string>(num) + ".pcd";
	    //   io::savePCDFileASCII (id, *fpfhs);

	}

	void computeClustersFPFH(){

		for(vector<PointCloud<PointNormal>::Ptr>::const_iterator it = clusters.begin(); it != clusters.end(); ++it){
			PointCloud<PointNormal>::Ptr obj = *it;
			PointCloud<FPFHSignature33>::Ptr obj_sign = computeFPFH(obj);
			// cout << "obj_sign size " << obj_sign->points.size() << endl;
			clusters_signatures.push_back(obj_sign);
		}

		cout << "MATCH: clusters_signatures size " << clusters_signatures.size() << endl;

	}

	void computeModsFPFH(){

		for(vector<PointCloud<PointT>::Ptr>::const_iterator it = models.begin(); it != models.end(); ++it){
			PointCloud<PointT>::Ptr model = *it;
			PointCloud<FPFHSignature33>::Ptr mod_sign = computeFPFH(model);
			// cout << "mod_sign size " << mod_sign->points.size() << endl;
			models_signatures.push_back(mod_sign);
		}

		cout << "MATCH: models_signatures size " << models_signatures.size() << endl;

	}

};