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


using namespace std;
using namespace pcl;

typedef PointXYZRGBA PointT;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

class Matcher{

public:

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
	}

	~Matcher(){

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
	    //   pcl::io::savePCDFileASCII (id, *fpfhs);

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
	    //   pcl::io::savePCDFileASCII (id, *fpfhs);

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
	    //   pcl::io::savePCDFileASCII (id, *fpfhs);

	}

	void computeClustersFPFH(){

		for(vector<PointCloud<PointNormal>::Ptr>::const_iterator it = clusters.begin(); it != clusters.end(); ++it){
			PointCloud<PointNormal>::Ptr obj = *it;
			PointCloud<FPFHSignature33>::Ptr obj_sign = computeFPFH(obj);
			// cout << "obj_sign size " << obj_sign->points.size() << endl;
			clusters_signatures.push_back(obj_sign);
		}

		// cout << "clusters_signatures size " << clusters_signatures.size() << endl;

	}

	void computeModsFPFH(){

		for(vector<PointCloud<PointT>::Ptr>::const_iterator it = models.begin(); it != models.end(); ++it){
			PointCloud<PointT>::Ptr model = *it;
			PointCloud<FPFHSignature33>::Ptr mod_sign = computeFPFH(model);
			// cout << "mod_sign size " << mod_sign->points.size() << endl;
			models_signatures.push_back(mod_sign);
		}

		// cout << "models_signatures size " << models_signatures.size() << endl;

	}



	void FeatureMatching(){

		SampleConsensusPrerejective<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> align;
		PointCloud<PointXYZ>::Ptr model_aligned(new PointCloud<PointXYZ>());
		// PointCloud<FPFHSignature33>::Ptr c_hist = *(clusters_signatures.begin());
		// cout << "c_hist size " << c_hist->points.size() << endl;
		// PointCloud<FPFHSignature33>::Ptr m_hist = *(models_signatures.begin());
		// cout << "m_hist size " << m_hist->points.size() << endl;

		PointCloud<PointNormal>::Ptr cnorm = clusters.at(3);
		PointCloud<PointT>::Ptr m = models.at(0);
		PointCloud<PointXYZ>::Ptr model(new PointCloud<PointXYZ>());

		copyPointCloud(*m,*model);

		PointCloud<PointXYZ>::Ptr cluster(new PointCloud<PointXYZ>);
		cluster->points.resize(cnorm->points.size());
		for(int i = 0; i < cnorm->points.size(); i++){
			cluster->points[i].x = cnorm->points[i].x;
			cluster->points[i].y = cnorm->points[i].y;
			cluster->points[i].z = cnorm->points[i].z;
		}

		PointCloud<FPFHSignature33>::Ptr c_hist = computeFPFH(cluster);
		PointCloud<FPFHSignature33>::Ptr m_hist = computeFPFH(model);

		align.setInputSource (model);
	    align.setSourceFeatures (m_hist);
	    align.setInputTarget (cluster);
	    align.setTargetFeatures (c_hist);
	    align.setMaximumIterations (10000); // Number of RANSAC iterations
	    align.setNumberOfSamples (8); // Number of points to sample for generating/prerejecting a pose
	    align.setCorrespondenceRandomness (4); // Number of nearest features to use
	    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	    const float leaf = 0.05f;
	    align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
	    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis

	    align.align (*model_aligned);

	    Eigen::Matrix4f transformation = align.getFinalTransformation ();

	    // Show alignment
	    pcl::visualization::PCLVisualizer visu("Alignment");
	    visu.addPointCloud (cluster, ColorHandlerT (cluster, 0.0, 255.0, 0.0), "scene_cluster");
	    visu.addPointCloud (model_aligned, ColorHandlerT (model_aligned, 255.0, 0.0, 0.0), "object_aligned");
	    visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object_aligned");
	    visu.spin ();




	}

};