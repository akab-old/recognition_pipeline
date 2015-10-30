#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>

using namespace std;
using namespace pcl;

typedef PointCloud<VFHSignature308> VFHSig;

void writeHist(PointCloud<PointXYZ>::Ptr cloud){

	// Compute normals
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
	search::KdTree<PointXYZ>::Ptr pKdTree(new search::KdTree<PointXYZ>());

	NormalEstimation<PointXYZ, Normal> ne;
	pKdTree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(pKdTree);
	ne.setKSearch(20);
	ne.compute(*normals);

	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(pKdTree);

	// Output datasets
  	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

	// Compute the features
	vfh.compute (*vfhs);

	if(vfhs->points.size() == 1) cout << "VFH computed! " << endl;

	cout << "Write VFH histogram to a file... " << endl;

	float num = rand() % 100;
	string id = boost::lexical_cast<string>(num) + ".pcd";
	io::savePCDFileASCII (id, *vfhs);

	cout << "...DONE " << endl;

}

int run(){

  VFHSig::Ptr train_vfhs(new VFHSig());

  KdTree<VFHSignature308>::Ptr tree;

  string hist_name;

  //Load VFH Features
  vector<string> filenames;
  hist_name = "hist_EM-01_aid_tray.pcd";
  filenames.push_back(hist_name);
  hist_name = "hist_AX-01b_bearing_box.pcd";
  filenames.push_back(hist_name);

  for(int i = 1; i < 10; ++i){
    hist_name = "cluster_" + boost::to_string(i) + ".pcd";
    filenames.push_back(hist_name);
  }

  // for(size_t i = 0; filenames.size(); ++i){
  //   train_vfhs += *(loadPointCloud<VFHSignature308>(filenames[i], ""));
  // }

  // tree = KdTree<VFHSignature308>::Ptr(new KdTree<VFHSignature308>);
  // tree->setInputCloud(descriptors);


	return 0;
}