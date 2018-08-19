#include "jni_hellopcl.h"

#include <cstdio>
#include <android/log.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <jni.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "hello-ndk", __VA_ARGS__))


using namespace boost::filesystem;
using namespace pcl;

JNIEXPORT jint JNICALL Java_tw_com_hellopcl_MainActivity_boostMkDir
(JNIEnv * env, jobject)
{
	/*std::string hello( "hello world!" );
	
	sregex rex = sregex::compile( "(\\w+) (\\w+)!" );
	smatch what;
	
	if( regex_match( hello, what, rex ) )
	{
		std::cout << what[0] << '\n'; // whole match
		std::cout << what[1] << '\n'; // first capture
		std::cout << what[2] << '\n'; // second capture
	} */
	
	
	printf("Hello from NDK using PCL\n");
	LOGI("Hello from NDK using PCL");
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
  cloud->width  = 50;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
	
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);		
  }
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	for (int i = 0; i < 100; ++i)
	{
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.nearestKSearch (0, 3, indices, distances);
		printf ("indices size: %d\n", indices.size ());
		LOGI ("indices size %d\n", indices.size ());
	}
	
	LOGI ("Doing stuff with PCL on Android");
	
	
	LOGI ("Doing stuff with PCL on Android cloud size: %d %d %d\n", cloud->width, cloud->height, pcl::io::savePCDFileASCII ("/sdcard/PCL/output.pcd", *cloud));
	pcl::io::savePLYFileASCII ("/sdcard/PCL/output.ply", *cloud);
	
	std::ofstream file ("/sdcard/PCL/caca.caca");
	file << "caca caca caca\n";
	file.close ();
	
	
	return (cloud->width);
}


JNIEXPORT void JNICALL Java_tw_com_hellopcl_MainActivity_smoothPointCloud
(JNIEnv * env, jobject)
{
	LOGI ("Doing stuff with PCL on Android Java_tw_com_hellopcl_MainActivity_smoothPointCloud() ");

		
	// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPLYFile ("/sdcard/PCL/pc2.ply", cloud_blob);
  LOGI("Input point cloud loaded  %d %d", cloud_blob.width, cloud_blob.height);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  LOGI("Input point cloud loaded  %d %d", cloud->width, cloud->height);



  // Outlier Removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new PointCloud<pcl::PointXYZ>);
  
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (500);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);


  LOGI("Input point cloud filtered  %d %d", cloud_filtered->width, cloud_filtered->height);
  // Upsampling
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setInputCloud (cloud_filtered);
  mls.setSearchRadius (0.04);
  mls.setPolynomialFit (true);
  mls.setPolynomialOrder (2);
  mls.setUpsamplingMethod (MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE); 
  mls.setUpsamplingRadius (0.005);
  mls.setUpsamplingStepSize (0.004);
  mls.process (*cloud_smoothed); 
  
  LOGI("Input point cloud upsampled  %d %d", cloud_smoothed->width, cloud_smoothed->height);
  pcl::io::savePLYFileASCII("/sdcard/PCL/upsampled.ply", *cloud_smoothed);


  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_smoothed);
  n.setInputCloud (cloud_smoothed);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_smoothed, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.08);

  // Set typical values for the parameters
  gp3.setMu (5.0);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/3); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::saveOBJFile ("/sdcard/PCL/pc2Mesh.obj", triangles); 
LOGI ("Doing stuff with PCL on Android Java_tw_com_hellopcl_MainActivity_smoothPointCloud() DONEEEEEEEE ");
}
