#pragma once
#define production
#define VTK_LEGACY_SILENT
#include <iostream>
#include <string>
#include <vector>
#include <ctime>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;
bool fin = true;
PointCloudT::Ptr detectDifferences(PointCloudT::Ptr cloudA, PointCloudT::Ptr cloudB)
{
	srand((unsigned int)time(NULL));

	// Octree resolution - side length of octree voxels
	float resolution = 5.0f;

	// Instantiate octree-based point cloud change detection class
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

	// Add points from cloudA to octree
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();

	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
	octree.switchBuffers();

	// Add points from cloudB to octree
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	PointCloudT::Ptr cloudC(new PointCloudT);
	// Output points
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
std:cout << newPointIdxVector.size();
	for (std::size_t i = 0; i < newPointIdxVector.size(); ++i) {
		cloudC->points.push_back(cloudB->points[newPointIdxVector[i]]);
	}
	return cloudC;

}
void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
  if (event.getKeySym() == "e" && event.keyDown()) {
	  fin = true;
	  std::cout << "bye" << std::endl;
  }
}

int
main (int argc,
      char* argv[])
{
	int noiter = 1;
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_basetr(new PointCloudT);  // Transformed point cloud at begining
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  // Checking program arguments
  if (argc < 3)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide one ply file.\n");
    return (-1);
  }

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 4)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[4]);
  }
  if (argc > 5)
  {
	  // If the user passed the number of comited iterations as an argument
	  noiter = atoi(argv[5]);
  }

  // loading files
  pcl::console::TicToc time;
  time.tic ();
  if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0)
  {
	  PCL_ERROR("Error loading cloud %s.\n", argv[1]);
	  return (-1);
  }
  std::cout << "\nLoaded base file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;
  time.tic();
  if (pcl::io::loadPLYFile(argv[2], *cloud_tr) < 0)
  {
	  PCL_ERROR("Error loading cloud %s.\n", argv[2]);
	  return (-1);
  }
  std::cout << "\nLoaded referecne file " << argv[2] << " (" << cloud_tr->size() << " points) in " << time.toc() << " ms\n" << std::endl; time.tic();
  if (pcl::io::loadPLYFile(argv[3], *cloud_basetr) < 0)
  {
	  PCL_ERROR("Error loading cloud %s.\n", argv[3]);
	  return (-1);
  }
  std::cout << "\nLoaded file to modify" << argv[3] << " (" << cloud_tr->size() << " points) in " << time.toc() << " ms\n" << std::endl;
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  // We backup cloud_icp into cloud_tr for later use
  *cloud_icp = *cloud_tr;

  //if 0 only octaTree is applied
  if (iterations != 0) {

  // The Iterative Closest Point algorithm
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  icp.align (*cloud_icp);
  icp.setMaximumIterations (1);
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two vertically separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_basetr, 20, 180, 20);
  viewer.addPointCloud (cloud_basetr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);



  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud\nBlue: Where diferences are detected", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << noiter;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.loadCameraParameters("camparams.cam");
  viewer.setSize(1920, 1080);// Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
 
  // Display the visualiser
  while (!viewer.wasStopped() && fin)//
  {
	  viewer.spinOnce(10000);
	viewer.saveScreenshot("D:\\screens\\" + std::to_string(noiter++) + ".png");
    // The user pressed "space" :
    if (!next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << noiter << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << noiter;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
	
  }

  // difference points cloud is blue
  fin = true;
 PointCloudT::Ptr clouddiff = detectDifferences(cloud_in, cloud_icp);
 pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_diff_color_h(clouddiff, 20, 20, 180);
 viewer.addPointCloud(clouddiff, cloud_diff_color_h, "cloud_diff_v2", v2);
  pcl::io::savePLYFile("test3.ply", *cloud_icp); 
  while (!viewer.wasStopped() && fin)//
  {
	  viewer.spinOnce(100000);
  }
  viewer.spinOnce(100000);

  viewer.saveCameraParameters("camparams.cam");
  viewer.saveScreenshot("D:\\screens\\Afin.png");
  }
  else {
  // Visualization
  pcl::visualization::PCLVisualizer viewer("ICP demo");
  // Create two vertically separated viewports
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
	  (int)255 * txt_gray_lvl);
  viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_basetr, 20, 180, 20);
  viewer.addPointCloud(cloud_basetr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
  viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud\nBlue: Where diferences are detected", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << noiter;
  std::string iterations_cnt = "ICP iterations = " + ss.str();
  viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1920, 1080);// Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
  viewer.loadCameraParameters("camparams.cam");
  PointCloudT::Ptr clouddiff = detectDifferences(cloud_in, cloud_icp);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_diff_color_h(clouddiff, 20, 20, 180);
  viewer.addPointCloud(clouddiff, cloud_diff_color_h, "cloud_diff_v2", v2);
  viewer.spin();
  viewer.saveScreenshot("D:\\screens\\" + std::to_string(noiter++) + ".png");
}
  return (0);
}

//"E:\Systemowe\Documents\CODE\vs17pg\vs17pg\vs17pg\Out_DemMorenaPixelClose-20200206T093424Z-001\Out_DemMorenaPixelClose\clearedScaledToCm.ply"