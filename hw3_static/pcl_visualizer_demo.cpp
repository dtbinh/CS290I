

#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFile 
//#include <pcl/visualization/pcl_visualizer.h>



boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
   pcl::PointCloud<pcl::PointXYZ>::ConstPtr norm_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, float x, float y, float z)
{
  cout << "load bunny" << endl;
  pcl::PolygonMesh mesh; 
  pcl::io::loadPolygonFile("../bunny.obj",mesh); 

  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);

  Eigen::Vector3f norm;
  norm[0] = normals->at(0).normal[0];
  norm[1] = normals->at(0).normal[1];
  norm[2] = normals->at(0).normal[2];

  Eigen::Vector3f first;
  first[0] = 0;
  first[1] = 1;
  first[2] = 0;

 // float angle = 1.57079633;
 // float c = cos(angle);
  float c = first.dot(norm);
  cout << "c = " << c << endl;
  Eigen::Vector3f cross = first.cross(norm);
//  Eigen::Vector3f cross = first;
  float s = sin(acos(c));
  cout << "s = " << s << endl;
  float t = 1.0 - c;
  cout << "t = " << t << endl;

  Eigen::Matrix4f rotate_1 = Eigen::Matrix4f::Identity();
  rotate_1(0,0) = c + cross[0]*cross[0]*t;
  rotate_1(1,1) = c + cross[1]*cross[1]*t;
  rotate_1(2,2) = c + cross[2]*cross[2]*t;
  float tmp1 = cross[0]*cross[1]*t;
  float tmp2 = cross[2]*s;
  rotate_1(1,0) = tmp1+tmp2;
  rotate_1(0,1) = tmp1-tmp2;
  tmp1 = cross[0]*cross[2]*t;
  tmp2 = cross[1]*s;
  rotate_1(2,0) = tmp1-tmp2;
  rotate_1(0,2) = tmp1+tmp2;
  tmp1 = cross[1]*cross[2]*t;
  tmp2 = cross[0]*s;
  rotate_1(2,1) = tmp1+tmp2;
  rotate_1(1,2) = tmp1-tmp2;
//  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//  transform_1 (0,3) = x;
//  transform_1 (1,3) = y;
//  transform_1 (2,3) = z;
  rotate_1 (0,3) = x;
  rotate_1 (1,3) = y;
  rotate_1 (2,3) = z;

  cout << rotate_1 << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (mesh_cloud, *transformed_cloud, rotate_1);

  pcl::PCLPointCloud2 pcl_pc;
  pcl::toPCLPointCloud2(*transformed_cloud, pcl_pc);
  
  mesh.cloud = pcl_pc;
 // for (int i = 0; i < mesh.cloud.points.size(); i++){
  //  cout << mesh.polygons[1].vertices[i] <<endl;
 //   mesh.vertices.at(i).y += y;
 //   mesh.vertices.at(i).z += z;
 // }

  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (norm_cloud, normals, 10, 0.05, "normals");
  viewer->addPolygonMesh(mesh, "polygon");

//  viewer->addSphere (point, 0.2, 0.5, 0.5, 0.0, "sphere");
//  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Genarating example point clouds.\n\n";

//  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../inputCloud0.pcd", *temp_cloud_ptr) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../table_scene_lms400.pcd", *temp_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
 

  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
} else {
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (temp_cloud_ptr);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*basic_cloud_ptr);
}
     pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.01);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
     
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (basic_cloud_ptr);
      seg.segment (*inliers, *coefficients);

      // Extract the inliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
      extract.setInputCloud (basic_cloud_ptr);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  cout << "normals1" << endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_p);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);
  cout << "normals2" << endl;
  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);
  cout << "normals done" << endl;

 
     Eigen::Vector4f temp_centroid;
     //find centroid of data
     cout << "your" << endl;
     pcl::compute3DCentroid (*cloud_p, temp_centroid);
     cout << "face" << endl;
     //save point
     cout << temp_centroid[0] << " " << temp_centroid[1] << " " << temp_centroid[2] << endl;
      float x = temp_centroid[0];
      float y = temp_centroid[1];
      float z = temp_centroid[2];
 
     //find normal at that point
     //create object at that point 

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = normalsVis(cloud_p, basic_cloud_ptr, cloud_normals2, x, y, z);

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}


