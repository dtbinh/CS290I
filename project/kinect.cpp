/*
 * This file contains code that is part of the OpenKinect Project.
 * http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 * Additional code is copyright (c) 2011 Jeff Kramer (jeffkramr@gmail.com).
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 * 
 * 
 */

#include <iostream>
#include "freenect_fix.hpp"
#include <libfreenect/libfreenect_registration.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/console/parse.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include "boost/lexical_cast.hpp"
#include "pcl/filters/voxel_grid.h"
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include <boost/thread/mutex.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <sstream>

int threshold = 500;

///Kinect Hardware Connection Class
/* thanks to Yoda---- from IRC */
class MyFreenectDevice : public FreenectFix::FreenectDevice {
public:
  MyFreenectDevice(freenect_context *_ctx, int _index)
    : FreenectFix::FreenectDevice(_ctx, _index), depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes),m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_new_rgb_frame(false), m_new_depth_frame(false)
  {
    
  }
  //~MyFreenectDevice(){}
  // Do not call directly even in child
  void VideoCallback(void* _rgb, uint32_t timestamp) {
    boost::mutex::scoped_lock lock(m_rgb_mutex);
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
  };
  // Do not call directly even in child
  void DepthCallback(void* _depth, uint32_t timestamp) {
    boost::mutex::scoped_lock lock(m_depth_mutex);
    depth.clear();
    uint16_t* call_depth = static_cast<uint16_t*>(_depth);
    for (size_t i = 0; i < 640*480 ; i++) {
      depth.push_back(call_depth[i]);
    }
    m_new_depth_frame = true;
  }
  bool getRGB(std::vector<uint8_t> &buffer) {
    boost::mutex::scoped_lock lock(m_rgb_mutex);
    if (!m_new_rgb_frame)
      return false;
    buffer.swap(m_buffer_video);
    m_new_rgb_frame = false;
    return true;
  }

  bool getDepth(std::vector<uint16_t> &buffer) {
    boost::mutex::scoped_lock lock(m_depth_mutex);
    if (!m_new_depth_frame)
      return false;
    buffer.swap(depth);
    m_new_depth_frame = false;
    return true;
  }

private:
  std::vector<uint16_t> depth;
  std::vector<uint8_t> m_buffer_video;
  boost::mutex m_depth_mutex;
  boost::mutex m_rgb_mutex;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
};


FreenectFix::Freenect freenect;
MyFreenectDevice* device;

//PCL

// --------------
// -----Main-----
// --------------
pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud;

boost::shared_ptr<std::vector<pcl::PolygonMesh::Ptr> > vis_meshes;

void visualizerThread()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  std::vector<std::string> meshnames;

  while (!viewer->wasStopped ())
  {  

    viewer->spinOnce ();
    
    if(vis_cloud){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
      vis_cloud.swap(cloud);

      if(!viewer->updatePointCloud (cloud, "Kinect Cloud")){
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "Kinect Cloud");
	viewer->setPointCloudRenderingProperties 
	  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Kinect Cloud");
      }
    }
    if(vis_meshes){
      boost::shared_ptr<std::vector<pcl::PolygonMesh::Ptr> > meshes;
      vis_meshes.swap(meshes);
      int i = 0;
      for(std::vector<std::string>::iterator it = meshnames.begin();
          it != meshnames.end(); it++)
         viewer->removePolygonMesh(*it);
      meshnames.clear();
      for(std::vector<pcl::PolygonMesh::Ptr>::iterator it = meshes->begin();
          it != meshes->end();
          it++){
       //viewer->removePolygonMesh("polygon"+i); 
       std::stringstream ss;
       ss << "polygon"<< i++;
       meshnames.push_back(ss.str());
       viewer->addPolygonMesh(**it, ss.str());
       viewer->setPointCloudRenderingProperties 
	  (pcl::visualization::PCL_VISUALIZER_COLOR, (rand()%256)/256.0f,(rand()%256)/256.0f,(rand()%256)/256.0f, ss.str());
      }
    }
  }
}

int main (int argc, char** argv)
{
  //More Kinect Setup
  static std::vector<uint16_t> mdepth(640*480);
  static std::vector<uint8_t> mrgb(640*480*4);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // Fill in the cloud data
  cloud->width    = 640;
  cloud->height   = 480;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  // Create and setup the viewer
  printf("Create the viewer.\n");

  //Voxelizer Setup
	
  printf("Create the devices.\n");
  device = &freenect.createDevice<MyFreenectDevice>(0);
  //devicetwo = &freenect.createDevice<MyFreenectDevice>(1);
  device->startVideo();
  device->startDepth();

  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setLeafSize (30.0f, 30.0f, 30.0f);

  // Some region growing stuff
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (6);
  reg.setMinClusterSize (600);



  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setKSearch (8);

/*
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (8);
  reg.setSmoothnessThreshold (30 / 180.0 * M_PI);
  reg.setCurvatureThreshold (0.001);
*/
  pcl::SACSegmentation<pcl::PointXYZRGB> seg_plane;
  seg_plane.setOptimizeCoefficients (true);
  seg_plane.setModelType (pcl::SACMODEL_PLANE);
  seg_plane.setMethodType (pcl::SAC_RANSAC);
  seg_plane.setDistanceThreshold (20);
  seg_plane.setMaxIterations (100);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg_plane2;
  seg_plane2.setOptimizeCoefficients (true);
  seg_plane2.setModelType (pcl::SACMODEL_PLANE);
  seg_plane2.setMethodType (pcl::SAC_RANSAC);
  seg_plane2.setDistanceThreshold (20);
  seg_plane2.setMaxIterations (100);
  
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg_cylinder;
  seg_cylinder.setModelType (pcl::SACMODEL_CYLINDER);
  seg_cylinder.setMethodType (pcl::SAC_RANSAC);
  seg_cylinder.setNormalDistanceWeight (0.1);
  seg_cylinder.setMaxIterations (100);
  seg_cylinder.setDistanceThreshold (0.05);
  seg_cylinder.setRadiusLimits (0, 0.1);
  
  boost::thread thrd1(
    boost::bind(&visualizerThread));
  //--------------------
  // -----Main loop-----
  //--------------------
  double x = 0;
  double y = 0;
  double tx = 0;
  double ty = 0;
  int iRealDepth = 0;
  int iTDepth = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  
  pcl::ConvexHull<pcl::PointXYZRGB> cHull;
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voronoi_centers1 (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract_neg;
  extract_neg.setNegative(true);
  
  printf("Start the main loop.\n");
  while (1){
      device->updateState();
      device->getDepth(mdepth);
      device->getRGB(mrgb);
		
      size_t i = 0;
      for (size_t v=0 ; v<480 ; v++)
    	{
	  for ( size_t u=0 ; u<640 ; u++, i++)
	    {
	      iRealDepth = mdepth[i];
	      freenect_camera_to_world(device->getDevice(), u, v, iRealDepth, &x, &y);
	      cloud->points[i].x  = x;
	      cloud->points[i].y  = y;
	      cloud->points[i].z = iRealDepth;
	      cloud->points[i].r = mrgb[i*3];
	      cloud->points[i].g = mrgb[(i*3)+1];
	      cloud->points[i].b = mrgb[(i*3)+2];  				
	    }
	}
/*
      pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
      fbf.setSigmaS (10.0f);
      fbf.setSigmaR (10.0f);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bfiltered (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
      fbf.setInputCloud (cloud);
      fbf.applyFilter (*cloud_bfiltered);   
  */
      vox.setInputCloud (cloud);
      vox.filter (*cloud_filtered);
      
      std::vector <pcl::PointIndices> clusters;
      normal_estimator.setInputCloud (cloud_filtered);
      normal_estimator.compute (*normals);
      reg.setInputCloud (cloud_filtered);
      reg.setInputNormals (normals);
//      reg.extract (clusters);


      seg_cylinder.setInputCloud(cloud_filtered);
      seg_cylinder.setInputNormals (normals);

      proj.setInputCloud (cloud_filtered);

      seg_plane.setInputCloud(cloud_filtered);



      i = 0;

      boost::shared_ptr<std::vector<pcl::PolygonMesh::Ptr> > 
         meshes(new std::vector<pcl::PolygonMesh::Ptr>());

//      for(std::vector<pcl::PointIndices>::iterator
//	    cluster = clusters.begin(); 
//	  cluster != clusters.end();
//	  cluster++){

	int fsize = 65536;

	
//	pcl::PointIndices::Ptr clust(new pcl::PointIndices(*cluster));

	//seg_cylinder.setIndices(clust);
//	seg_plane.setIndices(clust);

	seg_plane.segment (*inliers_plane, *coefficients_plane);
	fsize = inliers_plane->indices.size();
	extract_neg.setInputCloud(cloud_filtered);

	int j = 0;
	while(fsize > threshold){	  
	  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  	  proj.setIndices(inliers_plane);
          proj.setModelCoefficients (coefficients_plane);
          proj.filter (*cloud_proj);
          cHull.setInputCloud(cloud_proj);
          cHull.reconstruct (*mesh);
          meshes->push_back(mesh);


	  extract_neg.setIndices(inliers_plane);
	  extract_neg.filter(*outliers);

	  seg_plane2.setInputCloud(outliers);
	  seg_plane2.segment (*inliers_plane, *coefficients_plane);
	  
	  extract_neg.setInputCloud(outliers);

	  cout << "Cluster: "<< i << " " << j++ << "Size: " << fsize << " " << outliers->size()<< " " << inliers_plane->indices.size() << endl;
	  fsize = inliers_plane->indices.size();

	}
        i++;
  //    }
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      vis_cloud = colored_cloud;
      vis_meshes = meshes;
      //      cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  }
  thrd1.join();
  device->stopVideo();
  device->stopDepth();
  //devicetwo->stopVideo();
  //devicetwo->stopDepth();
  return 0;
}
