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


///Mutex Class
class Mutex {
public:
  Mutex() {
    pthread_mutex_init( &m_mutex, NULL );
  }
  void lock() {
    pthread_mutex_lock( &m_mutex );
  }
  void unlock() {
    pthread_mutex_unlock( &m_mutex );
  }

  class ScopedLock
  {
    Mutex & _mutex;
  public:
    ScopedLock(Mutex & mutex)
      : _mutex(mutex)
    {
      _mutex.lock();
    }
    ~ScopedLock()
    {
      _mutex.unlock();
    }
  };
private:
  pthread_mutex_t m_mutex;
};




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
    Mutex::ScopedLock lock(m_rgb_mutex);
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
  };
  // Do not call directly even in child
  void DepthCallback(void* _depth, uint32_t timestamp) {
    Mutex::ScopedLock lock(m_depth_mutex);
    depth.clear();
    uint16_t* call_depth = static_cast<uint16_t*>(_depth);
    for (size_t i = 0; i < 640*480 ; i++) {
      depth.push_back(call_depth[i]);
    }
    m_new_depth_frame = true;
  }
  bool getRGB(std::vector<uint8_t> &buffer) {
    Mutex::ScopedLock lock(m_rgb_mutex);
    if (!m_new_rgb_frame)
      return false;
    buffer.swap(m_buffer_video);
    m_new_rgb_frame = false;
    return true;
  }

  bool getDepth(std::vector<uint16_t> &buffer) {
    Mutex::ScopedLock lock(m_depth_mutex);
    if (!m_new_depth_frame)
      return false;
    buffer.swap(depth);
    m_new_depth_frame = false;
    return true;
  }

private:
  std::vector<uint16_t> depth;
  std::vector<uint8_t> m_buffer_video;
  Mutex m_rgb_mutex;
  Mutex m_depth_mutex;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
};


FreenectFix::Freenect freenect;
MyFreenectDevice* device;

//PCL

// --------------
// -----Main-----
// --------------
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
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
	
  //Voxelizer Setup
	
  printf("Create the devices.\n");
  device = &freenect.createDevice<MyFreenectDevice>(0);
  //devicetwo = &freenect.createDevice<MyFreenectDevice>(1);
  device->startVideo();
  device->startDepth();
  boost::this_thread::sleep (boost::posix_time::seconds (1));
  //devicetwo->startVideo();
  //devicetwo->startDepth();
  //boost::this_thread::sleep (boost::posix_time::seconds (1));
  //Grab until clean returns
  int DepthCount = 0;
  while (DepthCount == 0) {
    device->updateState();
    device->getDepth(mdepth);
    device->getRGB(mrgb);
    for (size_t i = 0;i < 480*640;i++)
      DepthCount+=mdepth[i];
  }

  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setLeafSize (10.0f, 10.0f, 10.0f);

  // Some region growing stuff
  /*
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (6);
  reg.setMinClusterSize (600);
  */

  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setKSearch (50);


  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (500);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  //reg.setIndices (indices);
  reg.setSmoothnessThreshold (1.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);


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
  printf("Start the main loop.\n");
  while (!viewer->wasStopped ())
    {
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
	      cloud->points[i].x  = x;//1000.0;
	      cloud->points[i].y  = y;//1000.0;
	      cloud->points[i].z = iRealDepth;//1000.0;
	      cloud->points[i].r = mrgb[i*3];
	      cloud->points[i].g = mrgb[(i*3)+1];
	      cloud->points[i].b = mrgb[(i*3)+2];  				
	    }
	}

     
      vox.setInputCloud (cloud);
      vox.filter (*cloud_filtered);
      
      std::vector <pcl::PointIndices> clusters;
      normal_estimator.setInputCloud (cloud_filtered);
      normal_estimator.compute (*normals);
      reg.setInputCloud (cloud_filtered);
      reg.setInputNormals (normals);
      reg.extract (clusters);
      cout << clusters.size() << std::endl;

      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      
      
      if(!viewer->updatePointCloud (colored_cloud, "Kinect Cloud")){
        viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud, "Kinect Cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Kinect Cloud");
      }
      viewer->spinOnce ();
  }
  device->stopVideo();
  device->stopDepth();
  //devicetwo->stopVideo();
  //devicetwo->stopDepth();
  return 0;
}
