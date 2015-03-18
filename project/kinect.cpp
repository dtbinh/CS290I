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

#include "primitives.hpp"
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
#include <GL/glut.h>

int threshold = 250;

//Global for main loop so openGL
bool GO = true;
bool data_rdy = false;
boost::mutex mtx;
boost::condition_variable condition;

//Clipping plane [near,far] 1439.65, 5454.22
//Focal point [x,y,z] -111.606, 29.9423, 875.226
//Position [x,y,z] -111.606, 29.9423, -2338.3


//global vars for camera (GL)
GLdouble eyeX = 5.0;
GLdouble eyeY = 0;
GLdouble eyeZ = 0;
GLdouble centerX = 0;
GLdouble centerY = 0;
GLdouble centerZ = -100;

//openGL shape vector
boost::shared_ptr<std::vector<boost::shared_ptr<PRIMITIVE> > > items;

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
pcl::PointCloud<pcl::Normal>::Ptr vis_normals;

boost::shared_ptr<std::vector<pcl::PolygonMesh::Ptr> > vis_meshes;
boost::shared_ptr<std::vector<pcl::ModelCoefficients::Ptr> > 
vis_spheres(new std::vector<pcl::ModelCoefficients::Ptr>());
boost::shared_ptr<std::vector<pcl::ModelCoefficients::Ptr> > 
vis_cylinders(new std::vector<pcl::ModelCoefficients::Ptr>());

void visualizerThread()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  std::vector<std::string> meshnames;
  boost::mutex::scoped_lock lock(mtx);
  while (!viewer->wasStopped ())
    {  
      while(!data_rdy)
        condition.wait(lock);
      viewer->spinOnce ();
    
      if(vis_cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	vis_cloud.swap(cloud);
	
	//	if(!viewer->updatePointCloud (cloud, "Kinect Cloud")){
	viewer->removePointCloud("cloud1");
	//viewer->removePointCloud("cloud1");
	//	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud,vis_normals,1,100);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "cloud1");
	//viewer->setPointCloudRenderingProperties 
	//  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud");
	//}
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
	  std::stringstream ss;
	  ss << "polygon"<< i++;
	  meshnames.push_back(ss.str());
	  viewer->addPolygonMesh(**it, ss.str());
	  viewer->setPointCloudRenderingProperties 
	    (pcl::visualization::PCL_VISUALIZER_COLOR, (rand()%256)/256.0f,(rand()%256)/256.0f,(rand()%256)/256.0f, ss.str());
	}
      }

      if(vis_spheres){
	boost::shared_ptr<std::vector<pcl::ModelCoefficients::Ptr> > 
	  spheres(new std::vector<pcl::ModelCoefficients::Ptr>());
	vis_spheres.swap(spheres);
	static int i = 0;
	for(std::vector<pcl::ModelCoefficients::Ptr>::iterator it = 
	      spheres->begin();
	    it != spheres->end();
	    it++){
	  std::stringstream ss;
	  ss << "shape"<< i++;
          i=i%3;
	  if(!viewer->updateSphere(pcl::PointXYZ((*it)->values[0],
						 (*it)->values[1],
						 (*it)->values[2]),
				   (*it)->values[3], 1,0,0,ss.str()))
	      
	    viewer->addSphere(pcl::PointXYZ((*it)->values[0],
					    (*it)->values[1],
					    (*it)->values[2]),
			      (*it)->values[3],
			      1,0,0,ss.str());
            
	  cout << (*it)->values[3] << endl;
	}
      }
      if(vis_cylinders){
	boost::shared_ptr<std::vector<pcl::ModelCoefficients::Ptr> > 
	  cylinders(new std::vector<pcl::ModelCoefficients::Ptr>());
	vis_cylinders.swap(cylinders);
       	
	static int i = 0;
	for(std::vector<pcl::ModelCoefficients::Ptr>::iterator it = 
	      cylinders->begin();
	    it != cylinders->end();
	    it++)
	  {
	    std::stringstream ss;
	    ss << "cylinders"<< i++;
            i = i %10;
	    viewer->removeShape(ss.str());
	    viewer->addCylinder(**it, ss.str());
				
	  }
      }
		data_rdy = false;
    }
}

void reshape(int w, int h) {
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective (60.0, (GLfloat) w/(GLfloat)h, 1, 5000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle() {
	glutPostRedisplay();
}

void display() {
  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //set matrix to default
  glLoadIdentity();
  //set what we are looking at
  gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, 0.0, 1.0, 0.0);
  if (items){
    boost::shared_ptr<std::vector<boost::shared_ptr<PRIMITIVE> > > temp_items = items;
    for (int i = 0; i < temp_items->size(); i++){
      (*temp_items)[i]->draw_p();
//      cout << "drawing items " << i << endl;
    }
  }
  glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y) {
	switch(key)
	{
		case 27:
			GO = false;
			exit(0);
			break;
	}
}

void glutThread() {
 
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glEnable(GL_DEPTH_TEST);
  glutInitWindowSize(600,600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Final OpenGL");

  glutDisplayFunc(display);
//  glutMotionFunc(mouseMotion);
//  glutMouseFunc(mouseButton);
//  glutMotionFunc(mouseMotion);
//  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);
  glutReshapeFunc(reshape);
  glutMainLoop();
}

int main (int argc, char** argv)
{
	glutInit(&argc, argv);
  //More Kinect Setup
  static std::vector<uint16_t> mdepth(640*480);
  static std::vector<uint8_t> mrgb(640*480*4);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


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
  /*
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (6);
    reg.setMinClusterSize (600);
  */




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
  seg_plane.setMaxIterations (500);

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg_cylinder;
  seg_cylinder.setModelType (pcl::SACMODEL_CYLINDER);
  seg_cylinder.setMethodType (pcl::SAC_RANSAC);
  seg_cylinder.setNormalDistanceWeight (10);
  seg_cylinder.setMaxIterations (1000);
  seg_cylinder.setDistanceThreshold (35);
  seg_cylinder.setRadiusLimits (0, 300);
  //  seg_cylinder.setOptimizeCoefficients (true);

 
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg_sphere;
  seg_sphere.setModelType (pcl::SACMODEL_SPHERE);
  seg_sphere.setMethodType (pcl::SAC_RANSAC);
  seg_sphere.setNormalDistanceWeight (0.0001);
  seg_sphere.setMaxIterations (100000);
  seg_sphere.setDistanceThreshold (5);
  seg_sphere.setRadiusLimits (0, 200); 
  seg_sphere.setOptimizeCoefficients (true);
 
  	boost::thread thrd1(
    boost::bind(&glutThread));
  boost::thread thrd2(
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
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  
  pcl::ConvexHull<pcl::PointXYZRGB> cHull;
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voronoi_centers1 (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract_neg;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::ExtractIndices<pcl::Normal> extract_neg_normal;
 
  extract_neg.setNegative(true);
  extract_neg_normal.setNegative(true);
  
  printf("Start the main loop.\n");
  while (GO){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
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

    vox.setInputCloud (cloud);
    vox.filter (*cloud_filtered);
      
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);
    eyeX = centroid[0];
    eyeY = centroid[1];
    eyeZ = centroid[2] - 2000;
    centerX = centroid[0];
    centerY = centroid[1];
    centerZ = centroid[2];
    
    cout << "centroid " << centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;
    cout << "eye "  << eyeX << " " << eyeY << " " << eyeZ << endl;
    cout << "center "  << centerX << " " << centerY << " " << centerZ << endl;
 
    std::vector <pcl::PointIndices> clusters;
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setRadiusSearch(100);

    normal_estimator.setInputCloud (cloud_filtered);
    normal_estimator.compute (*normals);

    /* Need to make sure normals aren't NaN
       for (i = 0; i < normals->points.size (); ++i)
       {
       cout << "Normal "
       << normals->points[i].normal_x << " " << normals->points[i].normal_y << " " << normals->points[i].normal_z<< endl;
       }

    */

    boost::shared_ptr<std::vector<pcl::PolygonMesh::Ptr> > 
      meshes(new std::vector<pcl::PolygonMesh::Ptr>());
    boost::shared_ptr<std::vector<pcl::ModelCoefficients::Ptr> > 
      spheres(new std::vector<pcl::ModelCoefficients::Ptr>());
    boost::shared_ptr<std::vector<pcl::ModelCoefficients::Ptr> > 
      cylinders(new std::vector<pcl::ModelCoefficients::Ptr>());
 
    int inlier_size;

    boost::shared_ptr<vector <boost::shared_ptr<PRIMITIVE> > > items_temp(new vector<boost::shared_ptr<PRIMITIVE> >); 
    
    
    outliers = cloud_filtered;
    
    do{
      pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

      proj.setInputCloud(outliers);
      seg_plane.setInputCloud(outliers);
      seg_plane.segment (*inliers, *coefficients);
      inlier_size = inliers->indices.size();

      if(inlier_size > threshold){
	proj.setIndices(inliers);
	proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_proj);
        cHull.setInputCloud(cloud_proj);
        cHull.reconstruct (*mesh);
        meshes->push_back(mesh);
	
	extract_neg.setInputCloud(outliers);
	extract_neg.setIndices(inliers);
	extract_neg.filter(*outliers);
	
	extract_neg_normal.setInputCloud(normals);
	extract_neg_normal.setIndices(inliers);
	extract_neg_normal.filter(*normals);
      }
    }while(inlier_size > threshold);

    for(std::vector<pcl::PolygonMesh::Ptr>::iterator it = meshes->begin();
	    it != meshes->end();
	    it++){
	
      std::vector<Vec3f> points;
      pcl::PointCloud<pcl::PointXYZ> temp_cloud;
      pcl::fromPCLPointCloud2((*it)->cloud, temp_cloud);
      points.reserve(temp_cloud.size());
      for (int j = 0; j < temp_cloud.size(); j++){
        points.push_back(Vec3f(temp_cloud.at(j).x, temp_cloud.at(j).y, temp_cloud.at(j).z));
      }
     boost::shared_ptr<PLANE> temp(new PLANE(points, Vec3f(0,1,0), 0, 0, 0));
      items_temp->push_back(temp);
    }
   
    i = 0;

    do{
      cout << "trying to segment cylinder: "<< outliers->size() << endl;
      pcl::ModelCoefficients::Ptr coefficients_cylinder 
	(new pcl::ModelCoefficients);

      extract.setInputCloud(outliers);
      seg_cylinder.setInputCloud(outliers);
      seg_cylinder.setInputNormals(normals);
      seg_cylinder.segment (*inliers, *coefficients_cylinder);
      inlier_size = inliers->indices.size();
      

      if(inlier_size > 70){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud 
	  (new pcl::PointCloud<pcl::PointXYZRGB> ());
	extract.setIndices(inliers);
	extract.filter(*incloud);
	
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	/*
	transform.translation() << 
	  -coefficients_cylinder->values[0],
	  -coefficients_cylinder->values[1],
	  -coefficients_cylinder->values[2];
	transform.quaternion() <<
	  1,1,1,0;
	  */
	float tx = -coefficients_cylinder->values[0];
	float ty = -coefficients_cylinder->values[1];
	float tz = -coefficients_cylinder->values[2];
	float rx = coefficients_cylinder->values[3];
	float ry = coefficients_cylinder->values[4];
	float rz = coefficients_cylinder->values[5];
	/*
	float roll  = 0;//std::atan(-rx/ry);
	float pitch = 0;
	float yaw   = 0;//sqrt(rx*rx+ry*ry)/rz;
	*/

	//	transform = transform
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud 
	  (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud2 
	  (new pcl::PointCloud<pcl::PointXYZRGB> ());

	transform.translation() << tx, ty,tz;
	//	transform.rotate (rollAngle);
	//transform.rotate (yawAngle);
	//transform.rotate (pitchAngle);
	transform = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(rx,ry,rz), Eigen::Vector3f(0,0,1)) * transform;

	pcl::transformPointCloud<pcl::PointXYZRGB> (*incloud, *transformed_cloud, transform);
	cout << "cylinders:" << i++ << " " << transformed_cloud->size()<< endl;
	pcl::PointXYZRGB pmin;
	pcl::PointXYZRGB pmax;

	pcl::getMinMax3D<pcl::PointXYZRGB>(*transformed_cloud, pmin,pmax);
	
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	transform2.translation() << 0,0,-pmin.z;
	float height = pmax.z - pmin.z;

	Eigen::Affine3f transform3 = transform2 * transform;
        Eigen::Affine3f tInv = transform3.inverse();
	cout << tInv(0,0) << " " << tInv(1,0) << " " << tInv(2,0) << " " << tInv(3,0) << endl;
	cout << tInv(0,1) << " " << tInv(1,1) << " " << tInv(2,1) << " " << tInv(3,1) << endl;
	cout << tInv(0,2) << " " << tInv(1,2) << " " << tInv(2,2) << " " << tInv(3,2) << endl;
	cout << tInv(0,3) << " " << tInv(1,3) << " " << tInv(2,3) << " " << tInv(3,3) << endl;
        cout << "Cylinder height" << height << endl;
/*	
	pcl::transformPointCloud<pcl::PointXYZRGB> (*incloud,
						    *transformed_cloud2, 
						    transform3);
	vis_cloud = transformed_cloud2;
*/	

	cylinders->push_back(coefficients_cylinder);

        GLdouble m[16]= {tInv(0,0),tInv(1,0),tInv(2,0),0,
                         tInv(0,1),tInv(1,1),tInv(2,1),0,
                         tInv(0,2),tInv(1,2),tInv(2,2),0,
                         tInv(0,3),tInv(1,3),tInv(2,3),1};
        float r = coefficients_cylinder->values[6];
        boost::shared_ptr<CYLINDER> temp( new CYLINDER(Vec3f(0,0,1),r,height,m));
        items_temp->push_back(temp);
 

	extract_neg.setInputCloud(outliers);
	extract_neg.setIndices(inliers);
	extract_neg.filter(*outliers);

	extract_neg_normal.setInputCloud(normals);
	extract_neg_normal.setIndices(inliers);
	extract_neg_normal.filter(*normals);
      }      
     
    }while(inlier_size > 70);
    i = 0;
    do{
      pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
      seg_sphere.setInputCloud(outliers);
      seg_sphere.setInputNormals(normals);
      seg_sphere.segment (*inliers, *coefficients_sphere);
      inlier_size = inliers->indices.size();
     

      if(inlier_size > 50){
	cout << "sphere:" << i++ << " " << inlier_size << endl;
	spheres->push_back(coefficients_sphere);

     
        Vec3f center = Vec3f(coefficients_sphere->values[0],
  	  coefficients_sphere->values[1],coefficients_sphere->values[2]);
        boost::shared_ptr<SPHERE> temp( new SPHERE(center,
	  coefficients_sphere->values[3], Vec3f(1,0,0)));
        items_temp->push_back(temp);
  

	extract_neg.setInputCloud(outliers);
	extract_neg.setIndices(inliers);
	extract_neg.filter(*outliers);

	extract_neg_normal.setInputCloud(normals);
	extract_neg_normal.setIndices(inliers);
	extract_neg_normal.filter(*normals);
      }      
     
    }while(inlier_size > 50);


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = outliers;//cloud;//reg.getColoredCloud ();
    vis_normals = normals;

    vis_cloud = colored_cloud;
    vis_meshes = meshes;
    if(spheres->size())
      vis_spheres = spheres;
    if(cylinders->size())
      vis_cylinders = cylinders;
 
    items = items_temp;  
  
    data_rdy = true;
    condition.notify_one();
   
  }
  thrd1.join();
  thrd2.join();
  device->stopVideo();
  device->stopDepth();
  //devicetwo->stopVideo();
  //devicetwo->stopDepth();
  return 0;
}
