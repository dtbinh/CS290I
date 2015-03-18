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
#define GLM_FORCE_RADIANS
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GLFW/glfw3.h> 

#include "primitives.hpp"
#include <iostream>
#include "freenect_fix.hpp"
#include <libfreenect/libfreenect_registration.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <algorithm>
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
#include <pcl/surface/convex_hull.h>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include "common/shader.hpp"
#include "common/texture.hpp"
#include "common/controls.hpp"


//Global for main loop so openGL
bool GO = true;
bool data_rdy = false;
bool data_rdy1 = false;
bool ONCE = true;
boost::mutex mtx;
boost::mutex mtx1;
boost::condition_variable condition;

//global vars for camera (GL)
GLdouble eyeX = 0.0;
GLdouble eyeY = 0.0;
GLdouble eyeZ = 5.0;
GLdouble centerX = 0.0;
GLdouble centerY = 0.0;
GLdouble centerZ = 0.0;

boost::shared_ptr<std::vector<boost::shared_ptr<PRIMITIVE> > > items;

//particle globals
static GLfloat* g_particule_position_size_data;
static GLubyte* g_particule_color_data;
GLuint particles_position_buffer;
GLuint particles_color_buffer;
GLuint programID;
GLuint Texture;
GLuint TextureID;
GLuint CameraRight_worldspace_ID, CameraUp_worldspace_ID;
GLuint ViewProjMatrixID;
GLuint billboard_vertex_buffer;

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

/****Particle class/overhead****/
// CPU representation of a particle
struct Particle{
	glm::vec3 pos, speed;
	unsigned char r,g,b,a; // Color
	float size, angle, weight;
	float life; // Remaining life of the particle. if <0 : dead and unused.
	float cameradistance; // *Squared* distance to the camera. if dead : -1.0f

	bool operator<(const Particle& that) const {
		// Sort in reverse order : far particles drawn first.
		return this->cameradistance > that.cameradistance;
	}
};

const int MaxParticles = 100000;
Particle ParticlesContainer[MaxParticles];
int LastUsedParticle = 0;

// Finds a Particle in ParticlesContainer which isn't used yet.
// (i.e. life < 0);
int FindUnusedParticle(){

	for(int i=LastUsedParticle; i<MaxParticles; i++){
		if (ParticlesContainer[i].life < 0){
			LastUsedParticle = i;
			return i;
		}
	}

	for(int i=0; i<LastUsedParticle; i++){
		if (ParticlesContainer[i].life < 0){
			LastUsedParticle = i;
			return i;
		}
	}

	return 0; // All particles are taken, override the first one
}

void SortParticles(){
	std::sort(&ParticlesContainer[0], &ParticlesContainer[MaxParticles]);
}

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
	boost::mutex::scoped_lock lock(mtx);
  while (!viewer->wasStopped ())
  {  
		while(!data_rdy)
    	condition.wait(lock);
    viewer->spinOnce ();
    if(vis_cloud){
			printf("vis_cloud\n");
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
      }
    }
		data_rdy = false;
  }
}

void reshape(int w, int h) {
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective (60.0, (GLfloat) w/(GLfloat)h, 0.01, 200.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle() {
	glutPostRedisplay();
}
double lastTime = 0;
void display() {
	boost::mutex::scoped_lock lock(mtx1);
	while(!data_rdy1)
		condition.wait(lock);
  /**** BEGIN ONCE ****/

	if(ONCE)
	{
		GLuint VertexArrayID;
		glGenVertexArrays(1, &VertexArrayID);
		glBindVertexArray(VertexArrayID);

		// Create and compile our GLSL program from the shaders
		programID = LoadShaders( "shaders/Particle.vertexshader", "shaders/Particle.fragmentshader" );

		// Vertex shader
		CameraRight_worldspace_ID  = glGetUniformLocation(programID, "CameraRight_worldspace");
		CameraUp_worldspace_ID  = glGetUniformLocation(programID, "CameraUp_worldspace");
		ViewProjMatrixID = glGetUniformLocation(programID, "VP");

		// fragment shader
		TextureID  = glGetUniformLocation(programID, "myTextureSampler");

	
		g_particule_position_size_data = new GLfloat[MaxParticles * 4];
		g_particule_color_data         = new GLubyte[MaxParticles * 4];

		for(int i=0; i<MaxParticles; i++){
			ParticlesContainer[i].life = -1.0f;
			ParticlesContainer[i].cameradistance = -1.0f;
		}

		Texture = loadDDS("particle.DDS");

		// The VBO containing the 4 vertices of the particles.
		// Thanks to instancing, they will be shared by all particles.
		static const GLfloat g_vertex_buffer_data[] = { 
			 -0.5f, -0.5f, 0.0f,
			  0.5f, -0.5f, 0.0f,
			 -0.5f,  0.5f, 0.0f,
			  0.5f,  0.5f, 0.0f,
		};
		glGenBuffers(1, &billboard_vertex_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

		// The VBO containing the positions and sizes of the particles
		glGenBuffers(1, &particles_position_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
		// Initialize with empty (NULL) buffer : it will be updated later, each frame.
		glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW);
	
		// The VBO containing the colors of the particles
		glGenBuffers(1, &particles_color_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
		// Initialize with empty (NULL) buffer : it will be updated later, each frame.
		glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW);
		lastTime = glfwGetTime();
		ONCE = false;
	}



  boost::shared_ptr<std::vector<boost::shared_ptr<PRIMITIVE> > > temp_items;
	temp_items = items;
//  for (int j = 0; j < temp_items->size(); j++){
//   items[j]->draw_p();
//  }
	/****END ONCE****/
  
	
	glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
	glLoadIdentity();
  gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, 0.0, 1.0, 0.0);
  for (int j = 0; j < temp_items->size(); j++){
  //PARTICLES
		double currentTime = glfwGetTime();
		double delta = currentTime - lastTime;
		lastTime = currentTime;

		//computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();

		// We will need the camera's position in order to sort the particles
		// w.r.t the camera's distance.
		// There should be a getCameraPosition() function in common/controls.cpp, 
		// but this works too.
		glm::vec3 CameraPosition(glm::inverse(ViewMatrix)[3]);

		glm::mat4 ViewProjectionMatrix = ProjectionMatrix * ViewMatrix;

		// Generate 10 new particule each millisecond,
		// but limit this to 16 ms (60 fps), or if you have 1 long frame (1sec),
		// newparticles will be huge and the next frame even longer.
		int newparticles = (int)(delta*10000.0);
		if (newparticles > (int)(0.016f*10000.0))
			newparticles = (int)(0.016f*10000.0);
		
		for(int i=0; i<newparticles; i++){
			int particleIndex = FindUnusedParticle();
			ParticlesContainer[particleIndex].life = 5.0f; // This particle will live 5 seconds.
			ParticlesContainer[particleIndex].pos = glm::vec3(0,0,-20.0f);

			float spread = 1.5f;
			glm::vec3 maindir = glm::vec3(0.0f, 10.0f, 0.0f);
			// Very bad way to generate a random direction; 
			// See for instance http://stackoverflow.com/questions/5408276/python-uniform-spherical-distribution instead,
			// combined with some user-controlled parameters (main direction, spread, etc)
			glm::vec3 randomdir = glm::vec3(
				(rand()%2000 - 1000.0f)/1000.0f,
				(rand()%2000 - 1000.0f)/1000.0f,
				(rand()%2000 - 1000.0f)/1000.0f
			);
			
			ParticlesContainer[particleIndex].speed = maindir + randomdir*spread;

			ParticlesContainer[particleIndex].r = 255;
			ParticlesContainer[particleIndex].g = 0;
			ParticlesContainer[particleIndex].b = 0;
			ParticlesContainer[particleIndex].a = 255/3; //(rand() % 256) / 3;
			ParticlesContainer[particleIndex].size = (rand()%1000)/2000.0f + 0.1f;			
		}

		// Simulate all particles
		int ParticlesCount = 0;
		for(int i=0; i<MaxParticles; i++){
			Particle& p = ParticlesContainer[i]; // shortcut
			if(p.life > 0.0f){
				// Decrease life
				p.life -= delta;
				if (p.life > 0.0f){
					// Simulate simple physics : gravity only, no collisions
					p.speed += glm::vec3(0.0f,-9.81f, 0.0f) * (float)delta * 0.5f;
					p.pos += p.speed * (float)delta;
					p.cameradistance = glm::length2( p.pos - CameraPosition );
					//ParticlesContainer[i].pos += glm::vec3(0.0f,10.0f, 0.0f) * (float)delta;
					// Fill the GPU buffer
					g_particule_position_size_data[4*ParticlesCount+0] = p.pos.x;
					g_particule_position_size_data[4*ParticlesCount+1] = p.pos.y;
					g_particule_position_size_data[4*ParticlesCount+2] = p.pos.z;							   
					g_particule_position_size_data[4*ParticlesCount+3] = p.size;

					g_particule_color_data[4*ParticlesCount+0] = p.r;
					g_particule_color_data[4*ParticlesCount+1] = p.g;
					g_particule_color_data[4*ParticlesCount+2] = p.b;
					g_particule_color_data[4*ParticlesCount+3] = p.a;

				}else{
					// Particles that just died will be put at the end of the buffer in SortParticles();
					p.cameradistance = -1.0f;
				}
				ParticlesCount++;
			}
		}
		SortParticles();
		// Update the buffers that OpenGL uses for rendering.
		// There are much more sophisticated means to stream data from the CPU to the GPU, 
		// but this is outside the scope of this tutorial.
		// http://www.opengl.org/wiki/Buffer_Object_Streaming

		glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
		glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
		glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLfloat) * 4, g_particule_position_size_data);

		glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
		glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
		glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLubyte) * 4, g_particule_color_data);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		// Use our shader
		glUseProgram(programID);

		// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, Texture);
		// Set our "myTextureSampler" sampler to user Texture Unit 0
		glUniform1i(TextureID, 0);

		// Same as the billboards tutorial
		glUniform3f(CameraRight_worldspace_ID, ViewMatrix[0][0], ViewMatrix[1][0], ViewMatrix[2][0]);
		glUniform3f(CameraUp_worldspace_ID   , ViewMatrix[0][1], ViewMatrix[1][1], ViewMatrix[2][1]);

		glUniformMatrix4fv(ViewProjMatrixID, 1, GL_FALSE, &ViewProjectionMatrix[0][0]);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);
		
		// 2nd attribute buffer : positions of particles' centers
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
		glVertexAttribPointer(
			1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
			4,                                // size : x + y + z + size => 4
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// 3rd attribute buffer : particles' colors
		glEnableVertexAttribArray(2);
		glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
		glVertexAttribPointer(
			2,                                // attribute. No particular reason for 1, but must match the layout in the shader.
			4,                                // size : r + g + b + a => 4
			GL_UNSIGNED_BYTE,                 // type
			GL_TRUE,                          // normalized?    *** YES, this means that the unsigned char[4] will be accessible with a vec4 (floats) in the shader ***
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// These functions are specific to glDrawArrays*Instanced*.
		// The first parameter is the attribute buffer we're talking about.
		// The second parameter is the "rate at which generic vertex attributes advance when rendering multiple instances"
		// http://www.opengl.org/sdk/docs/man/xhtml/glVertexAttribDivisor.xml
		glVertexAttribDivisor(0, 0); // particles vertices : always reuse the same 4 vertices -> 0
		glVertexAttribDivisor(1, 1); // positions : one per quad (its center)                 -> 1
		glVertexAttribDivisor(2, 1); // color : one per quad                                  -> 1

		// Draw the particules !
		// This draws many times a small triangle_strip (which looks like a quad).
		// This is equivalent to :
		// for(i in ParticlesCount) : glDrawArrays(GL_TRIANGLE_STRIP, 0, 4), 
		// but faster.
		glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, ParticlesCount);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);

		glfwPollEvents();

  //PARTICLES END


//  for (int j = 0; j < items->size(); j++){
		(*temp_items)[j]->draw_p();
  }
  glutSwapBuffers();
	data_rdy1 = false;
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
  GLdouble m[16]= {1,0,0,0,
                   0,1,0,0,
                   0,0,1,0,
                   1,0,0,1};
  
  int capacity = 10;
  std::vector<Vec3f> points;
  points.reserve(capacity);
  for (int i = 0; i < capacity; i++){
    float bob = i*6.28/capacity;
    points.push_back(Vec3f(sin(bob),cos(bob),-1)); 
  }
  boost::shared_ptr<std::vector<boost::shared_ptr<PRIMITIVE> > > 
					temp_items(new std::vector<boost::shared_ptr<PRIMITIVE> >);
	boost::shared_ptr<PRIMITIVE> temp(new SPHERE(Vec3f(-1, 0, 0), .1,  Vec3f(1, 0, 0)));
	boost::shared_ptr<PRIMITIVE> temp2(new CYLINDER(Vec3f(0,0,1), .1, .1, m));
	boost::shared_ptr<PRIMITIVE> temp3(new PLANE(points, Vec3f(0,1,0), 0, 0, 0));


  temp_items->push_back (temp);  
  temp_items->push_back (temp2);  
  temp_items->push_back (temp3);  
  items = temp_items; 

  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glEnable(GL_DEPTH_TEST);
  glutInitWindowSize(600,600);
  glutInitWindowPosition(100, 500);
  glutCreateWindow("Final OpenGL");

  glutDisplayFunc(display);
//  glutMotionFunc(mouseMotion);
//  glutMouseFunc(mouseButton);
//  glutMotionFunc(mouseMotion);
//  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);
  glutReshapeFunc(reshape);

	glewExperimental = GL_TRUE;
	glewInit();

  glutMainLoop();
}
	
int main (int argc, char** argv)
{
	glutInit(&argc, argv);
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
  //pcl::SACSegmentation<pcl::PointXYZRGB> seg_plane;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg_plane;
  seg_plane.setOptimizeCoefficients (true);
  seg_plane.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_plane.setMethodType (pcl::SAC_RANSAC);
  seg_plane.setDistanceThreshold (0.03); //10
  seg_plane.setMaxIterations (100);
  
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg_cylinder;
  seg_cylinder.setModelType (pcl::SACMODEL_CYLINDER);
  seg_cylinder.setMethodType (pcl::SAC_RANSAC);
  seg_cylinder.setNormalDistanceWeight (0.1);
  seg_cylinder.setMaxIterations (10000);
  seg_cylinder.setDistanceThreshold (0.05);
  seg_cylinder.setRadiusLimits (0, 0.1);
  
	boost::thread thrd1(glutThread), thrd2(visualizerThread);
  //boost::thread thrd2(
  //  boost::bind(&visualizerThread));
	
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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);

  printf("Start the main loop.\n");
  while (GO){
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
      reg.extract (clusters);


      seg_cylinder.setInputCloud(cloud_filtered);
      seg_cylinder.setInputNormals (normals);
      seg_plane.setInputCloud(cloud_filtered);	
			seg_plane.setInputNormals(normals);
      proj.setInputCloud (cloud_filtered);

      i = 0;

      boost::shared_ptr<std::vector<pcl::PolygonMesh::Ptr> > 
         meshes(new std::vector<pcl::PolygonMesh::Ptr>());
      for(std::vector<pcl::PointIndices>::iterator
	    cluster = clusters.begin(); 
	  cluster != clusters.end();
	  cluster++, i++){
        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	
	pcl::PointIndices::Ptr clust(new pcl::PointIndices(*cluster));

	//seg_cylinder.setIndices(clust);
	seg_plane.setIndices(clust);

	float fsize = clust->indices.size();

	seg_plane.segment (*inliers_plane, *coefficients_plane);
	proj.setIndices(inliers_plane);
        proj.setModelCoefficients (coefficients_plane);
        proj.filter (*cloud_proj);
        cHull.setInputCloud(cloud_proj);
        cHull.reconstruct (*mesh);
        meshes->push_back(mesh);

	cout << "Cluster: "<< i << "Size: " << clust->indices.size() << endl  
	     << "Cylinder: "
	     << float(inliers_cylinder->indices.size())/fsize << endl
	     << "Plane " 
	     << float(inliers_plane->indices.size())/fsize
	     << endl;
      }
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      vis_cloud = colored_cloud;
      vis_meshes = meshes;
      //      cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			data_rdy = true;
			data_rdy1 = true;
			condition.notify_all();
  }
  thrd1.join();
	thrd2.join();
  device->stopVideo();
  device->stopDepth();
	
  //devicetwo->stopVideo();
  //devicetwo->stopDepth();
  return 0;
}
