
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

using namespace pcl;
using namespace pcl::visualization;
using namespace std;

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    if (++count == 100) \
    { \
      double now = pcl::getTime (); \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

template <typename PointType>
class OpenNIFastMesh
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIFastMesh (const std::string& device_id = "")
      : device_id_(device_id), vertices_ ()
    {
      ofm.setTrianglePixelSize (3);
      ofm.setTriangulationType (pcl::OrganizedFastMesh<PointType>::QUAD_MESH);
    }
    
    void 
    cloud_cb (const CloudConstPtr& cloud)
    {
      // Computation goes here
      FPS_CALC ("computation");
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(*cloud, *point_cloud_ptr);

    
      // Prepare input
      ofm.setInputCloud (cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr ds (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (point_cloud_ptr);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*ds);

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
      seg.setInputCloud (ds);
      seg.segment (*inliers, *coefficients);

      // Extract the inliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
      extract.setInputCloud (ds);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);

     Eigen::Vector4f temp_centroid;
   //  boost::shared_ptr<Eigen::Vector4f> temp_centroid;
     //find centroid of data
     cout << "your" << endl;
     pcl::compute3DCentroid (*cloud_p, *temp_centroid);
     cout << "face" << endl;
     //save point
     //find normal at that point
     //create object at that point 

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloud_p);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);
    //  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
      boost::shared_ptr<pcl::PointCloud<pcl::Normal> > cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch (0.05);
      ne.compute (*cloud_normals1);


      // Store the results in a temporary object
      boost::shared_ptr<std::vector<pcl::Vertices> > temp_verts (new std::vector<pcl::Vertices>);
      ofm.reconstruct (*temp_verts);

      // Lock and copy
      {
        boost::mutex::scoped_lock lock (mtx_);
        vertices_= temp_verts;
        normals_=cloud_normals1;
      //  centroid_ = temp_centroid;
        cloud_ = cloud;
      }
    }

    void
    run (int argc, char **argv)
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIFastMesh::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
     
      view.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCL OpenNI Mesh Viewer"));

      interface->start ();
      
      CloudConstPtr temp_cloud;
      boost::shared_ptr<std::vector<pcl::Vertices> > temp_verts;
      boost::shared_ptr<pcl::PointCloud<pcl::Normal> > cloud_normals1;
    //  boost::shared_ptr<Eigen::Vector4f> temp_centroid;
      pcl::console::TicToc t1;

      while (!view->wasStopped ())
      {
        if (!cloud_ || !mtx_.try_lock ())
        {
          boost::this_thread::sleep (boost::posix_time::milliseconds (1));
          continue;
        }

        temp_cloud = cloud_;
        temp_verts = vertices_;
        cloud_normals1 = normals_;
      //  temp_centroid = centroid_;
        mtx_.unlock ();

        if (!view->updatePolygonMesh<PointType> (temp_cloud, *temp_verts, "surface"))
        {
          view->addPolygonMesh<PointType> (temp_cloud, *temp_verts, "surface");
        //  view->addSphere (pcl::PointXYZ::PointXYZ((*temp_centroid)[0],(*temp_centroid)[1],(*temp_centroid)[2]), 0.2, 0.5, 0.5, 0.0, "sphere"); 

        }

        FPS_CALC ("visualization");
        view->spinOnce (1);
      }

      interface->stop ();
    }

    pcl::OrganizedFastMesh<PointType> ofm;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    CloudConstPtr cloud_;
    boost::shared_ptr<std::vector<pcl::Vertices> > vertices_;
    boost::shared_ptr<pcl::PointCloud<pcl::Normal> > normals_;
 //   boost::shared_ptr<Eigen::Vector4f> centroid_;
    pcl::PolygonMesh::Ptr mesh_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    OpenNIFastMesh<pcl::PointXYZRGBA> v ("");
    v.run (argc, argv);
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIFastMesh<pcl::PointXYZ> v ("");
    v.run (argc, argv);
  }
  return (0);
}
