#include <iostream>
#include <chrono>
#include <thread>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::shared_ptr;

void
writePointCloud (const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::io::savePCDFileASCII(fileName, *cloud);
}


std::shared_ptr<pcl::visualization::PCLVisualizer>
rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
createSampleRGBPointCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZRGB point;
      point.x = 0.5 * cosf (pcl::deg2rad(angle));
      point.y = sinf (pcl::deg2rad(angle));
      point.z = z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;
  return point_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
convertFreenectFrameToPointCloud(
  libfreenect2::Registration* registration,
  libfreenect2::Frame* undistorted,
  libfreenect2::Frame* registered)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  float x, y, z, rgb;

  for (int c = 0; c < 512; c++)
  {
    for (int r = 0; r < 424; r++)
    {
      registration->getPointXYZRGB(undistorted, registered, r, c, x, y, z, rgb);
      if (std::isnan(x)) continue;
      
      pcl::PointXYZRGB point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.rgb = rgb;
      cloud->points.push_back (point);
    }
  }

  return cloud;
}


shared_ptr<pcl::visualization::PCLVisualizer> viewer;

void initPclViewer()
{
   viewer = shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters();
}

bool
renderWithPclViewer(
  libfreenect2::Frame* rgb,
  libfreenect2::Frame* ir, 
  libfreenect2::Frame* depth,
  libfreenect2::Registration* registration,
  libfreenect2::Frame *undistorted, 
  libfreenect2::Frame *registered)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = convertFreenectFrameToPointCloud(registration, undistorted, registered);

  viewer->removePointCloud("kinect");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbField(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgbField, "kinect");

  viewer->spinOnce(100);
  std::this_thread::sleep_for(std::chrono::microseconds(2000));

  return viewer->wasStopped();
}
