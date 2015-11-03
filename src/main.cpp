#include <iostream>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::shared_ptr;
using libfreenect2::Freenect2;
using libfreenect2::Freenect2Device;
using libfreenect2::Registration;

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void
createFreenectDev(
  Freenect2 &freenect2,
  Freenect2Device *dev,
  Registration *registration)
{
  if (freenect2.enumerateDevices() == 0) return;
  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  libfreenect2::OpenCLPacketPipeline *pipeline = new libfreenect2::OpenCLPacketPipeline();
  dev = freenect2.openDevice(serial, pipeline);
  // registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

void
stopFreenectDev(libfreenect2::Freenect2Device *dev)
{
  dev->stop();
  dev->close();
}

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void
showInOpencvLoop(Freenect2Device *dev)
{
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  Registration* registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  size_t framecount = 0;

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  while(1)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration->apply(rgb, depth, &undistorted, &registered);

    framecount++;
    if (framecount % 100 == 0)
      std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;

    cv::Mat rgbMat(rgb->height, rgb->width, CV_8UC4, rgb->data);
    resize(rgbMat, rgbMat, cv::Size(), 0.5, 0.5);

    cv::imshow("rgb", rgbMat);
    cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 500.0f);
    cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);

    int key = cv::waitKey(1);

    listener.release(frames);

    bool protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
    if (protonect_shutdown) break;

  }
  stopFreenectDev(dev);
  delete registration;
}

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void
writePointCloud (const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::io::savePCDFileASCII(fileName, *cloud);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer>
rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
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
  libfreenect2::FrameMap* frames,
  Registration* registration,
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

void
showInPclViewerLoop(Freenect2Device* dev)
{

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  Registration* registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = createSampleRGBPointCloud();

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  while(!viewer->wasStopped())
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    registration->apply(rgb, depth, &undistorted, &registered);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = convertFreenectFrameToPointCloud(&frames, registration, &undistorted, &registered);

    viewer->removePointCloud("kinect");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbField(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgbField, "kinect");

    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    listener.release(frames);
  }
  stopFreenectDev(dev);
  delete registration;
}


// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

int main(int argc, char *argv[])
{
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
  libfreenect2::Freenect2 freenect2;
  Freenect2Device *dev;
  Registration *registration;
  createFreenectDev(freenect2, dev, registration);

  if (dev == 0) { std::cout << "no device connected!" << std::endl; return -1; }
  // showInOpencvLoop(dev);

  showInPclViewerLoop(dev);

  return 0;
}
