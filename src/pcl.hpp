#ifndef KINECT_TEST_PCL_INCLUDED_H_
#define KINECT_TEST_PCL_INCLUDED_H_

#include <libfreenect2/registration.h>
#include <libfreenect2/libfreenect2.hpp>

#include <pcl/io/pcd_io.h>

void writePointCloud (const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertFreenectFrameToPointCloud(
  libfreenect2::Registration* registration,
  libfreenect2::Frame* undistorted,
  libfreenect2::Frame* registered);

void initPclViewer();

bool renderWithPclViewer(
  libfreenect2::Frame* rgb,
  libfreenect2::Frame* ir, 
  libfreenect2::Frame* depth,
  libfreenect2::Registration* registration,
  libfreenect2::Frame *undistorted, 
  libfreenect2::Frame *registered);


#endif  // KINECT_TEST_PCL_INCLUDED_H_