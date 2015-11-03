#ifndef KINECT_TEST_OPENCV_H_INCLUDED_
#define KINECT_TEST_OPENCV_H_INCLUDED_

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>

bool renderWithOpencv(
  libfreenect2::Frame* rgb,
  libfreenect2::Frame* ir,
  libfreenect2::Frame* depth,
  libfreenect2::Registration* registration,
  libfreenect2::Frame *undistorted,
  libfreenect2::Frame *registered);


#endif  // KINECT_TEST_OPENCV_H_INCLUDED_