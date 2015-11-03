#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>

#include "pcl.hpp"
#include "opencv.hpp"
#include "freenect.hpp"

using libfreenect2::Freenect2;
using libfreenect2::Freenect2Device;

int main(int argc, char *argv[])
{
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
  Freenect2 freenect2;
  Freenect2Device *dev;
  createFreenectDev(freenect2, &dev);

  if (dev == 0) { std::cout << "no device connected!" << std::endl; return -1; }

  // freenectCaptureLoop(dev, renderWithOpencv);

  initPclViewer();
  freenectCaptureLoop(dev, renderWithPclViewer);

  return 0;
}
