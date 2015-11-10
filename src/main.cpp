#include <iostream>
#include <thread>

#include <chrono>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>

#include "pcl.hpp"
#include "opencv.hpp"
#include "freenect.hpp"

using libfreenect2::Freenect2;
using libfreenect2::Freenect2Device;

void renderLoop()
{
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
  Freenect2 freenect2;
  Freenect2Device *dev;
  createFreenectDev(freenect2, &dev);

  if (!dev) { std::cout << "no device connected!" << std::endl; return; }

  // freenectCaptureLoop(dev, renderWithOpencv);

  initPclViewer();
  freenectCaptureLoop(dev, renderWithPclViewer);
}

// void renderLoop()
// {
//   libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
//   Freenect2 freenect2;
//   Freenect2Device *dev;
//   createFreenectDev(freenect2, &dev);

//   if (!dev) { std::cout << "no device connected!" << std::endl; return; }

//   std::chrono::system_clock clock;
//   std::chrono::system_clock::time_point lastTime = clock.now();

//   freenectCaptureLoop(dev, [&](
//     libfreenect2::Frame *rgb,
//     libfreenect2::Frame *ir,
//     libfreenect2::Frame *depth,
//     libfreenect2::Registration *registration,
//     libfreenect2::Frame *undistorted,
//     libfreenect2::Frame *registered) -> bool
//     {
      
//       std::chrono::system_clock::time_point now = clock.now();
//       std::chrono::milliseconds duration =
//         std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime);
//       lastTime = now;
//       std::cout << "got freenect input " << duration.count() << std::endl;

//       std::cout << "rgb: " << rgb->width << "x " << rgb->height << "x " << rgb->bytes_per_pixel << std::endl;
//       std::cout << "ir: " << ir->width << "x " << ir->height << "x " << ir->bytes_per_pixel << std::endl;
//       std::cout << "depth: " << depth->width << "x " << depth->height << "x " << depth->bytes_per_pixel << std::endl;

//       return false;
//     });
// }

int main(int argc, char *argv[])
{

  // std::thread renderThread(renderLoop);
  // renderThread.join();
  renderLoop();

  return 0;
}
