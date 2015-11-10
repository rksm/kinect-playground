#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>

#include "opencv.hpp"

// #include <chrono>
// std::chrono::system_clock cock;
// std::chrono::system_clock::time_point lastTime = cock.now();


bool renderWithOpencv(
  libfreenect2::Frame* rgb,
  libfreenect2::Frame* ir,
  libfreenect2::Frame* depth,
  libfreenect2::Registration* registration,
  libfreenect2::Frame *undistorted,
  libfreenect2::Frame *registered)
{
  // std::chrono::system_clock::time_point now = cock.now();
  // std::chrono::milliseconds duration =
  //   std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime);
  // lastTime = now;
  // std::cout << duration.count() << std::endl;

  cv::Mat rgbMat(rgb->height, rgb->width, CV_8UC4, rgb->data);
  resize(rgbMat, rgbMat, cv::Size(), 0.5, 0.5);

  cv::imshow("rgb", rgbMat);
  cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 500.0f);
  cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);

  int key = cv::waitKey(1);
  return (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
}

