#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include <opencv2/opencv.hpp>

#include "Canvas.h"

class Application {
 public:
  void render();
  bool loadImage(char* path);
  void setWindowSize(int width, int height);

 private:
  cv::Mat m_image;
  Canvas m_canvas;
};

#endif  // __APPLICATION_H__
