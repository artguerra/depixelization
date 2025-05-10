#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include <opencv2/opencv.hpp>

#include "Canvas.h"

class Application {
 public:
  void render();
  bool loadImage(char* path);

  // camera controls
  void setWindowSize(int width, int height) { m_canvas.setViewportSize(width, height); }
  void setMousePos(double x, double y) { m_canvas.setMousePos(x, y); }
  void zoom(float factor) { m_canvas.zoom(factor); }
  void pan(float dx, float dy) { m_canvas.pan(dx, dy); }

 private:
  cv::Mat m_image;
  Canvas m_canvas;
};

#endif  // __APPLICATION_H__
