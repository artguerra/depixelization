#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include <opencv2/opencv.hpp>

class Application {
  public:
    void render();
    void loadImage(const cv::Mat& image);
    void setWindowSize(int width, int height);
};

#endif  // __APPLICATION_H__